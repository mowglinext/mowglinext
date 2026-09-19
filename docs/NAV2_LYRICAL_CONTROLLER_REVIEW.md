# Nav2 Lyrical — revue des contrôleurs, et pourquoi FTC reste

> **Statut : courant (2026-09-16).** Décision d'architecture + relevé de ce que
> ROS 2 Lyrical / Nav2 1.5.1 apporte réellement à ce robot.
> Référence de comportement : [`CLAUDE.md`](../CLAUDE.md) invariants 5 et 8 et
> le codemap [`mowgli_nav2_plugins`](claude/codemaps/mowgli_nav2_plugins.md).

## Question posée

Après la migration Lyrical (voir [`ROS2_LYRICAL_MIGRATION.md`](ROS2_LYRICAL_MIGRATION.md)),
Nav2 propose-t-il quelque chose qui remplacerait `mowgli_nav2_plugins/FTCController`
sur le créneau `FollowCoveragePath` ?

## Réponse : non. Décision : FTC reste le contrôleur de tonte.

### Inventaire réel

Les contrôleurs présents sur la branche `lyrical` de `ros-navigation/navigation2`
(et dans le tag **1.5.1**, celui de nos paquets apt) :

| Paquet | Verdict pour la tonte |
|--------|-----------------------|
| `nav2_regulated_pure_pursuit_controller` | Non pour la tonte, **oui à évaluer sur le transit** (voir DWPP ci-dessous) |
| `nav2_mppi_controller` | Non — déjà essayé et reverté le 2026-06-19 |
| `nav2_graceful_controller` | Non — suiveur de pose, `motion_target_dist ≈ 1,0 m` |
| `nav2_dwb_controller` | Non — échantillonnage, même famille de défauts que MPPI |
| `nav2_rotation_shim_controller` | Déjà utilisé, en enveloppe du transit |

**Aucun nouveau contrôleur** n'est apparu en Lyrical. `nav2_following`
(`opennav_following`) est un serveur de suivi d'objet dynamique, pas un suiveur
de chemin. Vector Pursuit (Black Coffee Robotics), qui vise pourtant « high speed
path tracking and sharp turns », reste un paquet tiers **non publié pour
Lyrical** : l'adopter voudrait dire le compiler depuis les sources et le
maintenir nous-mêmes.

### Le seul vrai progrès : DWPP

Nav2 1.5.1 ajoute à RPP l'algorithme **Dynamic Window Pure Pursuit**
(Ohnishi & Takahashi, 2026) derrière `use_dynamic_window`. Au lieu d'appliquer la
courbure du carrot à la vitesse linéaire régulée, il cherche dans la fenêtre de
vitesses atteignable en une période de contrôle, sous contraintes
d'accélération, le couple (v, ω) qui suit le chemin au plus près — et ralentit
donc tout seul dans les virages serrés.

- **Pour la tonte : non.** DWPP reste une méthode à carrot. La coupe de virage
  est fixée par la distance de lookahead, et les arcs de demi-tour d'une route
  F2C font R = 0,20 m. FTC suit à 8–16 mm ; aucune poursuite pure ne tient ça sur
  des arcs de ce rayon.
- **Pour le transit : à tester.** Le S-weave lent, à l'échelle du mètre, observé
  sur les transits (terrain 2026-07-21) est le cycle limite de la poursuite pure :
  RPP dépasse chaque carrot parce que la boucle de lacet firmware traîne, puis
  corrige. Il a été amorti en **allongeant** le lookahead (`lookahead_time`
  1,5 → 2,0 ; `min_lookahead_dist` 0,30 → 0,45), au prix de la précision en
  virage près de la station. DWPP s'attaque au même cycle à la source.

  → Exposé comme `transit_dynamic_window` dans `mowgli_robot.yaml`, **OFF par
  défaut**, injecté dans `FollowPath.primary_controller.use_dynamic_window`.

### Pourquoi MPPI ne revient pas

Lyrical ajoute à MPPI le mode boucle ouverte, le support de dynamique, des
limites d'accélération asymétriques, des modèles de mouvement en plugin, des
validateurs de trajectoire et la compensation de retard. **La liste des critics
est inchangée** — aucun nouveau critic de suivi de chemin. Or le motif du revert
du 2026-06-19 (coupe des virages / boucles oméga aux demi-tours, louvoiement sur
les lignes droites) vient de l'échantillonnage et de l'arbitrage
`path_align` / `wz_std`. Rien dans ces ajouts n'y touche.

### Ce que FTC fait et qu'aucun plugin standard ne fait

Le suivi n'est qu'une partie des ~5 100 lignes de `mowgli_nav2_plugins`. Le reste
n'existe nulle part en amont : PRE_ROTATE, `forward_only`, rampe anti-patinage
(`ftc_stall.hpp`), ralentissement sous charge de lame (`ftc_blade_load.hpp`),
déviation latérale avec garde et masque de zone (`obstacle_deviation.cpp`), garde
de cul-de-sac, échappement arrière borné (`ftc_reverse_escape.hpp`), détecteur
d'oscillation, démarrage à l'indice 0 pour les anneaux fermés
(`ftc_start_index.hpp`), et la sémantique d'exception dont `FollowStrip` dépend
pour détourner ou sauter un segment. Changer de contrôleur voudrait dire
reconstruire tout cela autour de lui — pour un gain de suivi nul, puisque le
suivi n'est pas le problème terrain actuel.

## Ce qui a été récupéré de Lyrical (implémenté)

| Apport | Où |
|--------|-----|
| **`nav2_msgs/TrackingFeedback`** — le `controller_server` mesure lui-même l'erreur latérale signée, l'erreur de cap, l'indice courant et la longueur restante, pour **n'importe quel** contrôleur (FTC compris), et les publie sur `/tracking_feedback` (espace de noms racine — Nav2 crée ce publisher avec un nom relatif, il n'est donc pas sous `/controller_server/`) et dans le feedback de l'action `FollowPath`. Première mesure objective de la qualité de tonte du robot. | `mowgli_interfaces/path_tracking_stats.hpp` (réduction pure partagée) → statut `Path Tracking` dans `/diagnostics` (`diagnostics_node`, visible en GUI sans code GUI) + une ligne de log par sous-chemin dans `FollowStrip` |
| **DWPP** sur le transit | `transit_dynamic_window` (OFF) |
| **`nav2_controller::AxisGoalChecker`** — premier goal checker standard capable de survivre à un anneau fermé, parce que Lyrical passe désormais le plan transformé à `isGoalReached()` et que `path_length_tolerance` refuse l'arrivée tant qu'il reste du plan | déclaré comme `coverage_axis_goal_checker`, sélectionnable via `coverage_goal_checker_id` (défaut : notre `PathProgressGoalChecker`) |
| **`custom_inscribed_radius`** (couche d'inflation) — permet de **déclarer** le rayon inscrit au lieu de le dériver du rayon circonscrit du footprint (~0,597 m), donc de descendre `obstacle_inflation_radius` sans casser la bande 253 que lit le détecteur d'obstacles de FTC | `local_inflation_inscribed_radius` (−1,0 = comportement actuel) |

Les quatre sont **inertes par défaut** : une mise à jour ne change aucun
comportement tant qu'un opérateur n'a pas basculé le réglage (Réglages →
Avancé dans la GUI).

## Ce qui a été évalué et écarté

- **Vector Object Server** (rastérisation de polygones/cercles) pour les keepouts
  « dig » : **redondant**. `map_server_node` rastérise déjà ses polygones dans le
  masque keepout et expose déjà l'ajout/retrait dynamique par service
  (`~/apply_promoted_obstacle`, `~/promote_obstacle`, `~/discard_obstacle`).
  L'adopter ajouterait un nœud et un saut de message dans un chemin critique
  pour la sécurité (invariant 16), sans capacité nouvelle.
- **`AsymmetricInflationLayer`** (dépassement biaisé à gauche/droite) : écarté sur
  les deux costmaps. Sur le costmap **local**, il modifie le gradient dont FTC
  déduit la bande inscrite (seuils 253/254) — c'est un risque, pas une
  fonctionnalité, et FTC applique déjà son propre biais à gauche
  (`chooseDeviationSide`). Sur le costmap **global**, il ne ferait que reclasser
  des transits Smac, et personne n'a jamais signalé un contournement du mauvais
  côté.
- **`nav2_controller::AdaptiveToleranceGoalChecker`** : tolérances grossière puis
  fine sur l'approche du but. Sans objet ici — pour la tonte le cap final est
  ignoré (`yaw_goal_tolerance: 3.14`) et l'arrivée est jugée sur la progression.

## À surveiller (non fait)

- **`IsWithinPathTrackingBounds`** (nœud BT Lyrical) : laisserait l'arbre réagir
  à une dérive de suivi. À faire seulement une fois que les mesures
  `Path Tracking` de plusieurs tontes auront dit ce qu'est un seuil réaliste —
  poser le seuil avant d'avoir la donnée reproduirait l'erreur de seuil
  arbitraire du détecteur de dig.
- **`ZoneParameterFilter`** (filtre de costmap) : surcharges de paramètres par
  zone. Candidat naturel pour des réglages par zone de tonte.
- **`PauseResumeController`** (nœud BT) : à comparer à notre pause/reprise maison.

## Fields2Cover — état amont (2026-09-16)

Constat fait en même temps, sans rapport avec Nav2 :

- Notre épingle `884d895b` est la tête de la branche **`v3.0`, gelée depuis le
  2026-03-02**.
- Le développement est reparti sur `main`/`develop`, qui a publié **v2.1.0 le
  2026-09-03** et **rapatrie** les apports de v3 (#247, générateur de headland à
  largeur requise).
- Les correctifs depuis notre épingle (#222 segfault `FieldCoverage::computeCost`,
  #239 durcissement du graphe, #243 trois échecs silencieux, #249
  `Cells::splitByLine`, #219 mémoire) ne touchent **pas** notre chemin d'appel :
  on n'utilise que `hg::ConstHL`, `sg::BruteForce`, `rp::BoustrophedonOrder`,
  `obj::NSwath` et les types, et `splitByLine` ne sert qu'à la décomposition.
- Le repin coûte une adaptation d'API **à l'envers** : `generateHeadlandSwaths`
  rend `std::vector<F2CCells>` sur `main` contre `std::vector<F2CMultiLineString>`
  sur notre v3.
- Nouveauté à regarder : **`hg::CorridorHL`** (#243/#248) ouvre un corridor entre
  cellules décomposées, ce qui adresse nos transits lame-coupée entre
  sous-chemins.

Décision : repin sur `develop` dans une PR séparée, pas dans celle-ci.
