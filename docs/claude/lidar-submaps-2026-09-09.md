# Scan-to-map et sous-cartes — 2026-09-09

Ce changement prolonge les corrections de précision du 8 septembre, dans le worktree `feat/lidar-map-anchor`. Aucun déploiement sur le robot.

## Architecture implémentée

- Suppression de l’ICP : recalage scan-to-scan, fermetures de boucle, relocalisation initiale, stockage et publication des scans ICP, API, tests spécifiques et paramètres associés. Les launchs, réglages GUI et installateur ne les proposent plus. Le nettoyage des anciennes clés enregistrées reste une migration active, pas une option de compatibilité.
- Les archives de poses `.graph`/`.meta` restent lisibles ; `.scans` n’est plus lu ni produit.
- Sous-cartes persistantes de 10 m, avec une fenêtre de 5 × 5 tuiles en RAM, résolution 0,1 m : 250 000 cellules locales au lieu de 640 000 pour l’ancienne grille fixe de 80 m. Hystérésis de 1 m aux frontières. Les coordonnées restent exprimées dans le datum ; aucun changement de propriétaire des TF.
- Les tuiles conservent exactement les log-odds et l’indicateur de cellule observée. Chargement/sauvegarde/effacement en tâche de fond, écritures par fichier temporaire puis renommage. La mémoire de cartographie reste bornée par les fenêtres locales ; le stockage disque croît avec les cellules visitées.
- Migration de `.lidarmap` avec contrôle du datum, résolution, dimensions et longueur réelle du fichier ; renommage en `.lidarmap.migrated` après succès. Effacer la carte annule aussi une sauvegarde antérieure en cours et empêche la remigration de l’ancien fichier actif.
- Un échec d’E/S conserve l’évidence précédente, expose `lidar_map_io_error` et espace les tentatives. Une fenêtre demandée indisponible ne produit aucune correction. Le service d’effacement annonce une demande asynchrone, pas une écriture déjà achevée.

## Calcul et confiance

`lidar_anchor_max_rate_hz=5`, calibration RTK de 10 s toutes les 60 s. Hors calibration sous Fixed, le PF ne calcule pas. Après perte de Fixed, il reprend à 15 s pour préparer l’application à partir de 20 s. Le mode shadow explicite autorise une calibration continue, toujours plafonnée. La publication de la grille locale et la reconstruction du champ de vraisemblance sont séparées : ce dernier est reconstruit seulement au prochain calcul utile.

La référence de dérive roues/IMU et la distance parcourue sont conservées pendant les pauses et changements de fenêtre. Chaque observation garde le timestamp du scan, son nœud historique et son décalage local. Les facteurs restent XY uniquement. Charge inconnue/active, délai après sortie du chargeur, fraîcheur, support du scan, dispersion et cohérence avec la dérive restent bloquants. Les paramètres invalides sont refusés au démarrage.

Une sous-carte étend la couverture, elle ne crée pas de repères. Un terrain sans retours exploitables repose sur GPS + roues/IMU ; les contrôles de support ne constituent pas une preuve générale d’observabilité sur toutes les géométries répétitives.

La navigation active l’ancrage par défaut uniquement si le LiDAR est présent. Le node et launch standalone restent en activation explicite. Une préférence installée `use_lidar_map_anchor=false` est conservée.

## Validation

Compilation Release de `fusion_graph` réussie dans l’image ROS épinglée `sha256:870c244d326b3aa6962bcba2e1acfdd59b9a913c5fad5be6d241f40e9bc82544`, utilisateur 501. Après suppression des résultats périmés des tests ICP retirés : **311 vérifications, 0 erreur, 0 échec, 70 skips cppcheck**. Deux démarrages avec cadence/seuil invalides sont explicitement refusés. Clang-format 18, contrôle contre le merge-base `origin/main`, et `git diff --check` passent. Les symboles ICP recherchés sont absents du binaire. Les sources du package correspondent exactement aux sources compilées.

Autres suites : 106 tests bringup ; Go API/providers ; compilation TypeScript et 8 tests Vitest ciblés ; 29 vérifications installateur sous Linux, tous réussis. ESLint sur les fichiers modifiés : aucune erreur, 92 avertissements existants. Le harnais installateur ne matérialise pas le YAML sur macOS ; le résultat Linux est celui retenu.

Les tests de stockage couvrent notamment un aller-retour à plus de 100 m et un redémarrage, coordonnées négatives, conservation exacte des observations, frontières/hystérésis, effacement pendant sauvegarde, fichiers corrompus, datum incompatible et migration de cartes anciennes au-delà de la fenêtre active.

## Rosbags et résultats

Sept replays achevés, dont six dans la série comparative finale. Aucun robot commandé. Les trois bags disponibles sont utilisés. Le parcours mobile `bag_full_20260908_1515` est rejoué 500 s, avec GPS, statut GPS et COG masqués selon leur timestamp entre 200 et 360 s (800 fixes retenus). La référence d’erreur est l’antenne GPS RTK retenue, avec bras de levier de 0,30 m ; aucune transformation n’est ajustée sur la trajectoire masquée. Le bag mobile ne contient pas de carte préchargée : la carte est construite avant la coupure, sans accès aux données futures.

Comparaison sur **793 timestamps communs**, de 201 à 359,5 s. La référence est la version corrigée du 8 septembre, **avec ICP déjà désactivé** : ce tableau mesure le changement de carte et d’ordonnancement, pas le gain supplémentaire de suppression de l’ICP.

| Version | Erreur P90 coupure | CPU sous RTK, % d’un cœur | CPU coupure, % d’un cœur | RSS maximal sous RTK |
|---|---:|---:|---:|---:|
| Référence | 30,9 cm | 4,05 % | 4,62 % | 46,8 Mo |
| Sous-cartes, essai 1 | 25,3 cm | 3,67 % | 4,33 % | 33,9 Mo |
| Sous-cartes, essai 2 | 21,7 cm | 3,58 % | 4,60 % | 34,3 Mo |

Le CPU total sous RTK diminue de **9,4 à 11,5 %** ; pendant la coupure, l’écart est plus faible. Les mesures viennent du temps CPU du processus dans les conteneurs, replays à vitesse réelle, phases RTK 40–180 s et coupure 220–350 s. Elles ne remplacent pas un profil CPU sur le robot. Le graphe GPS/roues/IMU et la construction de carte continuent à coûter du CPU même quand le PF dort.

Un premier essai de référence, lancé pendant une compilation lourde, est conservé séparément (`submaps-before-full-1`) : P90 68,9 cm, CPU RTK 10,16 % d’un cœur. Il n’est pas utilisé comme référence de gain. La variabilité des essais précédents et les deux répétitions actuelles ne permettent pas de conclure à une précision garantie sur tous les terrains.

Autres contrôles :

- `bag_mow_lidar_20260907_0644`, 400 s, coupure artificielle identique : P90 **6,6 cm**, 31 observations mises en file. Ce segment est principalement stationnaire ; il ne remplace pas le test mobile.
- Même bag mobile, retours LiDAR remplacés par `inf` entre 180 et 380 s : **aucun facteur LiDAR**, alors qu’une carte existait déjà. P90 pendant la coupure **63,9 cm**, cohérent avec une dérive roues/IMU ; une carte ne remplace donc pas les observations manquantes.
- `bag_lidar_rtk_20260906_2128`, 400 s, perte GPS réelle : **aucun calcul PF ni facteur**, car le bag ne fournit pas l’état du chargeur. Il valide ce contrôle, pas la précision du scan-to-map pendant cette perte réelle.
- Les deux essais mobiles franchissent trois positions de fenêtre, toutes de 500 × 500 cellules. Aucun doublon de timestamp candidat, âge maximal 115 ms (limite 500 ms), aucune observation appliquée pendant le trou de scans ni pendant la perte naturelle de Fixed d’environ 16 s. Au retour du GPS artificiellement masqué, P90 sur 20 s : 2,7 / 2,5 cm, référence 2,8 cm.
- Les sauvegardes de fin de replay réussissent ; les nouvelles archives contiennent 8 à 11 tuiles selon le scénario, sans erreur d’E/S. L’extension au-delà de 80 m est validée par les tests de stockage, pas par ces bags dont le parcours est plus petit.

## Artefacts et reproduction

Racine : `/Users/cedric/Dev/git/mowglinext-lidar-review-20260908`.

- `submaps-comparison.json`, `submaps_precision_cpu.png` et `.pdf` : chiffres et figure comparative.
- `submaps_launch.py`, `submaps_run.sh`, `submaps_runtime.py`, `submaps_batch.py` : isolation réseau, utilisateur non-root, masquage, surveillance du processus, CPU et métadonnées des fenêtres. `SCAN_MODE=empty` active le scénario sans retours.
- `submaps_compare.py`, `submaps_plot.py`, `evaluate.py` : calcul des métriques sans réalignement.
- `submaps-tests-clean.log`, `submaps-validation.log`, `submaps-format.log` : validations.
- `submaps.patch`, `submaps-source-hashes.json`, `submaps-binary-hashes.json` : sources et binaires identifiés. Le patch combine les corrections du 8 septembre et ce changement, depuis `599c23be`.
- `runs/submaps-*` : manifestes, paramètres, poses, candidats, diagnostics, CPU, erreurs, sauvegardes de tuiles et journaux ; `pre-submaps-install` / `pre-submaps-source` conservent la référence.

La branche principale de travail `rb524` reste intacte. Les modifications sont dans `feat/lidar-map-anchor`, non commitées et non déployées.
