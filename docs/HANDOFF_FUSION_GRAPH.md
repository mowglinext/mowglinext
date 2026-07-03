# Handoff — `fusion_graph` (factor-graph localizer) + suppression de slam_toolbox

> ⚠️ **Document historique.** La migration FusionCore → iSAM2 décrite ici est terminée et en
> production. Pour comprendre le comportement actuel du localiseur (steady state),
> ouvrir [`wiki/Architecture.md`][archi] et la section [`Configuration.md` § fusion_graph][cfg].
> Cette page reste publiée comme trace de la transition (commits, choix de design,
> liste des helpers retirés) — utile pour archéologie git, pas pour onboarder.
>
> [archi]: https://github.com/mowglinext/mowglinext/wiki/Architecture#optional-factor-graph-localizer-fusion_graph
> [cfg]: https://github.com/mowglinext/mowglinext/wiki/Configuration#7-fusion_graph

> Document de passation pour reprise dans une nouvelle session Claude Code.
> **Branche de départ :** `feat/slam-rtk-fallback` (ou créer `feat/fusion-graph` from `main`).
> **Date :** 2026-04-28.
> **Auteur des décisions :** chauber@onewealthplace.com.

## État au 2026-04-28 (commits sur `feat/slam-rtk-fallback`)

**Toutes les étapes du plan sont livrées.** 14 commits, code compilé + smoke-testé live (robot stationné sur dock, pas de motion).

| Étape | État | Commit principal |
|-------|------|------------------|
| 0 — suppression slam_toolbox + helpers | ✅ live | `506edeb` |
| 1 — scaffolding `ros2/src/fusion_graph/` | ✅ live | `5349ad0` |
| 2 — wheel+IMU+GPS via iSAM2 (Pose2) | ✅ live | `5349ad0` + QoS fix `7940389` |
| 3 — facteurs LiDAR scan-matching | ✅ live | `f32abfd` |
| 4 — loop closure + persistance graphe | ✅ live | `2cd5ebd` |
| 5 — bascule `use_fusion_graph` toggle | ✅ live | `1c5abcd` |
| 6 — Huber robustifiers + /fusion_graph/diagnostics | ✅ live | `6d02b5b` |
| Build — GTSAM 4.3a1 from source | ✅ live | `a19e75f` |
| Docs — CLAUDE.md sections sensor fusion | ✅ | `24bfdcf` |

**Smoke tests effectués (sans motion, robot sur dock) :**
- `colcon build --packages-select fusion_graph` → succès, 7/7 unit tests pass (analytic vs numeric Jacobian + ScanMatcher recovery + persistance round-trip).
- Live run avec `/wheel_odom`, `/imu/data`, `/gps/fix` (RTK-Fixed), `/imu/cog_heading`, `/imu/mag_yaw` : `/odometry/filtered_map` à ~17 Hz, init seedé en ~250 ms, 0 erreur sur 20 s.
- Service `~/save_graph` round-trip OK : 24 KB graph + scans + meta, autoload au boot suivant logs `loaded persisted graph`.
- `/fusion_graph/diagnostics` publie à 1 Hz avec total_nodes / scans_attached / loop_closures / cov_xx-yy-yawyaw.

**À faire pour la mise en production :**
1. Rebuild image ROS2 (le Dockerfile build GTSAM 4.3a1 from source — ~30 min sur ARM, à amortir via cache CI).
2. Valider en sim avec `make sim use_fusion_graph:=true` (les paramètres `use_scan_matching` et `use_loop_closure` restent off par défaut → fusion_graph se comporte comme `ekf_map_node` 1-pour-1, prêt à comparer A/B contre `:=false`).
3. Première session de mowing réel avec `use_fusion_graph:=true use_scan_matching:=true` — monitorer `/fusion_graph/diagnostics` pour vérifier qu'iSAM2 reste sub-100 ms par tick à graphe plein.
4. Activer `use_loop_closure:=true` quand on aura confiance que les scan-matching factors sont stables — le comportement loop-closure peut casser un graphe si lc_max_rmse est mal réglé.
5. Câbler `~/save_graph` sur les transitions BT `RECORDING → autre` (côté `mowgli_behavior`) pour checkpoints automatiques.

**Limitations connues (à régler après validation terrain) :**
- Pas de marginalisation iSAM2 explicite — graphe grandit indéfiniment. À évaluer sur ARM RK3588 quand un graphe atteint 6000+ nodes (~10 min de mowing). Si saturation iSAM2 update > 100 ms, ajouter un fixed-lag smoother (`gtsam::IncrementalFixedLagSmoother`) pour ne garder que la fenêtre récente "dense" + une couche de loop-closure-anchor.
- Loop closure search est brute-force O(N) — sub-ms pour <1000 nodes mais devra passer en KD-tree si la session dépasse 3000 nodes.
- Cold-boot sans GPS ni carte persistée → init bloque. Si une carte `.graph` existe au boot, autoload bypass le seeding GPS/COG et la nav démarre immédiatement.
- TF `map → odom` lookup dans le callback timer — skip le tick si `odom → base_footprint` n'est pas frais. Surveiller les logs (`fusion_graph: TF ... not available`).
- Mow session monitor n'est pas encore étendu pour subscribe `/fusion_graph/diagnostics` — c'est un follow-up trivial dans `ros2/scripts/mow_session_monitor.py` (10 lignes de Python).

---

## 1. Contexte et décision

L'archi actuelle empile slam_toolbox sur un arbre TF parallèle, un EWMA d'alignement (`slam_pose_anchor_node`), et une réinjection comme `pose1` dans `ekf_map_node`. Cette approche (Option A — "slam_toolbox/EKF en parallèle") a été essayée plusieurs fois, est fragile (double arbre TF, feedback loops, gating EWMA, helper nodes multiples) et ne fournit pas une vraie continuité quand le RTK tombe sur des fenêtres longues.

**Décision :** on bascule sur **Option B — factor graph custom (GTSAM/iSAM2)**, qui est l'état de l'art (LIO-SAM, GLIM, autonomous driving stacks).

L'odométrie d'entrée est aujourd'hui quasi-parfaite (RTK + IMU + COG + wheel), donc l'ajout d'un graphe avec scan-matching LiDAR comme contraintes between/loop-closure permet de continuer à naviguer précisément quand le GPS est désactivé ou en RTK-Float prolongé.

---

## 2. Cible architecture (résumé exécutable)

**Un seul localizer.** Plus de slam_toolbox, plus de helper nodes parallèles, plus de réinjection EKF.

```
ekf_odom_node (inchangé)        →  publie  odom → base_footprint   (dead-reckoning continu, ne saute jamais)
fusion_graph_node (NOUVEAU)     →  publie  map  → odom              (à partir du graphe optimisé)
mowgli_map/map_server_node      →  publie  /map  OccupancyGrid des polygones d'aire (inchangé)
Nav2                            →  consomme map → odom → base_footprint comme avant
```

Le graphe contient :
- Variables : `X_k ∈ SE2` (pose 2D à 10 Hz), optionnellement biais gyro `b_k`.
- Facteurs :
  1. **Wheel between-factor** — preintégration `/wheel_odom` entre nœuds, covariance non-holonomique (vy serré).
  2. **IMU preintegrated factor** — gyro_z preintegrated (Forster 2017).
  3. **GNSS unary factor** sur `X_k` quand `NavSatFix.status == GBAS_FIX` — σ ~3 mm, lever-arm corrigé par yaw du nœud (variable du graphe, optimisée jointly).
  4. **GNSS unary robustifié (Huber/DCS)** quand RTK-Float — σ depuis covariance NavSatFix.
  5. **COG heading factor** — unary sur yaw, gated `|v| > 0.15 m/s`.
  6. **Mag yaw factor** — unary sur yaw, robustifié Huber, seulement si magnétomètre calibré.
  7. **Scan-matching between-factor** — ICP/NDT 2D entre scans à `X_k` et `X_{k-N}`.
  8. **Loop closure factor** — scan-matching contre carte de scans persistée.

`iSAM2` pour l'update incrémental temps-réel. Sliding window fixed-lag pour la performance, full graph persisté pour les loop closures.

---

## 3. Plan d'implémentation (ordre strict)

### Étape 0 — Suppression de slam_toolbox et helpers (faire en premier, propre)

Supprimer / désactiver :

- **Launch :**
  - `ros2/src/mowgli_bringup/launch/slam_fallback.launch.py` → supprimer
  - Toute inclusion de `slam_fallback.launch.py` depuis `navigation.launch.py` et `sim_full_system.launch.py` → retirer
- **Config :**
  - `ros2/src/mowgli_bringup/config/slam_toolbox.yaml` → supprimer
  - `ros2/src/mowgli_bringup/config/robot_localization.yaml` → retirer la source `pose1` (`/slam/pose_cov`) du `ekf_map_node` (laisser pose0 GPS + IMU + wheel comme aujourd'hui en attendant que `fusion_graph` prenne le relais ; OU retirer complètement `ekf_map_node` quand `fusion_graph` est prêt — voir étape 5)
- **Nodes mowgli_localization à supprimer :**
  - `scripts/slam_pose_anchor_node.py`
  - `scripts/slam_scan_frame_relay.py`
  - `scripts/wheel_odom_tf_node.py`
  - `scripts/gps_anchored_odom_node.py` (si plus utilisé)
  - Mettre à jour `CMakeLists.txt` et `package.xml` en conséquence
- **CLAUDE.md (à actualiser à la fin, après merge) :** supprimer toutes les sections "RTK fallback", "slam_toolbox", "parallel TF tree", "slam_pose_anchor", "EWMA". Réécrire pour refléter `fusion_graph_node`.
- **Cartes persistées :** `/ros2_ws/maps/slam.posegraph` et `/ros2_ws/maps/slam.data` deviennent obsolètes — `fusion_graph` aura son propre format (voir étape 4).

**Vérification après étape 0 :** le robot doit toujours mower correctement avec **uniquement** GPS RTK + IMU + wheel via les deux EKF (l'archi pre-slam_toolbox). Tester en sim, puis demander à l'utilisateur de tester sur robot avant de continuer.

### Étape 1 — Scaffolding du package `fusion_graph`

```
ros2/src/fusion_graph/
├── CMakeLists.txt           # ament_cmake, dep gtsam, rclcpp, tf2_ros, sensor_msgs, nav_msgs, geometry_msgs
├── package.xml
├── include/fusion_graph/
│   ├── factors.hpp          # custom factors (lever-arm GNSS, COG, mag)
│   ├── graph_manager.hpp    # iSAM2 wrapper, sliding window
│   └── fusion_graph_node.hpp
├── src/
│   ├── fusion_graph_node.cpp
│   ├── graph_manager.cpp
│   └── factors.cpp
├── config/
│   └── fusion_graph.yaml    # noise sigmas, sliding window length, gates
├── launch/
│   └── fusion_graph.launch.py
└── test/
    ├── test_factors.cpp
    └── test_graph_manager.cpp
```

**Dépendances système** (ajouter dans Dockerfile de l'image ros2 si pas déjà là) :
- `libgtsam-dev` (≥ 4.2). Si pas dispo en apt sur Kilted, build from source dans le Dockerfile (pin commit).
- `libsmall-gicp-dev` ou `small_gicp` from source — pour scan matching 2D rapide sur ARM.
- (Optionnel) `libpointmatcher-dev` comme alternative.

**À créer aussi :** `ros2/Makefile` cible `build-fusion-graph` pour rebuild rapide.

### Étape 2 — Wheel + IMU + GPS only (pas encore de LiDAR)

Objectif : reproduire le comportement de `ekf_map_node` actuel, mais via le graphe.

- Subscriber `/wheel_odom` (TwistStamped) → between-factor à chaque réception, preintégration.
- Subscriber `/imu/data` → preintégration gyro_z entre nœuds, biais comme variable.
- Subscriber `/gps/fix` → unary factor avec lever-arm. Lecture du datum depuis `mowgli_robot.yaml` comme `navsat_to_absolute_pose_node` actuel. Suppression à terme de `navsat_to_absolute_pose_node` (le facteur GTSAM remplace).
- Subscriber `/imu/cog_heading` et `/imu/mag_yaw` → unary yaw factors.
- iSAM2 update à 10 Hz, publish `map → odom` TF + `/odometry/filtered_map`.
- **Sliding window 30-60 s.** Marginalisation des nœuds plus anciens.
- Initialisation : attendre premier RTK-Fixed + COG valide pour ancrer `X_0`. Sinon, prior très lâche.

**Critère de réussite étape 2 :** en sim, robot fait un undock + mowing, comportement indistinguable de l'EKF map actuel. RTK forcé OFF pendant 30 s → le graphe maintient la pose avec dérive bornée par wheel+IMU (~quelques cm/s comme aujourd'hui).

### Étape 3 — LiDAR scan-matching factors

- Subscriber `/scan` (LaserScan).
- À chaque nœud du graphe (10 Hz), associer le scan le plus proche temporellement (TF query `base_footprint → lidar_link` au timestamp du scan, **pas du nœud**).
- Worker thread async (n'utilise pas le thread iSAM2) : pour chaque nouveau scan, lancer ICP 2D contre `X_{k-1}` (between consecutif) ET contre 1-2 nœuds plus anciens dans la fenêtre (between de stabilisation).
- Ajouter `BetweenFactor<Pose2>` avec covariance dérivée du fitness ICP.
- Utiliser `small_gicp` 2D (ou libpointmatcher).

**Critère de réussite étape 3 :** GPS forcé OFF pendant 5 minutes, robot fait un aller-retour de 20 m, dérive finale < 10 cm (le scan matching seul tient).

### Étape 4 — Loop closure + persistance

- À chaque nouveau nœud, KD-tree query sur poses du graphe full → candidats à `< 5 m` et `> 30 s` ago.
- Worker thread basse prio : ICP du scan courant contre scan du candidat. Si fitness OK → `BetweenFactor` loop closure.
- **Persistance :**
  - Sérialiser le graphe optimisé (`gtsam::NonlinearFactorGraph` + `Values`) au format binaire `.g2o` ou GTSAM natif.
  - Stocker scans associés sous `/ros2_ws/maps/fusion_graph/scans/<node_id>.bin`.
  - Checkpoint sur transitions `HIGH_LEVEL_STATE_RECORDING → autre` (comme `slam_map_persist_node` faisait pour slam_toolbox).
- Au boot, recharger le graphe et faire une relocalisation par scan-matching contre les K nœuds les plus proches (en partant du dernier GPS connu ou du dock).

**Critère de réussite étape 4 :** redémarrage robot dans un coin de la carte mowée hier → relocalisation < 5 s, navigation reprend sans perte de précision.

### Étape 5 — Suppression de `ekf_map_node`

Quand étapes 2-4 sont validées en sim ET sur robot pendant au moins une session de mowing complète :

- Retirer `ekf_map_node` de `robot_localization.yaml` et de `navigation.launch.py`.
- `fusion_graph_node` devient seul propriétaire de `map → odom`.
- `ekf_odom_node` reste tel quel (dead-reckoning local pur).

### Étape 6 — Robustifiers et tuning

- Huber/DCS sur GPS unary quand RTK-Float.
- Huber sur mag yaw factor.
- Gates de cohérence : si scan matching et GPS divergent de > 50 cm, downweight scan matching (probablement glissement) ou GPS (probablement multipath).
- **Mow session monitor à étendre** : ajouter `fusion_graph` métriques — taille du graphe, latence iSAM2, nb facteurs scan/GPS/mag actifs, marginales du dernier nœud.

---

## 4. Fichiers de référence dans le repo (avant nettoyage)

À lire pour comprendre l'archi actuelle avant de la démonter :

- `ros2/src/mowgli_bringup/config/robot_localization.yaml` — config dual EKF.
- `ros2/src/mowgli_bringup/launch/navigation.launch.py` — comment slam_fallback est inclus.
- `ros2/src/mowgli_bringup/launch/slam_fallback.launch.py` — TF tree parallèle.
- `ros2/src/mowgli_localization/scripts/slam_pose_anchor_node.py` — EWMA d'alignement, gating RTK.
- `ros2/src/mowgli_localization/scripts/cog_to_imu.py` — COG → yaw absolu (à conserver, sera consommé par fusion_graph).
- `ros2/src/mowgli_localization/scripts/navsat_to_absolute_pose_node.py` (s'il existe encore — vérifier) — lever-arm + datum, logique à porter en C++ dans le facteur GNSS GTSAM.
- `ros2/src/mowgli_localization/scripts/mag_yaw_publisher.py` — yaw mag tilt-compensé (à conserver).
- `install/config/mowgli/mowgli_robot.yaml` — datum GPS, lever-arm, calibrations IMU.

---

## 5. Tests et validation

- **Unit tests GTSAM** (`test/test_factors.cpp`) : chaque custom factor — GNSS lever-arm, COG, mag — testé avec valeurs analytiques.
- **Sim test** (`ros2/Makefile` cible `sim`) : scénario undock → record area → mow → dock, RTK ON full session.
- **Sim test RTK dropout** : même scénario, RTK forcé OFF entre t=60s et t=180s. Erreur finale vs ground truth Gazebo < 15 cm.
- **Mow session monitor** (`ros2/scripts/mow_session_monitor.py`, déjà existant) : capture obligatoire pour chaque test sur vrai robot, comparer aux sessions baseline.
- **E2E test** : `cd ros2 && make e2e-test`.

---

## 6. Pièges connus

1. **Initialisation au cold boot sans RTK** : le graphe a besoin d'un prior. Stratégie : si carte persistée présente → relocalisation par scan matching. Sinon → attendre RTK-Fixed + COG valide.
2. **Latence iSAM2** : peut atteindre 100-200 ms si le sliding window grossit. Solution : marginalisation agressive (fixed-lag smoother), scan matching async hors thread iSAM2.
3. **Compute ARM (RK3588)** : tester tôt sur device. Si saturation → réduire fréquence des nœuds à 5 Hz, ou downsample LiDAR.
4. **2D vs 3D** : rester en `Pose2` (SE2). Tout le terrain mower est plat à <5°. Évite 4× de coût.
5. **Lever-arm GPS dans le facteur** : le yaw est variable du graphe → écrire un `NoiseModelFactor1<Pose2>` custom qui calcule l'erreur `gps_meas - (X.translation() + R(X.theta()) * lever_arm)`. Jacobian analytique (sinon autodiff lent).
6. **Wheel non-holonomie** : le between-factor wheel doit avoir cov_yy >> cov_xx pour bien représenter "pas de mouvement latéral". Sinon le graphe accepte des dérives latérales irréalistes pendant les tournants.
7. **Topics conservés vs supprimés :**
   - **Conservés :** `/wheel_odom`, `/imu/data`, `/gps/fix`, `/imu/cog_heading`, `/imu/mag_yaw`, `/scan`, `/odometry/filtered_map`, `/gps/absolute_pose` (pour GUI).
   - **Supprimés :** `/slam/pose_cov`, `/scan_slam`, `/map_slam`, tout TF parallèle `*_wheels`.
8. **Ne pas casser Nav2 pendant la transition** : tant que `fusion_graph` n'est pas validé, garder `ekf_map_node` actif. Bascule en étape 5, pas avant.

---

## 7. Invariants à NE PAS toucher

- TF chain REP-105 : `map → odom → base_footprint → base_link → sensors`.
- `base_link` au centre des roues arrière (OpenMower convention).
- Cyclone DDS uniquement.
- Map frame = GPS frame (X=east, Y=north).
- Firmware = unique safety authority. Pas de blade safety dans `fusion_graph`.
- `ekf_odom_node` (local, wheel+gyro, `odom → base_footprint`) reste intouché.

---

## 8. Pour la prochaine session — instructions de démarrage

```
1. Ouvrir le repo : cd /home/ubuntu/mowglinext
2. Lire ce fichier : docs/HANDOFF_FUSION_GRAPH.md
3. Lire CLAUDE.md pour contexte robot (ignorer sections slam_toolbox / RTK fallback — sont à supprimer).
4. Créer la branche :
     git checkout main && git pull
     git checkout -b feat/fusion-graph
5. Faire l'étape 0 (suppression slam_toolbox + helpers), commiter, tester en sim.
6. Faire l'étape 1 (scaffolding fusion_graph), commiter.
7. Faire l'étape 2 (wheel+IMU+GPS only), commiter, tester en sim.
8. Demander validation utilisateur avant étape 3 (LiDAR).
9. Étapes 3-6 séquentielles, chaque étape commiter + sim test + demander validation.
10. À la fin, mettre à jour CLAUDE.md (sections : Architecture Invariants #1, ROS2 Specifics — sensor fusion, What NOT to Do — retirer points slam_toolbox).
```

**Commandes utiles** (depuis devcontainer) :
```bash
cd ros2 && make build              # build complet
cd ros2 && make build-pkg PKG=fusion_graph
cd ros2 && make sim                # simulation headless
cd ros2 && make e2e-test           # E2E
cd ros2 && make format             # clang-format
cd ros2 && make test               # unit tests
```

**Référentiels externes utiles :**
- GTSAM docs : https://gtsam.org/doxygen/
- LIO-SAM (référence d'archi) : https://github.com/TixiaoShan/LIO-SAM
- GLIM (référence moderne) : https://github.com/koide3/glim
- Forster preintegrated IMU : https://gtsam.org/tutorials/intro.html#magicparlabel-65557
- small_gicp : https://github.com/koide3/small_gicp

---

## 9. Notes finales

- L'utilisateur veut que **tout soit codé** par Claude dans la nouvelle session, pas juste planifié.
- Pas de rollback prématuré : si une étape casse, fix forward (cf. memory `feedback_dev_no_rollback`).
- Pas de motion test sans confirmation utilisateur (cf. memory `feedback_no_motion_when_remote`).
- Co-Authored-By ne va PAS dans les commits de ce repo (cf. CLAUDE.md "Commit Conventions").
- Branche `dev` pour itérer, merge `main` quand stable.
