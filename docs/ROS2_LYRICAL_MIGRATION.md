# Migration ROS 2 Lyrical

Cette branche remplace Kilted par Lyrical dans les images ROS, GNSS et LiDAR,
l'environnement de développement, les scripts et la CI. Les conteneurs utilisent
Ubuntu 26.04 ; cela ne demande pas de réinstaller le système hôte du robot.
Les compilations s'exécutent sous un utilisateur non privilégié.

## Contrats adaptés

- C++20, cibles CMake exportées à la place de `ament_target_dependencies`,
  en-têtes TF2 `.hpp` et initialisation explicite des `NodeOptions` de fusion_graph
  pour GCC 15.
- `nav2_ros_common` et `nav2::LifecycleNode` pour FTC, le goal checker et le
  serveur de couverture ; nouvelles signatures Nav2 1.5.
- FTC reçoit toujours le **chemin complet** dans `newPathReceived`. Son curseur,
  ses pivots, ses PID et ses protections utilisent ce chemin, pas le chemin local
  tronqué transmis à `computeVelocityCommands`.
- Le goal checker conserve sa progression sur le chemin complet. La pose fournie
  dans le repère de la costmap est transformée vers le repère du chemin pour cette
  progression. `isGoalXYReached` retire uniquement la contrainte de lacet.
- Les réglages RPP vivent sous `FollowPath.primary_controller`, notamment
  `max_linear_vel`. Le lancement, les overlays LiDAR et les changements de vitesse
  du BT utilisent cette même adresse. La recherche de chemin vit dans `PathHandler` ; la tolérance TF du
  contrôleur et du path handler provient de `local_costmap.transform_tolerance`.
- Les deux `RoundRobin` de récupération conservent explicitement leur répétition
  avec `wrap_around="true"`.
- Les deux arbres de navigation utilisent `ValidatePath`, nouveau nom du nœud
  BT `IsPathValid` sous Lyrical ; le service ROS reste `is_path_valid`.
- Le serveur de couverture utilise les options standard de rétention des résultats
  de `SimpleActionServer`. Une rétention de résultat après achèvement ne constitue
  pas une limite de durée de planification.
- À l'arrêt, le nœud de comportement libère l'arbre puis rompt sa référence
  circulaire avec `BTContext`, après l'arrêt des callbacks. Les tests Lyrical ont
  révélé un SIGSEGV intermittent dans un thread Cyclone DDS pendant la fermeture
  des bibliothèques : le contexte conservait jusque-là le nœud et son listener TF.

## Dépendances reproductibles

Les paquets manquants du dépôt binaire Lyrical sont construits par
`ros2/scripts/build_lyrical_vendor.sh`, commun à Docker et à la CI, dans
`/opt/lyrical_vendor`. Le script ne modifie aucun sous-module du projet. Les constructions Docker
utilisent des caches par architecture pour conserver les compilations après un échec.
Les seuls correctifs appliqués à Fields2Cover sont des inclusions standard manquantes
(`<iomanip>` et `<algorithm>`), sans changement de géométrie.

| Dépendance | Révision |
|---|---|
| GTSAM | `4.3a1`, source, TBB désactivé comme avant migration, `-Wno-error=array-bounds` (voir CI) |
| Fields2Cover | `884d895b59192882476e986ba44ea9143a06a6a9`, v3, `/opt/fields2cover-300` |
| grid_map | `7ae24725d7effa0c22a23524b91b46bada5b2728` |
| Beluga | `22adc90e08cc7229cde042a756f044b908600fed` |
| Sophus | `de0f8d3d92bf776271e16de56d1803940ebccab9` |
| Nav2 Smac | `a6354f3f39d12c6e5c3e323f8021c831752edb23`, tag 1.5.1 |
| LD19/LDLiDAR | `4ee53a8b176037cf418a54b25007075bb2b1d3a0`, v0.3.0, adaptation lifecycle Nav2 |
| RPLidar | `24cc9b6dea97e045bda1408eaa867ce730fd3fc3` |
| STL27L | `bf668a89baf722a787dadc442860dcbf33a82f5a` |
| webots_ros2 | `db4a77c84ee91445d39de9a631e5b57a888814e2`, simulation seulement |

L'ancienne copie Fields2Cover v2, inutilisée, n'est plus construite. L'image
`ros2/Dockerfile.dev`, orpheline et divergente, est retirée ; utiliser
`.devcontainer/Dockerfile`. `nav2_bringup` n'est plus nécessaire : le lancement
Mowgli possède déjà sa configuration et ses nœuds. Le lisseur et le moniteur de
collision sont installés explicitement. Cyclone DDS reste obligatoire.
La simulation construit `webots_ros2_driver` et `webots_ros2_control` depuis la
même révision ; ses contrôleurs de roues et `libsndio7.0` sont installés explicitement.
`python3-pil` fournit Pillow, requis par l'importeur du superviseur Webots.
Les bibliothèques internes du pilote GNSS sont liées statiquement : le sous-module
épinglé n'installe pas leurs variantes partagées. La CI vérifie maintenant la
résolution des bibliothèques de tous les ELF de l'image GNSS après installation.
Le devcontainer extrait Go avec `tarfile` de Python : GNU tar sur Resolute échoue
sur les chemins imbriqués sous émulation AMD64/Rosetta
([incident OrbStack #2588](https://github.com/orbstack/orbstack/issues/2588)).
L'utilisateur `ubuntu` conserve l'accès au groupe `dialout` ; les opérations sur
`/dev/serial` du script de démarrage passent par `sudo`.

Le nom du contrôle protégé **`Build & Test (ROS2 kilted)` reste volontairement
identique** pour éviter de bloquer les PR sur `dev`. Il compile désormais Lyrical
sur `ubuntu-26.04`. Son nom historique n'indique plus la distribution testée.

Ce runner est la variante Ubuntu `amd64v3` : son GCC 15 cible `x86-64-v3` (AVX2)
par défaut, sans aucun `-march` de notre part. À ce niveau d'ISA, GCC 15 émet un
faux positif `-Warray-bounds` sur les chargements AVX d'Eigen (vecteurs de taille
fixe dans `EssentialMatrix.cpp` / `FundamentalMatrix.cpp`), que le `-Werror`
codé en dur de GTSAM 4.3a1 transforme en échec de compilation. Les trois recettes
GTSAM (`ros2-ci.yml`, `ros2/Dockerfile`, `.devcontainer/Dockerfile`) passent donc
`-DCMAKE_CXX_FLAGS=-Wno-error=array-bounds`, et `ros2/scripts/build_lyrical_vendor.sh`
fait de même pour grid_map (`grid_map_cmake_helpers` impose `-Werror` ; même faux
positif sur `grid_map::Position` dans `GridMap.cpp`) en ajoutant
`-isystem /usr/include/eigen3` : `nav2_smac_planner` (`-Werror` via `nav2_package`)
atteint Eigen à travers OMPL avec un simple `-I`, et sous `EIGEN_VECTORIZE_AVX2`
Eigen 3.4.0 lui-même n'est pas exempt d'avertissements (`F32ToBf16`, variable `r`
inutilisée). Traiter Eigen comme en-tête système les neutralise toutes d'un coup. L'image `ros:lyrical-ros-base` amd64
de base et les constructions arm64 n'activent jamais AVX : le flag y est inerte et
n'existe que pour garder les recettes identiques. Ce défaut n'était pas
reproductible localement, même en AMD64 émulé, pour cette raison.

## Construire et vérifier

Depuis la racine, sous l'utilisateur du projet :

```bash
docker build -f ros2/Dockerfile --target runtime -t mowgli-ros2:lyrical .
docker build -f sensors/gps/Dockerfile -t mowgli-gps:lyrical .
docker build -t mowgli-ldlidar:lyrical sensors/lidar-ldlidar
docker build -t mowgli-rplidar:lyrical sensors/lidar-rplidar
docker build -t mowgli-stl27l:lyrical sensors/lidar-stl27l
python3 -m pytest -q ros2/src/mowgli_bringup/test/test_nav2_params.py
```

Le `colcon test` complet reste le contrôle CI. Les nouveaux tests du plugin
vérifient les signatures, le refus d'une arrivée prématurée, la transformation
map/odom et la différence entre arrivée XY et arrivée XY+lacet. La simulation
Webots utilise toujours une image **Linux amd64**, même sur un hôte ARM.
Les trois services de `docker/docker-compose.simulation.yaml` fixent cette
architecture. Le devcontainer reste l'environnement de compilation ; lancer
la simulation depuis l'hôte, à la racine du dépôt :

```bash
docker compose -f docker/docker-compose.simulation.yaml up --build dev-sim
```

## Vérifications locales

Vérifications du 2026-09-14, dans des conteneurs Linux Lyrical / Nav2 1.5.1,
sur la baseline de travail indiquée ci-dessous. Les suites de tests s'exécutent
en ARM64 ; les constructions et contrôles AMD64 utilisent l'émulation Rosetta :

- Les 14 paquets de la pile et l'image d'exécution sont construits en ARM64 et AMD64 ;
  lancement `full_system --show-args`, outil de réglage moteur `--help`, schémas
  GNSS/u-blox et dépendances Nav2 supplémentaires vérifiés. Les 69 fichiers ELF
  contrôlés dans la pile et ses bibliothèques n'ont aucune dépendance introuvable.
- La suite `colcon test` complète passe sans erreur ni échec. Les 276 cas cppcheck
  sont ignorés par ament, qui désactive cppcheck 2.19.0 pour ses problèmes de
  performance ; ils ne sont pas comptés comme une analyse réussie.
  Une exécution séparée de cppcheck 2.13.0, avec les options du job CI,
  ne relève aucun problème dans les 30 fichiers C++ de production modifiés.
  Après correction de la durée de vie du
  contexte BT, huit démarrages/arrêts consécutifs passent ; le débogueur confirme
  la disparition des threads DDS avant la sortie du processus.
- Suite des plugins Nav2 : aucune erreur ni échec, également avec Cyclone DDS.
  Les trois nouveaux cas du goal checker passent.
- Serveur de couverture compilé contre Fields2Cover v3 sur Ubuntu 26.04 ;
  sa suite de tests passe sans erreur ni échec.
- Les trois tests du relais WebSocket de commande passent sous Python 3.14.
- 127 tests Python ciblés (configuration, lancement et dérive) passent, y compris
  sous Python 3.14 ; suites installateur ciblées : 35 + 17 assertions réussies.
- CMake lint, syntaxe des fichiers modifiés et clang-format 18 : conformes.
- Chargement de FTC, RotationShim, RPP, des goal checkers et du path handler
  dans `controller_server` : configuration réussie. Lecture et modification à
  chaud du plafond RPP imbriqué vérifiées, puis valeur initiale restaurée.
- Dans l'image d'exécution, les configurations fusionnées LiDAR et sans LiDAR
  passent les transitions configure/cleanup/shutdown des serveurs de contrôle,
  de planification Smac, de lissage et du moniteur de collision. Le navigateur BT,
  le serveur de manœuvres et le serveur de docking passent aussi ces transitions.
- Images LD19/LDLiDAR, RPLidar et STL27L ARM64 et AMD64 construites. Composant LD19
  chargé sous Cyclone DDS, laissé non configuré sans périphérique connecté ;
  bibliothèques des trois images résolues sur les deux architectures.
- Images GNSS ARM64 et AMD64 construites ; paquets, exécutables et imports des messages
  vérifiés. Les 39 ELF de chaque image passent le contrôle de liaison installé ;
  `gnss_inspect --help` démarre correctement. Suite du pont GNSS : 51 tests, aucun échec, 7 ignorés, y compris
  les contrôles de parité avec le pont Python.
- Devcontainer ARM64 et AMD64 construit ; utilisateur non privilégié membre de
  `dialout`, préfixes ROS/vendor, GTSAM/Fields2Cover et outils Python 3.14,
  clang-format 18, Go, Node et Yarn vérifiés.
- Pilote Webots et intégration `ros2_control` compilés sous Lyrical ARM64 et AMD64 ; imports
  Python et résolution des bibliothèques vérifiés. Le simulateur R2025a démarre
  sous Ubuntu 26.04 amd64 avec Xvfb (émulation sur l'hôte ARM) ; un contrôleur
  Python 3.14 exécute cinq pas de simulation puis termine normalement.
- L'image Webots finale démarre la pile complète, sans montage du code source,
  sur le domaine DDS 97 avec des cartes et une configuration temporaires.
  Les neuf serveurs Nav2 atteignent l'état actif ; le test reçoit plusieurs
  horodatages distincts pour la vérité terrain, l'odométrie brute et filtrée,
  le GNSS et le LiDAR. L'horloge avance et `map → base_footprint` est disponible.
  Les contrôleurs de roues et d'états articulaires sont actifs.
  Smac signale que le rayon d'inflation est inférieur à sa recommandation pour
  optimiser les tests de collision non circulaires ; ce réglage géométrique
  est conservé. Ce message n'empêche pas l'activation.

Ces tests n'impliquent aucun mouvement ni mesure du robot réel. Le contrôle
Webots couvre le démarrage et les flux, pas une tonte complète ni la performance
du suivi de trajectoire. La recette physique ci-dessous reste à réaliser.

Limite non résolue : lors de l'arrêt simultané par SIGINT de cette simulation
AMD64 sous Rosetta, Webots termine avec le code 139, son pilote avec le code 245,
et les serveurs controller/planner nécessitent SIGKILL après l'arrêt de l'horloge.
Le conteneur termine néanmoins avec le code 0 : ce code seul ne valide donc pas
un arrêt propre. L'origine exacte reste à diagnostiquer ; aucune attribution à
Rosetta ou au matériel réel n'est établie. Le démarrage validé ci-dessus ne vaut
pas validation de cet arrêt. Journal local : `/tmp/mowgli-full-sim-final.log`.

## Recette robot — HARDWARE_REQUIRED

Aucune mesure Kilted ne valide Lyrical. Cette migration est critique pour le
comportement physique : Nav2, les conversions géométriques et le middleware
changent. Aucun déploiement ni essai sur robot n'est effectué par cette migration.

Baseline de travail : dépôt `165c9fec68557818b4d5ca25d26d9fdf3390f79b` + le diff
de migration ; sous-module GNSS `ab32f673da6e9e6ffa8eac9a57b08656f8843645` ;
opennav_coverage `d6e41a298c5e9767cc2a42c9fafa7ca41b9bf089`.
L'unité robot, le hash du firmware réellement flashé et la révision du récepteur
ne sont pas disponibles dans cet environnement. Ils doivent être relevés avant
l'essai, avec le commit final, les digests de **toutes** les images, les versions
Nav2/Cyclone et le hash de la configuration. Ne pas qualifier un résultat sans
ces identifiants.

Prérequis de sécurité : sauvegarder la configuration et le volume de cartes,
retirer les lames pour la première recette, vérifier l'arrêt d'urgence matériel,
prévoir un opérateur à proximité et une zone dégagée avec marge extérieure.
Désactiver toute mise à jour automatique pendant la comparaison. La première
remise en tonte avec lames n'intervient qu'après réussite de la recette sans lames.

1. Démarrer simultanément toutes les images ROS/LiDAR/GNSS Lyrical sur un domaine
   DDS isolé. Critère : tous les nœuds requis actifs, `/gps/fix`, `/gps/status`,
   `/scan` si présent, et une seule chaîne TF `map → odom → base_footprint`.
2. Enregistrer une session avec `ros2/scripts/mow_session_monitor.py`. Tester
   sortie de station, transit et retour station. Critère : limites de vitesse
   configurées respectées, aucune oscillation entretenue ni blocage de fin de
   transit, arrivée en station confirmée par le matériel.
3. Sur une zone enregistrée connue, effectuer une couverture complète, puis
   pause/reprise en milieu de sous-chemin. Critère : aucun saut de curseur ni
   succès prématuré, sous-chemins réellement parcourus, aucun franchissement de
   bordure, progression finale cohérente avec le trajet enregistré.
4. Répéter avec et sans LiDAR. Avec LiDAR, présenter un obstacle inerte avec une
   marge suffisante : arrêt/contournement, reprise sans à-coup, limite de recul
   respectée. Tester une perte du flux LiDAR de manière contrôlée : pause et
   reprise conformes aux gardes existantes. Ne pas provoquer de creusement réel.
5. Vérifier arrêt d'urgence et annulation pendant mouvement, puis arrêt/redémarrage
   de la pile. Critère : roues arrêtées, lame désactivée, données de carte et de
   reprise conservées ; aucune commande résiduelle au redémarrage.

Retour arrière : arrêter toute la pile, remettre ensemble les digests Kilted
précédents, restaurer la sauvegarde des données si nécessaire, puis refaire les
contrôles statiques sans lames. Ne pas mélanger des images Kilted et Lyrical sur le
même domaine pour valider la migration.

## Sources

- [Guide officiel Nav2 Kilted → Lyrical](https://docs.nav2.org/rolling/configuration_and_development/migration_guides/kilted/Kilted/)
- [Interfaces Nav2 Lyrical](https://github.com/ros-navigation/navigation2/tree/lyrical/nav2_core)
- [Plateformes Lyrical](https://docs.ros.org/en/lyrical/Installation.html)
- Étude fournie par le mainteneur : `rapport-migration-lyrical-2026-09-14.md`,
  inventaire sur `dev@0432f427` ; ses hypothèses ne constituent pas une recette.
