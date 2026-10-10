# Récupération Universal GNSS — backend MAVROS
Date d’audit : 2026-09-30
Périmètre : inventaire et analyse en lecture seule. Aucun checkout, dépôt, build, robot ou Pixhawk modifié. Aucun test lancé.

## Résultat

Aucun changement non commité ou patch local du backend `gnss_mavros` n’a été retrouvé. L’implémentation MAVLink/GPS a été ajoutée dans Universal GNSS par le commit officiel `34afbf01e770dd0b4c3863f3822a122dc25e7b7b`, et MowgliMAVROS construit explicitement ce commit. Il est présent sur `origin/dev`, mais pas encore dans la référence locale `origin/main`.

Conformément à la demande, aucun patch vide ou patch reconstituant comme inédit un commit déjà présent dans UG n’est créé. Le fichier patch n’existe donc pas; son SHA-256 est sans objet.

## 1. Copies et artefacts trouvés

### Checkout UG autonome

Chemin : `/home/pepeuch/universal-gnss`

- Remote : `origin https://github.com/Pepeuch/universal-gnss.git` (fetch/push).
- HEAD : `df125ab95a347c2d631b01b55734908db25de013`.
- Branche : `dev`; description : `v0.6.0-165-gdf125ab`.
- `git status --short` : vide, aucune modification de travail.
- `gnss_mavros` : présent.
- Worktrees : uniquement ce checkout.
- Stashes : aucun.
- Historique récent :
  1. `df125ab` refactor(ros2): split public interfaces from runtime
  2. `ac06aae` feat(ros2): expose topic remapping in combined launch
  3. `437dddc` fix(docker): install CycloneDDS in runtime image
  4. `8118e0e` fix(docker): install CycloneDDS RMW runtime
  5. `ef31c4c` ddocs (vendor): update N4 documentation
  6. `34afbf0` feat(mavros): add Universal GNSS transport plugin
  7. `3f6cf33` docs(release): reconcile mower recovery and rate gates
  8. `626cc23` test(ros2): clarify runtime stamp versus position sequence
  9. `d4c55f8` fix(unicore): separate persistent apply from factory reset
  10. `41356bd` test(validation): record compact hardware evidence

La branche locale `dev` diverge de `origin/dev` : 1 commit local en avance et 3 commits de `origin/dev` absents localement. Le commit local `df125ab` concerne le découpage interfaces/runtime; `git log --all -- gnss_mavros` ne montre aucun changement `gnss_mavros` après `34afbf0`. Le reflog ne contient aucune opération des 29–30 septembre; les entrées disponibles vont du 11 au 13 septembre 2026.

### Submodule UG de MowgliNext

Chemin checkout : `/home/pepeuch/mowglinext/ros2/src/external/universal-gnss`
Gitdir : `/home/pepeuch/mowglinext/.git/modules/ros2/src/external/universal-gnss` (métadonnées de ce même checkout, pas une troisième copie).

- Remote : `origin https://github.com/Pepeuch/universal-gnss.git` (fetch/push).
- HEAD : `b459e3e8bcc1a92fe352b961102244905e79ad9a`, détaché.
- Description : `v0.1.3-rc1`.
- `git status --short` dans le checkout : vide.
- `gnss_mavros` : présent, hérité de `34afbf0`.
- Stashes : aucun; reflog utile des 29–30 septembre : aucune entrée trouvée.
- Historique récent :
  1. `b459e3e` test(ci): align ROS workflow contract with colcon environment
  2. `6ac6135` ci(ros2): avoid nounset during colcon environment sourcing
  3. `b815044` refactor(ros2): split public interfaces into universal_gnss_msgs
  4. `ac06aae` feat(ros2): expose topic remapping in combined launch
  5. `437dddc` fix(docker): install CycloneDDS in runtime image
  6. `8118e0e` fix(docker): install CycloneDDS RMW runtime
  7. `ef31c4c` ddocs (vendor): update N4 documentation
  8. `34afbf0` feat(mavros): add Universal GNSS transport plugin
  9. `3f6cf33` docs(release): reconcile mower recovery and rate gates
  10. `626cc23` test(ros2): clarify runtime stamp versus position sequence

Le tree MowgliNext de référence `origin/feat/mavros-refresh` épingle `56d96d538d90551d9ca42ae02c25b3b960fa620b`; le checkout submodule visible est à `b459e3e`. Le patch non suivi `/home/pepeuch/mowglinext/diff.patch` contient cette bascule de gitlink 56d→b459. C’est le rewind signalé dans la demande; il est exclu de toute récupération UG.

### Autres emplacements

- `/workspaces` : aucun checkout ou répertoire source Universal GNSS trouvé. Le workspace contient MowgliMAVROS, sans copie UG.
- `/home/pepeuch/mowglimavros-integration-20260929` : overlay source/build MowgliMAVROS, pas de checkout UG embarqué. Son Dockerfile clone le dépôt distant et épingle `UNIVERSAL_GNSS_COMMIT=34afbf01e770dd0b4c3863f3822a122dc25e7b7b`.
- `/home/pepeuch/mowglimavros-final-mm304` : checkout MowgliMAVROS distinct, lui aussi configuré pour cloner/pinner exactement `34afbf0`; le dépôt est dirty avec les changements d’intégration MowgliMAVROS, laissés intacts.
- `/home/pepeuch/mowglinext-gui-overlay-20260929` : artefacts GUI compilés, pas de source UG.
- `/home/pepeuch/mowglinext-integration-backup-20260929` : sauvegarde runtime, pas de source UG.
- `/home/pepeuch/mowglinext/docker/data/universal_gnss` et `docker/logs/universal_gnss` : répertoires runtime trouvés, aucun fichier source/log présent au moment de l’inventaire.
- `/home/pepeuch/gnss-recovery` : artifacts du split `universal_gnss_msgs`, pas des changements MAVROS. `universal-gnss-msgs-final.patch` (SHA-256 `622ac2690ecf7c0a251b3d28989117c94ba85b3677a8ecf2f355a878877757a4`) et `universal-gnss-msgs-fix1.patch` (`ad4a7af5df59ae859f2a6ea83a5168007765e7efb4abd279d2c02dccfad11ec0`) passent le contrôle inverse contre le checkout courant, donc leur contenu est déjà représenté. `universal-gnss-msgs-split.patch` (`4c407d4c99afd52269eafd4864100675fb1cd11868fa3f1d40f58cce1b0c797f`) est un draft plus ancien; il ne passe pas ce contrôle inverse complet. Aucun de ces patches ne touche `gnss_mavros/`. L’archive `universal-gnss-msgs-incremental.tar.gz` (`3e3cdda7671fc8acac3412212fa80a51799a4afca7fe34af6c1defab25e7bef1`) appartient au même chantier d’interfaces.

## 2. Baseline retenu et preuve

Baseline exact de l’ajout MAVROS : parent de `34afbf0`, soit `3f6cf3392a956d26b0462090cdce6a0f8f94d602`.

- `34afbf01e770dd0b4c3863f3822a122dc25e7b7b`, daté du 8 septembre 2026 22:44:13Z, a ce parent exact.
- Le commit crée le plugin MAVROS en 14 fichiers (1 200 insertions, 7 suppressions), dont huit nouveaux fichiers sous `gnss_mavros/`.
- `origin/dev` pointe actuellement à `b459e3e`, descendant du commit 34; celui-ci est publié dans le dépôt officiel sur `dev`.
- `origin/main` pointe à `f974b565100b4f2aa3d522cd9a292f30f905e8bc`. C’est l’ancêtre commun calculé de `origin/main` et `34afbf0`; le commit 34 n’est pas atteignable depuis `origin/main` aux refs inventoriées.
- Les deux Dockerfiles MowgliMAVROS et le lock `tools/external_backend_contract.lock.json` pinent explicitement le SHA complet de `34afbf0`. Le build n’est donc pas basé sur le checkout submodule courant à b459 ni sur une source cachée modifiée.
- `56d96d53` n’est pas présent dans l’object store du checkout UG autonome. Il s’agit du gitlink MowgliNext, pas du parent du commit backend.

Le seul commit qui modifie `gnss_mavros` dans l’historique inventorié est `34afbf0`. Aucune différence de travail, stash, reflog fin septembre, archive ou overlay contenant une autre implémentation MAVROS n’a été trouvée.

## 3. Déjà présent dans UG avant l’ajout MAVROS

Le parent `3f6cf33` contient déjà l’architecture de base Universal GNSS : `gnss_core`, agrégateur runtime, adaptateurs ROS, message `GnssStatus`, `NavSatFix`, launch récepteur/NTRIP et node NTRIP qui publie `universal_gnss_ros2/msg/RtcmFrame`. Le lancement `receiver_and_ntrip.launch.py` démarre toujours le `receiver_node` série/TCP puis `ntrip_node`; il n’a pas de sélection de backend.

Le contrat RTCM UG existant est une frame `stamp`, `message_type`, `data[]`. Le NTRIP UG consomme un `GnssStatus` pour GGA et publie les corrections sur son topic `rtcm`.

## 4. Ce que le commit 34 ajoute

Le plugin `universal_gnss_mavros` est un transport d’entrée MAVLink → état Universal GNSS :

- `GPS_RAW_INT` vers GPS1; `GPS2_RAW` vers GPS2.
- `GPS_RTK`/`GPS2_RTK` enrichissent indépendamment les récepteurs.
- `SYSTEM_TIME` et les transitions de connexion gèrent l’incarnation de source.
- Les topics privés sont `~/gps1/{fix,status}` et `~/gps2/{fix,status}`; le statut est `universal_gnss_ros2/msg/GnssStatus`, le fix `sensor_msgs/msg/NavSatFix`.
- Chaque observation GPS incrémente `position_observation_sequence`, même si ses valeurs sont répétées. Identité de source, incarnation, cache d’agrégat et invalidation de reconnexion sont déjà codés.
- CMake rend le plugin externe optionnel, l’export pluginlib est présent, et la version MAVROS admise est exactement 2.15.1.

Le plugin ne configure pas le récepteur; c’est bien un transport MAVLink, pas un backend série/vendor.

## 5. Ce qui se trouve seulement dans l’intégration MowgliMAVROS

Le Dockerfile MowgliMAVROS clone UG au SHA 34 puis compile `gnss_mavros`. Le lock de contrat vérifie le pin et le plugin. Le launch MowgliMAVROS choisit GPS1/GPS2 et possède un booléen `MAVROS_GPS1_CANONICAL`.

Deux chemins temporaires existent :

- Quand le booléen est faux, le launch remappe les sorties du plugin UG vers `/gps/fix` et `/gps/status`.
- Quand il est vrai, les sorties UG restent privées et `mavros_hardware_bridge` lit directement `/mavros/gpsstatus/gps1/raw` puis publie lui-même les messages Mowgli sur les topics canoniques.

Le second chemin contourne le traitement GNSS UG. Le premier publie un `universal_gnss_ros2/msg/GnssStatus` sur `/gps/status`, alors que le lock MowgliNext attend `mowgli_interfaces/msg/GnssStatus`. Ce choix temporaire ne suffit donc pas comme contrat final.

L’intégration MowgliMAVROS possède aussi son propre `mowgli_ntrip_client`: il produit `mavros_msgs/msg/RTCM` pour `/mavros/gps_rtk/send_rtcm`. Ce chemin bypass le `ntrip_node` et `RtcmFrame` d’UG.

## 6. État par catégorie A–E

- **A — Modifications non commitées UG :** aucune dans les deux worktrees.
- **B — Commit UG officiel :** `34afbf0`, publié sur `origin/dev`, parent `3f6cf33`; non contenu dans `origin/main` au snapshot.
- **C — Copie/overlay seulement :** aucun code `gnss_mavros` caché. Le checkout autonome a un commit local divergent `df125ab` sur le split interfaces/runtime, sans delta `gnss_mavros`. Les artifacts gnss-recovery sont déjà appliqués ou sont des drafts du split d’interfaces.
- **D — Code dans MowgliMAVROS destiné à UG :** aucun code plugin à déplacer. Le pin/build du plugin est dans MowgliMAVROS; la projection GPS1 directe et le client NTRIP Mowgli sont des adaptations propres à MowgliMAVROS. Le NTRIP Mowgli devra cesser d’être le propriétaire lorsque le chemin UG sera complet.
- **E — Documentation seulement :** les README UG documentent que l’injection RTCM n’est volontairement pas implémentée. Le chemin GPS plugin, lui, est du code compilé et testé au niveau adaptateur.

## 7. Ce qui manque pour GNSS_STACK=universal → backend=mavros → MAVROS/Pixhawk → GPS+RTCM

1. Une sélection explicite `backend=serial|mavros` dans l’orchestration Universal GNSS. En mode MAVROS, `receiver_node` série ne doit pas être démarré; le NTRIP UG reste disponible.
2. Choix explicite d’un récepteur GPS1/GPS2 comme source active pour GGA/canonical fix/status, sans deux publishers.
3. Un chemin de statut/fix unique entre les sorties UG du plugin et les interfaces attendues par les consommateurs MowgliNext. Le type `GnssStatus` diffère aujourd’hui entre UG et MowgliNext; il faut fixer l’adaptation/ownership avant de raccorder directement les topics.
4. Une entrée RTCM dans le plugin/sink MAVROS UG : recevoir le `RtcmFrame` émis par NTRIP UG, convertir/fragmenter en messages MAVLink GPS_RTCM_DATA avec les limites MAVLink, appeler le transport MAVROS, et suivre l’acceptation/activité sans altérer les octets.
5. Une configuration runtime qui conserve `GNSS_STACK=universal`, sélectionne son transport MAVROS, et éteint tout producteur parallèle de `/gps/fix`, `/gps/status` et NTRIP.
6. Validation pluginlib sous MAVROS 2.15.1, compatibilité de type/QoS/remaps, reconnect/incarnation, choix GPS1/GPS2, NTRIP→RTCM byte-perfect et absence de second owner.

### Ownership cible

- Universal GNSS reste propriétaire de l’agrégation GNSS, du statut/provenance/fraîcheur, de la sélection de source et du NTRIP.
- Le backend MAVROS est un transport : données GPS MAVLink entrantes et corrections RTCM sortantes.
- Un seul chemin publie `/gps/fix` et `/gps/status`. La projection vers le type ROS MowgliNext existant doit préserver son contrat; le plugin UG ne doit pas fabriquer un second topic canonique concurrent.
- NTRIP UG consomme le statut du récepteur sélectionné et publie `RtcmFrame`; le sink Universal GNSS MAVROS injecte ensuite dans le FCU. Le client NTRIP Mowgli séparé ne doit pas rester un second owner nominal.

## 8. Fichiers UG candidats pour compléter le travail manquant

Aucun de ces fichiers n’est inclus dans un patch de récupération; cette liste est une portée de travail futur déduite des trous :

1. `gnss_mavros/src/universal_gnss_plugin.cpp` — subscription RTCM et envoi MAVLink GPS_RTCM_DATA, plus sortie/source sélectionnée si ce choix est placé dans le plugin.
2. Nouveau `gnss_mavros/include/universal_gnss_mavros/rtcm_mavlink_sink.hpp` et `gnss_mavros/src/rtcm_mavlink_sink.cpp` (ou équivalent testé) — segmentation, flags/sequence et transmission RTCM.
3. `gnss_mavros/CMakeLists.txt`, `gnss_mavros/package.xml` — dépendances/tests éventuels pour le sink.
4. Nouveau test `gnss_mavros/tests/test_rtcm_mavlink_sink.cpp`, et tests de chargement/remapping plugin; étendre `test_mavlink_gnss_adapter.cpp` uniquement si le modèle GPS change.
5. `gnss_ros2/launch/receiver_and_ntrip.launch.py` (ou nouveau `gnss_ros2/launch/backend.launch.py`) — sélection exclusive du transport et statut sélectionné fourni au NTRIP.
6. `gnss_ros2/tests/test_combined_launch.py` — aucune instance série en backend MAVROS, propriétaire unique et remaps cohérents.
7. `gnss_mavros/README.md` et `docs/ros2.md` — contrat de transport, selection/fallback, RTCM et ownership.

Le besoin exact de traduire le type UG `GnssStatus` vers le type MowgliNext doit être résolu dans le design de l’adaptateur avant implementation; ne pas ajouter une dépendance UG envers MowgliNext sans décision d’architecture.

## 9. Tests existants et manquants

La cible CMake existante `gnss_mavros_test_adapter` exécute les scénarios `TestGps1Observation`, `TestGps2ObservationAndAvailability`, observations identiques avec séquence incrémentée, effacement de métadonnées absentes, reconnexion/incarnation, régression et wrap de boot time, indépendance GPS1/GPS2, RTK sans nouvelle position et invalidation sur preuve tardive de reboot.

Aucun test de plugin MAVROS chargé dans un processus réel/factice, sélection backend/canonical owner, entrée RTCM→GPS_RTCM_DATA, fragmentation/limites MAVLink, conservation octet pour octet, GGA venant de la source sélectionnée ou transition NTRIP/reconnexion end-to-end n’est présent dans le plugin actuel. Aucun test n’a été relancé pendant cet audit.

## 10. Patch

Patch de récupération non créé : aucun changement backend MAVROS non publié n’a été trouvé; le seul commit transport `34afbf0` est déjà dans l’historique officiel `origin/dev`. Aucun fichier `/home/pepeuch/UNIVERSAL_GNSS_MAVROS_RECOVERY_20260930.patch` n’est produit. SHA-256 : sans objet.

Sources de preuve principales : `git show 34afbf0`, historique/log/reflogs des deux worktrees UG, `ros2/Dockerfile`, `tools/external_backend_contract.lock.json`, `mavros_backend.launch.py`, `gnss_mavros/README.md`, `gnss_mavros/src/universal_gnss_plugin.cpp`, `gnss_ros2/launch/receiver_and_ntrip.launch.py`, artifacts `/home/pepeuch/gnss-recovery` et `/home/pepeuch/mowglinext/diff.patch`.
