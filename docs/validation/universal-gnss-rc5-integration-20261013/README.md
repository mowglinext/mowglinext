# Synchronisation Universal GNSS rc5 / MowgliMAVROS et site

Baseline MowgliNext : `integration/pr960-dev-validated`, HEAD
`52d1e3e611c6afd0574522c605ce57fc835fdedd`, dans
`/tmp/mowglinext-mavros-merge-test`. Les deux gitlinks étaient déjà déplacés
au démarrage ; aucun fichier interne des sous-modules n'a été modifié.
Le checkout `/ros2_ws/src/mowglinext` n'a pas été modifié.

## Identités vérifiées

- Universal GNSS : `383caba3de94e16167764393d5a4ef046078b015` →
  `9a329cc216ed137d0e375a4cfe1f5c26e1dd52d0` (`v0.7.4-rc5`).
- MowgliMAVROS : `811eda29c1fe7097907d6687ec560fb56100d802` →
  `5328dd70d1fce34ee9e0c81ff3883b0260a712de`.
- opennav_coverage : `d6e41a298c5e9767cc2a42c9fafa7ca41b9bf089`, inchangé.
- Les trois checkouts sont propres et correspondent aux pins proposés.

Image : `ghcr.io/pepeuch/universal-gnss-ros2-lyrical:v0.7.4-rc5@sha256:127103ef9e90c3fa47edb38c01797128f8c1e616ab4548a35fe135ff16349727`.

La requête buildx avec les identifiants Docker existants a reçu HTTP 403.
La lecture anonyme du registre a réussi : SHA256 recalculé sur les octets du
manifeste égal à Docker-Content-Digest, puis hashes des manifests enfants et
configurations vérifiés. Les labels des deux architectures indiquent
`org.opencontainers.image.version=v0.7.4-rc5` et le SHA Universal GNSS cible.
Le tag Git annoté distant pointe également sur ce SHA. Le lien de release
et le dépôt officiel MAVROS répondent HTTP 200.

## Portée et fichiers

Les références actives, le badge et les attentes des tests sont synchronisés.
La séparation UNIVERSAL_GNSS_IMAGE / IMAGE_TAG, les modes de sélection GNSS,
les digests obligatoires et le schéma de configuration `1` sont conservés.

- `.github/submodule-pins.txt`
- `README.md`
- `docker/.env.example`
- `docs/claude/codemaps/ros2_workspace_tooling.md`
- `docs/index.html`
- `install/compose/docker-compose.gps-mavros.yml`
- `install/compose/docker-compose.gps.yml`
- `install/deployment.json`
- `install/lib/config.sh`
- `install/tests/test_env_output.sh`
- `install/tests/test_gnss_external_sidecar_contract.sh`
- `ros2/src/external/mowglimavros`
- `ros2/src/external/universal-gnss`
- `docs/validation/universal-gnss-rc5-integration-20261013/README.md`

Sur le site, MAVROS est sélectionnable, sans Soon, avec deux liens vers
https://github.com/Pepeuch/mowglimavros et une description ARIA de son statut.
Il est présenté pour évaluation avec validation ARM64 en attente. Les styles
et points de rupture existants ne changent pas. OpenMower reste Soon/désactivé.

## Interfaces et validations

- 28 scripts d'installation PASS : 782 assertions comptées dans 26 scripts ;
  les deux scripts publication/updater passent sans compteur d'assertions.
- `docs/test_web_composer.sh` : 36/36 ; `docs/test_install.sh` : 19/19.
- Contrats bringup GNSS/sidecar/backend et config-drift : 43/43 pytest.
  Le premier essai échouait uniquement sur le répertoire ROS de logs en lecture
  seule ; le rerun avec ROS_LOG_DIR dans le scratch est entièrement vert.
- Docker Compose réel : configurations GNSS direct et MAVROS valides ;
  image/digest, schéma 1 et syntaxe des deux launchers Python vérifiés.
- Build ROS2 Lyrical isolé : mowgli_interfaces, universal_gnss_msgs et
  mowgli_gnss_bridge compilés depuis ce checkout, sans l'overlay MowgliNext
  préexistant. Bridge : 11 tests C++ + 15 tests de parité Python PASS.
  Résumé colcon : 61 entrées, 0 erreur/échec, 7 skips cppcheck (version 2.19.0 ignorée par ament pour ses problèmes de performance connus).
  Le paquet messages a été testé séparément : 2/2 PASS.
- Le Dockerfile copie et compile le paquet interface-only universal_gnss_msgs
  tel quel. L'ancien `ros2/universal_gnss_interfaces.CMakeLists.txt` n'est
  référencé par aucun workflow/script/Dockerfile actif ; ses trois interfaces
  correspondent à celles du paquet courant. Aucun adaptateur n'a été modifié.
- HTML : parseur lxml sans diagnostic avant/après, IDs uniques, liens et état
  Soon/disabled/ARIA vérifiés ; syntaxe JS Node valide. Exécution du JS avec un
  DOM minimal : sélection MAVROS, commande sans flags GNSS directs, aria-pressed,
  désactivation GNSS puis restauration en revenant à Mowgli PASS.
- Pins : 3/3 par comparaison checkouts/fichier. Le script CI inchangé passe
  également sur un arbre Git proposé construit dans un index et un répertoire
  d'objets temporaires. Son exécution directe sur HEAD signale normalement les
  deux anciens gitlinks : aucun commit n'a été créé. L'index principal est intact.
- Syntaxe Bash et `git diff --check` PASS. Aucun C++ n'est modifié.

Les logs et builds sont ignorés sous `build/ug-rc5-sync/` : ghcr-proof.json,
logs par script, python-contracts.log, ros-build.log, ros-test.log, msgs-test.log,
pins-head.log, pins-prospective-tree.log, compose-{direct,mavros}.yml et
site-behavior.log. Ils constituent du scratch local, pas une archive versionnée.

## Références historiques conservées

Aucune modification des rapports d'odométrie MAVROS du 2026-10-12
(`mavros-odometry-{window,finalization}-20261012`) ni du rapport
`pr960-merge-dev-20261012` : leurs anciens SHA décrivent leurs baselines exactes.
Le commentaire de l'incident v0.7.1-rc3 dans submodule-pins reste également intact.

## Limites et gates

CI MAVROS au dernier contrôle : AMD64 success, ARM64 in_progress.
[Workflow officiel](https://github.com/Pepeuch/mowglimavros/actions/runs/38440690958).
La publication finale n'est pas déclarée prête avant réussite ARM64. Si son
statut change, réviser aussi la mention d'attente du site avant publication.

OpenMower : HARDWARE_REQUIRED. `sensors/openmower/README.md`, section
Not covered (yet), indique explicitement sources et simulation uniquement.
Baseline logicielle : HEAD MowgliNext ci-dessus, code OpenMower inchangé.
Unité robot, révisions LowLevel/xESC et montage IMU n'ont pas été qualifiés ici ;
ils doivent être enregistrés avant toute mesure. Procédure : banc sécurisé avec
roues hors sol, `mowing_enabled: false`, lame désarmée, arrêt physique accessible ;
confirmer handshake et feedback des ESC, odométrie cohérente, réaction à STOP/lift,
perte de télémétrie et arrêt du processus. Critère : aucune commande de lame,
roues arrêtées sur chaque défaut, état firmware_compatible conforme aux liens
réels et watchdogs observés dans les bornes documentées pour les révisions
identifiées. Cette campagne n'a pas été exécutée ; Soon reste justifié.

Pas de build Docker complet MowgliNext ni de contrôle visuel navigateur/mobile
ou validateur HTML W3C ; les styles sont conservés et les contrôles disponibles
ont été exécutés. Aucun déploiement robot ou GitHub Pages. Aucun commit, push,
merge, tag ou changement de branche. Ensemble prêt pour revue avant commit,
sous réserve du gate ARM64 avant publication finale.
