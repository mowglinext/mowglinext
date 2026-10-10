#!/usr/bin/env bash
MSG_UPDATER_UNSUPPORTED="Les mises à jour automatiques nécessitent Linux amd64/arm64, systemd et Docker Compose. Les versions restent consultables."
MSG_UPDATER_RECOVERY="Une mise à jour est en maintenance/récupération. Terminez-la avant de relancer l'installation."
MSG_UPDATER_SOURCE="Dépôt source du programme de mise à jour non pris en charge."
MSG_UPDATER_UNPUBLISHED="Le programme de mise à jour de cette révision est indisponible. Attendez la fin du workflow Host updater, ou fournissez MOWGLI_UPDATER_BINARY compilé depuis cette révision. Installation arrêtée ; Watchtower n'a pas été activé en remplacement."
MSG_UPDATER_CHECKSUM="Échec de vérification du programme de mise à jour."
MSG_UPDATER_INSTALLED="Programme de mise à jour installé. Consultez sa version dans Paramètres > Mises à jour."
# French locale

# ── Common ──
MSG_YES_NO="O/n"
MSG_YOUR_CHOICE="Ton choix"
MSG_CHOICE="Choix"

# ── Menu de mode (lancement sans argument sur un robot installe) ──
MSG_MODE_TITLE="Ce robot est deja installe. Que voulez-vous faire ?"
MSG_MODE_UPDATE="Mettre a jour — synchroniser le depot, telecharger les images, redemarrer (sans le service de mise a jour)"
MSG_MODE_REPAIR="Reparer — reappliquer regles udev, UART, .env, compose et helpers depuis les choix enregistres"
MSG_MODE_REINSTALL="Reinstaller / reconfigurer — repasser par les questions materiel"
MSG_MODE_CHECK="Verifier — diagnostic uniquement"
MSG_MODE_INVALID="Choix invalide, mise a jour lancee"
MSG_MODE_SELECTED="Mode :"
MSG_MODE_UNINSTALL="Desinstaller — tout supprimer sauf les cartes et mowgli_robot.yaml"

# ── Desinstallation (uninstall.sh) ──
MSG_UNINSTALL_TITLE="Desinstaller MowgliNext"
MSG_UNINSTALL_REMOVES="Cela supprime :"
MSG_UNINSTALL_CONTAINERS="tous les conteneurs mowgli-* et leurs images (volume des cartes intact)"
MSG_UNINSTALL_UPDATER="le service de mise a jour, son binaire, sa configuration et son etat"
MSG_UNINSTALL_HOST="regles udev, sysctl DDS, MOTD, commandes mowgli-*, notre rc.local"
MSG_UNINSTALL_KEEPS="Cela conserve :"
MSG_UNINSTALL_MAPS="volume Docker : areas.dat et le graphe de localisation sauvegarde"
MSG_UNINSTALL_NOT_OURS="Docker lui-meme et les overlays UART de /boot (pas a nous)"
MSG_UNINSTALL_CONFIRM="Supprimer MowgliNext de cet hote ?"
MSG_UNINSTALL_NEEDS_YES="Pas de terminal pour confirmer ; passez --yes pour desinstaller sans interaction."
MSG_UNINSTALL_ABORTED="Desinstallation annulee ; rien n'a ete modifie."
MSG_UNINSTALL_COMPOSE_FAILED="docker compose down a echoue ; suppression des conteneurs un par un."
MSG_UNINSTALL_NOT_A_CHECKOUT="Refus de supprimer un repertoire qui n'est pas un depot MowgliNext :"
MSG_UNINSTALL_REMOVED="Supprime"
MSG_UNINSTALL_RESTORED="Restaure"
MSG_UNINSTALL_KEPT="Conserve"
MSG_UNINSTALL_DONE="MowgliNext supprime. Reinstallez avec la commande de https://mowgli.garden : mowgli_robot.yaml et le volume des cartes seront retrouves."
MSG_UNINSTALL_MAPS_HINT="Pour supprimer aussi les cartes : docker volume rm"
MSG_MODE_NO_TTY="deja installe, pas de terminal pour demander ; passez install|repair|check pour choisir"

# ── System update (system.sh) ──
MSG_SYSTEM_UPDATE="Voulez-vous mettre a jour le systeme ?"
MSG_SYSTEM_UPDATE_SKIPPED="Mise a jour systeme ignoree"
MSG_APT_PIN_CONFIRM="Voulez-vous epingler APT sur la version actuelle"
MSG_APT_PINNED="APT epingle sur"
MSG_APT_NO_PIN="Aucun epinglage APT applique"
MSG_APT_UPGRADE_CONFIRM="Voulez-vous lancer apt upgrade -y maintenant ?"
MSG_APT_UPGRADING="Lancement de apt upgrade..."
MSG_APT_UPGRADED="Systeme mis a jour"
MSG_APT_UPGRADE_SKIPPED="APT upgrade ignore"

# ── UART detection ──
MSG_UART_DETECTING="Detection des ports UART disponibles..."
MSG_UART_AVAILABLE="Ports UART disponibles :"
MSG_UART_NONE_FOUND="Aucun port UART detecte. Entrez le chemin manuellement."
MSG_UART_SELECT="Selectionner le port UART"
MSG_UART_MANUAL="Entrer manuellement"
MSG_UART_MANUAL_PROMPT="Chemin du peripherique UART ?"
MSG_UART_INVALID="Choix invalide"
MSG_UART_PORT_TAKEN="%s ne peut pas utiliser %s : le controleur %s y est cable. Choisissez un autre port (ou l'USB)."
MSG_UART_AFTER_REBOOT="disponible apres redemarrage"

# ── GPS (gps.sh) ──
MSG_GNSS_CONNECTION="Connexion GNSS :"
MSG_MAVROS_GNSS_SOURCE_TITLE="Où est raccordé le récepteur GNSS ?"
MSG_MAVROS_GNSS_SOURCE_PIXHAWK="GPS raccordé au Pixhawk (MAVROS possède le récepteur)"
MSG_MAVROS_GNSS_SOURCE_SOC="GPS raccordé directement au calculateur compagnon / SoC"
MSG_MAVROS_GNSS_SOURCE_EXISTING="Source GNSS prédéfinie/conservée :"
MSG_MAVROS_GNSS_SOURCE_SELECTED="Source GNSS sélectionnée :"
MSG_MAVROS_GNSS_SOURCE_INVALID="Choix de source GNSS invalide"
MSG_GPS_DEBUG_CONFIRM="Activer le port GPS debug (miniUART / gps_debug) ?"
MSG_GPS_INVALID_CONNECTION="Choix connexion GPS invalide"
MSG_GPS_INVALID_PROTOCOL="Choix protocole invalide"
MSG_GPS_MAIN="GPS principal"

# ── LiDAR (lidar.sh) ──
MSG_LIDAR_TYPE="Type de LiDAR :"
MSG_LIDAR_NONE="Aucun"
MSG_LIDAR_CONNECTION="Connexion LiDAR :"
MSG_LIDAR_INVALID_TYPE="Choix LiDAR invalide"
MSG_LIDAR_INVALID_CONNECTION="Choix connexion LiDAR invalide"


# ── Tools (tools.sh) ──
MSG_TOOLS_HELPERS="Outils optionnels : helpers Mowgli"

# ── MOTD (motd.sh) ──
MSG_MOTD_NOT_CONNECTED="non connecte"
MSG_MOTD_FREE="libres"
MSG_MOTD_PACKAGES="paquet(s)"
MSG_MOTD_LOCAL_IP="IP locale"
MSG_MOTD_NOT_SET="non defini"
MSG_MOTD_RUNNING="actif(s)"

MSG_UPDATER_STACK_BACKEND="Les mises à jour gérées prennent en charge le matériel Mowgli et OpenMower v1."
MSG_UPDATE_MANUAL_UPDATER="Le service de mise a jour est installe. Cette mise a jour manuelle regenere docker-compose.yaml depuis ce depot et laisse les images suivre les tags de .env au lieu de la version epinglee par le service ; Reglages > Mises a jour signalera l'installation comme divergente jusqu'a la prochaine version geree, qui adoptera le resultat."
MSG_UPDATE_MANUAL_PINS="Epingles d'images du service (docker/update-images.json) mises de cote en copie datee ; les images suivent maintenant docker/.env."
MSG_UPDATER_DIRECTORY_MISMATCH="Le service de mise a jour est configure pour un autre repertoire que cette execution. Lancez l'installateur depuis le chemin utilise a son installation (MOWGLI_HOME=<ce chemin>), ou reenregistrez-le avec --only=updater."
MSG_UPDATER_HARDWARE_LEGACY="Ces choix matériels nécessitent le parcours d'installation existant (MAVROS). Les conteneurs sélectionnés sont conservés ; les mises à jour coordonnées ne sont pas activées."
MSG_UPDATER_HARDWARE_MANAGED="Cette installation utilise déjà les mises à jour gérées. Le choix MAVROS nécessite une migration explicite ; les fichiers d'exécution n'ont pas été régénérés."
MSG_UPDATER_STACK_REVIEW="Choix matériels enregistrés. Consultez les mises à jour logicielles pour appliquer les changements de conteneurs ; la définition installée a été conservée."

# Compose baseline / legacy adoption (install/lib/compose.sh)
MSG_COMPOSE_BASELINE_UNAVAILABLE="Impossible d'enregistrer l'empreinte du fichier Compose généré (sha256sum/shasum absents, ou docker/stack-definition.sha256 non inscriptible) ; aucune référence enregistrée."
MSG_COMPOSE_LEGACY_EXPLAIN="Le service de mise a jour ne peut pas garantir docker/docker-compose.yaml : la raison est affichee ci-dessus (soit il precede l'empreinte enregistree et differe de la definition actuelle, soit il a ete modifie a la main apres sa generation). Si la modification n'est pas la votre, c'est seulement la version qui a evolue et il est sur de le regenerer."
MSG_COMPOSE_LEGACY_BACKUP="Le fichier actuel est conserve a cote du nouveau sous docker/docker-compose.yaml.legacy-<date> ou .edited-<date>. Les modifications a garder vont dans docker/stack-overrides.yaml."
MSG_COMPOSE_LEGACY_CONFIRM="Sauvegarder le docker/docker-compose.yaml actuel et le regenerer ?"
MSG_COMPOSE_MISSING="docker/docker-compose.yaml est absent ; il sera recree depuis la definition installee (plus docker/stack-overrides.yaml s'il existe)."
MSG_COMPOSE_LEGACY_DECLINED="docker/docker-compose.yaml laisse intact. Deplacez vos modifications dans docker/stack-overrides.yaml puis relancez (non interactif : MOWGLI_ADOPT_LEGACY_COMPOSE=true)."

# Repository self-update (install/lib/deploy.sh)
MSG_REPO_LOCAL_CHANGES="Des fichiers suivis de ce dépôt ont été modifiés localement :"
MSG_REPO_LOCAL_CHANGES_SAFE="La configuration du robot (docker/.env, docker/config/, docker/stack-overrides.yaml) n'est pas suivie par git et n'est jamais touchée ici."
MSG_REPO_LOCAL_CHANGES_CHOICE="(s) les mettre de côté dans une sauvegarde nommée et continuer, (k) les garder sans toucher au dépôt, (a) abandonner"
MSG_REPO_LOCAL_CHANGES_KEPT="Modifications locales conservées ; le dépôt n'a pas été modifié."
MSG_REPO_STASHED="Modifications locales sauvegardées dans le stash git :"
MSG_REPO_STASH_RESTORE="Pour les restaurer plus tard :"
MSG_REPO_STASH_FAILED="git stash a échoué ; le dépôt n'a pas été modifié."
MSG_REPO_UPDATE_ABORTED="Installation abandonnée ; rien n'a été modifié."
MSG_REPO_UPDATE_CONFIRM="nouveau(x) commit(s) disponible(s). Mettre à jour ce dépôt avant de continuer ?"
MSG_REPO_UPDATED="Dépôt avancé jusqu'à"
MSG_REPO_NOT_FAST_FORWARD="Ce dépôt contient ses propres commits et ne peut pas être avancé ; poursuite sans mise à jour. Distant :"
MSG_REPO_FETCH_FAILED="Dépôt distant injoignable ; poursuite avec le dépôt actuel. Distant :"
MSG_REPO_FOREIGN_OWNER="Une partie du dépôt appartient à un autre utilisateur (souvent après un 'sudo git ...'), git ne peut donc pas le mettre à jour :"
MSG_REPO_FOREIGN_OWNER_FIX="Poursuite avec le dépôt actuel. Pour corriger :"
MSG_REPO_SUBMODULE_SKIPPED="Impossible de mettre à jour les sous-modules git. Ils ne servent qu'à COMPILER les sources ROS2 ; un robot qui utilise les images publiées n'en a pas besoin."
MSG_COMPOSE_MISSING_CONFIRM="Recreer docker/docker-compose.yaml ?"
MSG_COMPOSE_MISSING_DECLINED="docker/docker-compose.yaml non recree ; la pile ne peut pas demarrer sans lui."
