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
MSG_UART_AFTER_REBOOT="disponible apres redemarrage"

# ── GPS (gps.sh) ──
MSG_GNSS_CONNECTION="Connexion GNSS :"
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

# ── Rangefinders (range.sh) ──
MSG_TFLUNA_CONFIG="Configuration des capteurs TF-Luna :"
MSG_TFLUNA_NONE="Aucun"
MSG_TFLUNA_FRONT_ONLY="Front uniquement"
MSG_TFLUNA_EDGE_ONLY="Edge uniquement"
MSG_TFLUNA_FRONT_EDGE="Front + edge"
MSG_TFLUNA_INVALID="Choix TF-Luna invalide"

# ── Tools (tools.sh) ──
MSG_TOOLS_DOCKER_CLI="Outils optionnels : gestionnaire Docker en ligne de commande"
MSG_TOOLS_DOCKER_LAZY="Oui, installer lazydocker (recommande)"
MSG_TOOLS_DOCKER_CTOP="Oui, installer ctop (alternatif)"
MSG_TOOLS_NO="Non"
MSG_TOOLS_FILE_MANAGER="Outils optionnels : gestionnaire de fichiers"
MSG_TOOLS_FILE_MC="Oui, installer Midnight Commander (mc)"
MSG_TOOLS_FILE_RANGER="Oui, installer ranger"
MSG_TOOLS_DEBUG="Outils optionnels : developpement et debug"
MSG_TOOLS_DEBUG_ALL="Tous les outils (recommande)"
MSG_TOOLS_DEBUG_ESSENTIAL="Outils essentiels seulement"
MSG_TOOLS_DEBUG_NONE="Aucun"
MSG_TOOLS_HELPERS="Outils optionnels : helpers Mowgli"
MSG_TOOLS_HELPERS_CONFIRM="Installer les commandes helper Mowgli ?"

# ── MOTD (motd.sh) ──
MSG_MOTD_NOT_CONNECTED="non connecte"
MSG_MOTD_FREE="libres"
MSG_MOTD_PACKAGES="paquet(s)"
MSG_MOTD_LOCAL_IP="IP locale"
MSG_MOTD_NOT_SET="non defini"
MSG_MOTD_RUNNING="actif(s)"

MSG_UPDATER_STACK_BACKEND="Les mises à jour gérées prennent en charge le matériel Mowgli."
MSG_UPDATER_HARDWARE_LEGACY="Ces choix matériels nécessitent le parcours d'installation existant (MAVROS, TF-Luna ou VESC). Les conteneurs sélectionnés sont conservés ; les mises à jour coordonnées ne sont pas activées."
MSG_UPDATER_HARDWARE_MANAGED="Cette installation utilise déjà les mises à jour gérées. Les choix MAVROS, TF-Luna et VESC nécessitent une migration explicite ; les fichiers d'exécution n'ont pas été régénérés."
MSG_UPDATER_STACK_REVIEW="Choix matériels enregistrés. Consultez les mises à jour logicielles pour appliquer les changements de conteneurs ; la définition installée a été conservée."

# Compose baseline / legacy adoption (install/lib/compose.sh)
MSG_COMPOSE_BASELINE_UNAVAILABLE="Impossible d'enregistrer l'empreinte du fichier Compose généré (sha256sum/shasum absents, ou docker/stack-definition.sha256 non inscriptible) ; aucune référence enregistrée."
MSG_COMPOSE_LEGACY_EXPLAIN="docker/docker-compose.yaml a été généré avant les mises à jour gérées : aucune empreinte n'en a été enregistrée. Il diffère de la définition actuelle sur les réglages listés ci-dessus. Si vous n'avez jamais modifié ce fichier à la main, il s'agit seulement de l'évolution de la version et il peut être remplacé sans risque."
MSG_COMPOSE_LEGACY_BACKUP="Le fichier actuel est conservé sous docker/docker-compose.yaml.legacy-<date>. Les modifications manuelles à garder vont dans docker/stack-overrides.yaml."
MSG_COMPOSE_LEGACY_CONFIRM="Remplacer docker/docker-compose.yaml par la définition actuelle ?"
MSG_COMPOSE_LEGACY_DECLINED="docker/docker-compose.yaml laissé intact. Déplacez vos modifications dans docker/stack-overrides.yaml puis relancez l'installateur (non interactif : MOWGLI_ADOPT_LEGACY_COMPOSE=true)."

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
