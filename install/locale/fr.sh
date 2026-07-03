#!/usr/bin/env bash
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
