MOWGLI_MANUAL.chapters.push({
  id: "install",
  icon: "⬇️",
  title: { en: "Install the software", fr: "Installer le logiciel" },
  steps: [
    {
      id: "install-composer",
      title: { en: "Compose your install command", fr: "Composer votre commande d'installation" },
      body: {
        en: `Go to the **install composer** on [mowgli.garden → Get Started](https://mowgli.garden/#getting-started) and click through your hardware:

| Group | Pick |
|---|---|
| Hardware backend | **Mowgli** (STM32 mainboard) |
| Universal GNSS — receiver family | **Auto**, or u-blox / Unicore explicitly |
| Universal GNSS — connection | **UART** or **USB**, as wired in chapter 6 |
| LiDAR | **LDLiDAR / UART**, **LDLiDAR / USB**, RPLidar…, or **None** |
| Release channel | **main** (stable) |

The composer prints a one-line command such as:

\`\`\`
curl -sSL https://mowgli.garden/install.sh | bash -s -- \\
  --backend=mowgli --gnss=auto --gnss-connection=uart --lidar=ldlidar-uart
\`\`\`

Copy it. The installer will still ask for the exact serial device, detect the receiver baud, and ask the mower-specific questions (datum, NTRIP, dock) interactively.`,
        fr: `Allez sur le **composeur d'installation** de [mowgli.garden → Get Started](https://mowgli.garden/#getting-started) et cliquez votre matériel :

| Groupe | Choix |
|---|---|
| Hardware backend | **Mowgli** (carte mère STM32) |
| Universal GNSS — famille de récepteur | **Auto**, ou u-blox / Unicore explicitement |
| Universal GNSS — connexion | **UART** ou **USB**, selon le câblage du chapitre 6 |
| LiDAR | **LDLiDAR / UART**, **LDLiDAR / USB**, RPLidar…, ou **None** |
| Canal de release | **main** (stable) |

Le composeur affiche une commande d'une ligne du type :

\`\`\`
curl -sSL https://mowgli.garden/install.sh | bash -s -- \\
  --backend=mowgli --gnss=auto --gnss-connection=uart --lidar=ldlidar-uart
\`\`\`

Copiez-la. L'installateur demandera quand même le périphérique série exact, détectera la vitesse du récepteur, et posera les questions propres à la tondeuse (datum, NTRIP, station) de façon interactive.`,
      },
    },
    {
      id: "install-run",
      title: { en: "Run the installer", fr: "Lancer l'installateur" },
      body: {
        en: `SSH into the compute board (mower on, everything wired) and paste the command. It is interactive; take it prompt by prompt:

1. **Language** — English or French.
2. It clones the repository into \`~/mowglinext\`, installs **Docker** if needed, and writes the **udev rules** that create the stable device names \`/dev/mowgli\` (mainboard), \`/dev/gps\` and \`/dev/lidar\`.
3. **UART setup** (Raspberry Pi): adds \`enable_uart=1\`, \`dtoverlay=uart1…uart5\`, disables Bluetooth. On a Pi 5 it also sets USB max current and a fan curve.
4. **GNSS**: connection USB (pick the \`/dev/serial/by-id/…\` entry) or UART (pick the port, default \`/dev/ttyAMA4\`); baud auto-probe. Then **NTRIP** host, port, mount point, user, password.
5. **LiDAR**: type and port (default \`/dev/ttyAMA5\` for UART).
6. **Mower configuration**: GPS datum (you can leave it for the wizard's "use current position" — say so when asked), dock pose (leave defaults, calibrated later).
7. Optional tools (lazydocker), then it **pulls the container images** — several GB, this is the long part — and starts the stack.

At the end it prints a summary and any issue it found. If it asked for a reboot (UART overlays), reboot now:

\`\`\`
sudo reboot
\`\`\`

> [!TIP] Re-running is safe
> The installer is idempotent: run it again any time to change a sensor, and \`cd ~/mowglinext/install && ./mowglinext.sh --check\` runs the diagnostics only.`,
        fr: `Connectez-vous en SSH à la carte de calcul (tondeuse allumée, tout câblé) et collez la commande. Elle est interactive ; prenez-la question par question :

1. **Langue** — anglais ou français.
2. Elle clone le dépôt dans \`~/mowglinext\`, installe **Docker** si besoin, et écrit les **règles udev** qui créent les noms stables \`/dev/mowgli\` (carte mère), \`/dev/gps\` et \`/dev/lidar\`.
3. **Configuration UART** (Raspberry Pi) : ajoute \`enable_uart=1\`, \`dtoverlay=uart1…uart5\`, désactive le Bluetooth. Sur un Pi 5 elle règle aussi le courant USB max et une courbe de ventilateur.
4. **GNSS** : connexion USB (choisissez l'entrée \`/dev/serial/by-id/…\`) ou UART (choisissez le port, défaut \`/dev/ttyAMA4\`) ; détection automatique de la vitesse. Puis **NTRIP** hôte, port, point de montage, identifiant, mot de passe.
5. **LiDAR** : type et port (défaut \`/dev/ttyAMA5\` en UART).
6. **Configuration tondeuse** : datum GPS (vous pouvez le laisser pour le bouton « utiliser la position actuelle » de l'assistant — dites-le quand on vous le demande), position de la station (laissez les défauts, calibrée plus tard).
7. Outils optionnels (lazydocker), puis elle **télécharge les images de conteneurs** — plusieurs Go, c'est la partie longue — et démarre la pile.

À la fin elle affiche un résumé et les problèmes trouvés. Si elle a demandé un redémarrage (overlays UART), redémarrez maintenant :

\`\`\`
sudo reboot
\`\`\`

> [!TIP] Relancer est sans risque
> L'installateur est idempotent : relancez-le quand vous voulez pour changer un capteur, et \`cd ~/mowglinext/install && ./mowglinext.sh --check\` lance seulement les diagnostics.`,
      },
    },
    {
      id: "install-verify",
      title: { en: "Check that everything talks", fr: "Vérifier que tout communique" },
      body: {
        en: `After the reboot, give the stack two minutes, then from SSH:

\`\`\`
ls -l /dev/mowgli /dev/gps /dev/lidar      # symlinks must exist (lidar only if enabled)
docker ps                                  # mowgli-ros2, mowgli-gps, mowgli-gui, mowgli-mqtt (+ mowgli-lidar)
cd ~/mowglinext/install && ./mowglinext.sh --check
\`\`\`

\`--check\` reports each device, container and config file, and tells you exactly what to fix (a missing UART needs the reboot; a missing symlink needs the udev rule; a receiver on the wrong baud gets re-probed).

Then open the web interface from a phone or laptop on the same Wi-Fi:

\`\`\`
http://<mower-ip>:4006
\`\`\`

The **onboarding wizard** opens automatically on first visit. That is the next chapter.

Useful commands for later (installed by the installer):

| Command | Does |
|---|---|
| \`mowgli-up\` / \`mowgli-down\` | start / stop the stack |
| \`mowgli-logs\` | follow all container logs |
| \`mowgli-pull\` | pull newer images (Settings → Updates does this from the GUI) |`,
        fr: `Après le redémarrage, laissez deux minutes à la pile, puis en SSH :

\`\`\`
ls -l /dev/mowgli /dev/gps /dev/lidar      # les liens doivent exister (lidar seulement si activé)
docker ps                                  # mowgli-ros2, mowgli-gps, mowgli-gui, mowgli-mqtt (+ mowgli-lidar)
cd ~/mowglinext/install && ./mowglinext.sh --check
\`\`\`

\`--check\` passe en revue chaque périphérique, conteneur et fichier de configuration, et dit exactement quoi corriger (un UART absent demande le redémarrage ; un lien manquant, la règle udev ; un récepteur sur la mauvaise vitesse est re-sondé).

Ouvrez ensuite l'interface web depuis un téléphone ou un portable sur le même Wi-Fi :

\`\`\`
http://<ip-tondeuse>:4006
\`\`\`

L'**assistant de démarrage** s'ouvre automatiquement à la première visite. C'est le chapitre suivant.

Commandes utiles pour plus tard (installées par l'installateur) :

| Commande | Effet |
|---|---|
| \`mowgli-up\` / \`mowgli-down\` | démarrer / arrêter la pile |
| \`mowgli-logs\` | suivre les journaux de tous les conteneurs |
| \`mowgli-pull\` | récupérer des images plus récentes (Réglages → Mises à jour le fait depuis l'interface) |`,
      },
    },
  ],
});
