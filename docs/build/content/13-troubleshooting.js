MOWGLI_MANUAL.chapters.push({
  id: "troubleshooting",
  icon: "🛠️",
  title: { en: "Troubleshooting", fr: "Dépannage" },
  steps: [
    {
      id: "ts-rtk",
      title: { en: "Never reaches RTK Fixed", fr: "N'atteint jamais RTK Fixed" },
      body: {
        en: `Work down the list; each line has fixed real builds.

| Symptom | Cause → fix |
|---|---|
| Stuck at *3D fix*, corrections counter at 0 | NTRIP not connected: wrong host/port/mount point, caster needs a non-empty user/password, Wi-Fi does not reach the dock. Check the GNSS card's correction status. |
| *RTK Float* forever | Base too far (> 30–40 km) → pick a closer mount point. Single-band antenna → replace. Antenna under the shell edge / next to metal → move it up. Loose SMA → hand-tighten. Wet antenna → dry it. |
| Fixed ↔ Float flicker while moving | Normal on an F9P; the stack debounces it. Real drops in the same spot every time = trees/buildings shading that corner. |
| Fixed on the bench, Float in the mower | EMI from the wheel-motor leads or the DC-DC: shielded USB cable, route the antenna cable away, add the capacitor on the 5 V. |
| No \`/gps/fix\` at all | Wrong serial device or baud: rerun the installer, pick the \`/dev/serial/by-id\` path (USB) or the UART you wired, let it probe the baud. |

The receiver is validated at 921600 baud; a fresh F9P at 38400 works too until the installer raises it.`,
        fr: `Descendez la liste ; chaque ligne a débloqué de vrais montages.

| Symptôme | Cause → correction |
|---|---|
| Bloqué en *3D fix*, compteur de corrections à 0 | NTRIP non connecté : hôte/port/point de montage faux, le caster exige un identifiant/mot de passe non vides, le Wi-Fi n'atteint pas la station. Vérifiez l'état des corrections sur la carte GNSS. |
| *RTK Float* en permanence | Base trop loin (> 30–40 km) → point de montage plus proche. Antenne mono-bande → remplacer. Antenne sous le bord de la coque / près du métal → la monter. SMA desserré → serrer à la main. Antenne mouillée → la sécher. |
| Alternance Fixed ↔ Float en mouvement | Normal sur un F9P ; la pile le filtre. Des pertes au même endroit à chaque fois = arbres/bâtiments qui masquent ce coin. |
| Fixed sur l'établi, Float dans la tondeuse | Perturbations des fils moteurs ou du DC-DC : câble USB blindé, éloigner le câble d'antenne, ajouter le condensateur sur le 5 V. |
| Aucun \`/gps/fix\` | Périphérique série ou vitesse faux : relancez l'installateur, choisissez le chemin \`/dev/serial/by-id\` (USB) ou l'UART câblé, laissez-le sonder la vitesse. |

Le récepteur est validé à 921600 bauds ; un F9P neuf à 38400 fonctionne aussi jusqu'à ce que l'installateur monte la vitesse.`,
      },
    },
    {
      id: "ts-usb-reenum",
      title: { en: "No firmware data after flashing the STM32", fr: "Plus de données firmware après le flash du STM32" },
      body: {
        en: `**Symptom:** right after a flash, *all* firmware topics go silent at once — no IMU, no wheel odometry, no status — while the GUI and the rest of the stack are fine. \`dmesg\` shows repeated \`device descriptor read/64, error -62\` ending in \`unable to enumerate USB device\`, and \`/dev/mowgli\` is missing.

**Cause:** the board re-enumerates over USB after the reset and the host's USB stack occasionally fails it (EMI). The bridge is holding a dead handle.

**Fix without a power cycle** (Raspberry Pi; the STM32 sits on its own controller so the GNSS is not disturbed):

\`\`\`
echo fc840000.usb | sudo tee /sys/bus/platform/drivers/ohci-platform/unbind
sleep 2
echo fc840000.usb | sudo tee /sys/bus/platform/drivers/ohci-platform/bind
docker restart mowgli-ros2
\`\`\`

\`dmesg\` should then show \`Product: Mowgli\` and \`/dev/mowgli → ttyACM*\` reappears. On other boards, or if in doubt, unplug and re-plug the USB A-to-A cable, or power-cycle the mower.`,
        fr: `**Symptôme :** juste après un flash, *tous* les topics firmware se taisent d'un coup — plus d'IMU, plus d'odométrie, plus d'état — alors que l'interface et le reste de la pile vont bien. \`dmesg\` montre des \`device descriptor read/64, error -62\` répétés finissant par \`unable to enumerate USB device\`, et \`/dev/mowgli\` a disparu.

**Cause :** la carte se ré-énumère en USB après le reset et la pile USB de l'hôte échoue parfois (perturbations). Le pont tient un descripteur mort.

**Correction sans couper l'alimentation** (Raspberry Pi ; le STM32 est sur son propre contrôleur, le GNSS n'est pas perturbé) :

\`\`\`
echo fc840000.usb | sudo tee /sys/bus/platform/drivers/ohci-platform/unbind
sleep 2
echo fc840000.usb | sudo tee /sys/bus/platform/drivers/ohci-platform/bind
docker restart mowgli-ros2
\`\`\`

\`dmesg\` doit alors montrer \`Product: Mowgli\` et \`/dev/mowgli → ttyACM*\` réapparaît. Sur d'autres cartes, ou en cas de doute, débranchez/rebranchez le câble USB A vers A, ou coupez et rallumez la tondeuse.`,
      },
    },
    {
      id: "ts-stlink",
      title: { en: "ST-Link: \"init mode failed\" or not detected", fr: "ST-Link : « init mode failed » ou non détecté" },
      body: {
        en: `- \`Error: init mode failed (unable to connect to the target)\` → SWCLK and SWDIO are swapped, or GND is not connected. Re-read the label on the dongle. If the wiring is right, the dongle itself may be dead — several builders fixed it by swapping the clone.
- \`Debug adapter doesn't support 'swd' transport\` → an old OpenOCD with a V2.1 clone; the GUI's command already lets OpenOCD negotiate, so update the stack if you see this.
- Not in \`lsusb\` → try another USB port / cable; some clones need a USB 2 port.
- **Firmware incompatible — reflash required** after a successful flash → the board did not re-enumerate; see the previous step. If it persists after a power cycle, the stack and the firmware really are from different releases: run Settings → Updates.`,
        fr: `- \`Error: init mode failed (unable to connect to the target)\` → SWCLK et SWDIO inversés, ou GND non connecté. Relisez l'étiquette du dongle. Si le câblage est bon, le dongle lui-même peut être mort — plusieurs monteurs ont réglé le problème en changeant de clone.
- \`Debug adapter doesn't support 'swd' transport\` → un vieil OpenOCD avec un clone V2.1 ; la commande de l'interface laisse déjà OpenOCD négocier, mettez la pile à jour si vous voyez ceci.
- Absent de \`lsusb\` → autre port / câble USB ; certains clones exigent un port USB 2.
- **Firmware incompatible — reflash required** après un flash réussi → la carte ne s'est pas ré-énumérée ; voir l'étape précédente. Si cela persiste après un cycle d'alimentation, la pile et le firmware viennent vraiment de releases différentes : lancez Réglages → Mises à jour.`,
      },
    },
    {
      id: "ts-power",
      title: { en: "Board reboots, USB devices drop, no charge", fr: "Carte qui redémarre, USB qui décroche, pas de charge" },
      body: {
        en: `- **Board reboots when motors start** → DC-DC too weak or wires too long. 5 A converter, short twisted 5 V pair, 470–1000 µF capacitor at the board.
- **GNSS / LiDAR / ST-Link disappear randomly** → an unpowered USB hub. Plug them straight into the board; if a hub is unavoidable it needs its own ≥ 2 A supply.
- **Robot does not charge on the dock** → inspect the charge-contact and power connectors for burnt or loose pins (a bad crimp heats up), measure the voltage at the contacts. Also check that the dock pose points the robot squarely onto the contacts.
- **Blade motor does not spin** → the panel must be plugged in (its switches are in the blade chain). Then unplug and re-plug every mainboard connector: an intermittent motor connector is the usual culprit.
- **Wheels do nothing although the GUI is happy** → the firmware drops commands until the stack sends a non-idle state, and hard-stops after 200 ms without a command. Both are normal at rest; if it persists during a mow, look at the *Emergency* indicator (a stuck STOP button or lift sensor).`,
        fr: `- **La carte redémarre au démarrage des moteurs** → DC-DC trop faible ou fils trop longs. Convertisseur 5 A, paire 5 V torsadée courte, condensateur 470–1000 µF à la carte.
- **GNSS / LiDAR / ST-Link disparaissent aléatoirement** → hub USB non alimenté. Branchez-les directement sur la carte ; si un hub est inévitable, il lui faut sa propre alimentation ≥ 2 A.
- **Le robot ne charge pas sur la station** → inspectez les connecteurs de charge et d'alimentation (broches brûlées ou desserrées — un mauvais sertissage chauffe), mesurez la tension aux contacts. Vérifiez aussi que la position de station oriente le robot bien en face des contacts.
- **Le moteur de lame ne tourne pas** → le panneau doit être branché (ses interrupteurs sont dans la chaîne de la lame). Puis débranchez et rebranchez tous les connecteurs de la carte mère : un connecteur moteur intermittent est le coupable habituel.
- **Les roues ne bougent pas alors que l'interface va bien** → le firmware ignore les commandes tant que la pile n'envoie pas un état non-idle, et s'arrête après 200 ms sans commande. Les deux sont normaux au repos ; si cela persiste pendant une tonte, regardez l'indicateur *Urgence* (bouton STOP ou capteur de levage coincé).`,
      },
    },
    {
      id: "ts-lidar",
      title: { en: "LiDAR: no scan, or the robot stops for nothing", fr: "LiDAR : pas de scan, ou le robot s'arrête pour rien" },
      when: { lidar: ["yes"] },
      body: {
        en: `- **"No scan received" warning** → the driver container is not running (installer said *None*) while the Sensors toggle is on, or \`/dev/lidar\` points at the wrong UART. Check \`ls -l /dev/lidar\` and \`docker logs mowgli-lidar\`. Rerunning the installer once mis-pointed the symlink at a silent port; pick the port you actually wired.
- **Scan at 10 Hz on the bench, dropouts on the lawn** → UART cable too long or next to the motor leads; the behaviour tree pauses on a stale scan. Shorten / re-route, or switch to the USB adapter.
- **Stops or detours in open grass** → the LiDAR is tilted and sees the ground. Level it; the scan filter drops ground returns but only up to a point.
- **Ignores a real obstacle** → it is below the scan plane (0.30 m). Lower the mount a little, or draw a no-go zone.`,
        fr: `- **Avertissement « No scan received »** → le conteneur pilote ne tourne pas (installateur sur *None*) alors que le bouton Capteurs est actif, ou \`/dev/lidar\` pointe sur le mauvais UART. Vérifiez \`ls -l /dev/lidar\` et \`docker logs mowgli-lidar\`. Une relance de l'installateur a déjà pointé le lien sur un port muet ; choisissez le port réellement câblé.
- **Scan à 10 Hz sur l'établi, coupures sur la pelouse** → câble UART trop long ou le long des fils moteurs ; l'arbre de comportement se met en pause sur un scan périmé. Raccourcir / dérouter, ou passer sur l'adaptateur USB.
- **Arrêts ou détours dans l'herbe dégagée** → le LiDAR est incliné et voit le sol. Mettez-le de niveau ; le filtre de scan élimine les retours du sol, mais jusqu'à un certain point.
- **Ignore un vrai obstacle** → il est sous le plan de scan (0,30 m). Baissez un peu le support, ou dessinez une zone interdite.`,
      },
    },
    {
      id: "ts-nav",
      title: { en: "Navigation: wrong place, wavy stripes, missed dock", fr: "Navigation : mauvais endroit, bandes ondulées, station ratée" },
      body: {
        en: `| Symptom | Likely cause |
|---|---|
| The whole map is offset from the lawn | Datum changed or was captured in Float. Re-capture under Fixed; the map re-projects automatically. |
| Position swings sideways in turns, dock consistently missed by the same few cm | Wrong antenna offset (\`gps_x/gps_y\`). Re-measure to the antenna centre. |
| Heads off at an angle right after undocking | IMU yaw not solved or IMU moved. Rerun the calibration step. |
| Wavy stripes, one wheel hesitates on tight turns | Drive tuning not done. Run both assistants in Settings → Drive Motor. |
| Mission shows *WAITING_FOR_RTK* on and off | RTK dropping in one corner. Check the sky there, or lower the RTK requirement time-outs in Settings → Localization only if you know what you do. |
| *DIG_OBSTRUCTION* / repeated stops at one spot | The wheel-slip detector latched three times at the same place (soft ground, a root). HOME drives it out; remove the obstacle or draw a no-go zone. |
| Robot idles after 100 % with *START_OCCUPIED* | The last stripe ended on the boundary band; drive it a little forward from the Map page and press Home. |

For anything else: **Diagnostics → Export** produces a redacted bundle to attach to a GitHub issue.`,
        fr: `| Symptôme | Cause probable |
|---|---|
| Toute la carte est décalée par rapport à la pelouse | Datum changé ou capturé en Float. Recapturez en Fixed ; la carte se reprojette automatiquement. |
| Position qui balance en virage, station ratée toujours des mêmes cm | Offset d'antenne faux (\`gps_x/gps_y\`). Remesurez jusqu'au centre de l'antenne. |
| Part de travers juste après la sortie de station | Cap IMU non résolu ou IMU déplacée. Relancez l'étape de calibration. |
| Bandes ondulées, une roue hésite dans les virages serrés | Réglage de traction non fait. Lancez les deux assistants dans Réglages → Moteurs de traction. |
| Mission en *WAITING_FOR_RTK* par intermittence | RTK qui décroche dans un coin. Vérifiez le ciel à cet endroit, ou ajustez les délais RTK dans Réglages → Localisation seulement en connaissance de cause. |
| *DIG_OBSTRUCTION* / arrêts répétés au même endroit | Le détecteur de patinage s'est verrouillé trois fois au même endroit (sol meuble, racine). HOME l'en sort ; retirez l'obstacle ou dessinez une zone interdite. |
| Robot inactif après 100 % avec *START_OCCUPIED* | La dernière bande finit sur la bande de bordure ; avancez-le un peu depuis la page Carte et appuyez sur Home. |

Pour tout le reste : **Diagnostics → Exporter** produit un paquet anonymisé à joindre à un ticket GitHub.`,
      },
    },
  ],
});
