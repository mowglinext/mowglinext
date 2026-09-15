MOWGLI_MANUAL.chapters.push({
  id: "calibration",
  icon: "🎯",
  title: { en: "First-boot checks and tuning", fr: "Vérifications et réglages du premier démarrage" },
  media: {
    type: "img",
    src: "../gui-walkthrough/screenshots/diagnostics/01-overview.png",
    alt: { en: "Diagnostics page", fr: "Page Diagnostics" },
  },
  steps: [
    {
      id: "cal-diagnostics",
      title: { en: "Diagnostics: everything green", fr: "Diagnostics : tout au vert" },
      media: {
        type: "img",
        src: "../gui-walkthrough/screenshots/diagnostics/03-bt-coverage-network.png",
        alt: { en: "Diagnostics → Calibration tab", fr: "Diagnostics → onglet Calibration" },
        caption: { en: "Diagnostics → Calibration: cross-checks, dock pose, IMU bias, magnetometer.", fr: "Diagnostics → Calibration : vérifications croisées, position de station, biais IMU, magnétomètre." },
      },
      body: {
        en: `Open **Diagnostics** (\`#/diagnostics\`). Within a minute of a restart you should see:

- **System:** every container running (\`mowgli\`, \`gps\`, \`gui\`, \`mqtt\`, \`lidar\` if enabled), CPU temperature under ~70 °C.
- **hardware_bridge:** OK, serial link open, firmware compatible, status packets at 4 Hz.
- **GNSS:** fix type **RTK Fixed**, corrections age under a few seconds, 5 Hz.
- **Fusion graph** (the localizer): nodes counting up, position covariance in the centimetre range, both TF transforms published.
- **Sensors:** IMU rates and accelerations moving when you nudge the robot; wheel encoders counting when you push it.
- **Calibration status:** dock pose *Present*, IMU yaw *Solved*, IMU bias *Calibrated*.
- **LiDAR** (if enabled): \`/scan\` at ~10 Hz.

Anything red has a tooltip with the reason. The **Configuration cross-checks** card flags an unset datum or dock heading explicitly.`,
        fr: `Ouvrez **Diagnostics** (\`#/diagnostics\`). Dans la minute qui suit un redémarrage, vous devez voir :

- **Système :** tous les conteneurs actifs (\`mowgli\`, \`gps\`, \`gui\`, \`mqtt\`, \`lidar\` si activé), température CPU sous ~70 °C.
- **hardware_bridge :** OK, liaison série ouverte, firmware compatible, paquets d'état à 4 Hz.
- **GNSS :** type de fix **RTK Fixed**, âge des corrections de quelques secondes, 5 Hz.
- **Fusion graph** (le localisateur) : nombre de nœuds qui augmente, covariance de position au centimètre, les deux transformations TF publiées.
- **Capteurs :** vitesses et accélérations IMU qui bougent quand vous secouez le robot ; encodeurs de roues qui comptent quand vous le poussez.
- **État des calibrations :** position de station *Present*, cap IMU *Solved*, biais IMU *Calibrated*.
- **LiDAR** (si activé) : \`/scan\` à ~10 Hz.

Tout ce qui est rouge a une info-bulle avec la raison. La carte **Vérifications croisées de configuration** signale explicitement un datum ou un cap de station non définis.`,
      },
    },
    {
      id: "cal-imu-tilt",
      title: { en: "IMU bias and mounting tilt", fr: "Biais IMU et inclinaison de montage" },
      body: {
        en: `Whenever the robot charges on the dock, the hardware bridge samples the IMU for ~2 s and removes the mean bias; it repeats every 10 minutes while docked and once after 15 s of standing still elsewhere. The result is saved on the Pi, so restarts do not lose it.

Look in the ROS container log (\`mowgli-logs\` or the GUI's Logs page) for:

\`\`\`
IMU calibration complete (200 samples) ...
Implied mounting tilt: pitch=+X.XX°, roll=+X.XX° ...
\`\`\`

If pitch or roll is larger than ~1°, the IMU is physically tilted: either re-seat it flat (better) or copy the values into **Settings → Sensors → imu_pitch / imu_roll**. Under 1° is chip bias and is already handled.`,
        fr: `Chaque fois que le robot charge sur la station, le pont matériel échantillonne l'IMU pendant ~2 s et retire le biais moyen ; il recommence toutes les 10 minutes sur la station et une fois après 15 s d'immobilité ailleurs. Le résultat est sauvegardé sur le Pi, un redémarrage ne le perd pas.

Cherchez dans le journal du conteneur ROS (\`mowgli-logs\` ou la page Journaux de l'interface) :

\`\`\`
IMU calibration complete (200 samples) ...
Implied mounting tilt: pitch=+X.XX°, roll=+X.XX° ...
\`\`\`

Si le tangage ou le roulis dépasse ~1°, l'IMU est physiquement inclinée : remettez-la à plat (mieux) ou copiez les valeurs dans **Réglages → Capteurs → imu_pitch / imu_roll**. Sous 1°, c'est le biais de la puce, déjà compensé.`,
      },
    },
    {
      id: "cal-drive",
      title: { en: "Drive tuning (do not skip)", fr: "Réglage de la traction (à ne pas sauter)" },
      media: {
        type: "img",
        src: "../gui-walkthrough/screenshots/settings/13-drive-motor.png",
        alt: { en: "Settings → Drive Motor", fr: "Réglages → Moteurs de traction" },
      },
      body: {
        en: `> [!WARNING] The robot drives a few metres
> Level, open ground, blades off, you next to it. These assistants drive on a dedicated lane that the collision monitor does **not** filter.

The robot ships with generic wheel gains and a nominal odometry scale. Two assistants in **Settings → Drive Motor → Calibration assistants** fix that — they are **not** part of the wizard and are easy to miss:

1. **Calibrate odometry / feed-forward** — drives a few metres, compares encoder ticks with the RTK-measured distance, learns \`ticks_per_meter\` and the PWM-per-m/s feed-forward. Run it first.
2. **Auto-tune drive PID** — step-response tuning of the per-wheel speed loop that runs in the firmware. Optional but recommended: the default template gains cannot even overcome the motors' PWM deadband on some units (one wheel refuses to turn on tight arcs).

Each run shows before/after numbers and offers a one-click **rollback**. They refuse to start with an emergency latched, and refuse to leave the dock unless you allow it.

Skipping this is possible for a first try, but expect wavy stripes and drift until it is done.`,
        fr: `> [!WARNING] Le robot roule quelques mètres
> Terrain plat et dégagé, lames retirées, vous à côté. Ces assistants roulent sur une voie dédiée que le moniteur de collision ne filtre **pas**.

Le robot est livré avec des gains de roues génériques et une échelle d'odométrie nominale. Deux assistants dans **Réglages → Moteurs de traction → Assistants de calibration** corrigent cela — ils ne font **pas** partie de l'assistant de démarrage et sont faciles à rater :

1. **Calibrer l'odométrie / feed-forward** — roule quelques mètres, compare les ticks encodeur à la distance mesurée en RTK, apprend \`ticks_per_meter\` et le feed-forward PWM par m/s. À lancer en premier.
2. **Auto-réglage PID de traction** — réglage par réponse indicielle de la boucle de vitesse par roue qui tourne dans le firmware. Optionnel mais recommandé : les gains par défaut du modèle n'arrivent même pas à franchir la zone morte PWM des moteurs sur certaines machines (une roue refuse de tourner dans les virages serrés).

Chaque exécution affiche les valeurs avant/après et propose un **retour arrière** en un clic. Ils refusent de démarrer avec une urgence verrouillée, et refusent de quitter la station sans votre accord.

On peut sauter cette étape pour un premier essai, mais attendez-vous à des bandes ondulées et à de la dérive tant que ce n'est pas fait.`,
      },
    },
    {
      id: "cal-dock",
      title: { en: "Dock calibration (if you skipped it, or moved the dock)", fr: "Calibration de la station (si sautée, ou station déplacée)" },
      body: {
        en: `The dock pose was captured during the wizard's calibration drive. If it shows *Missing* in Diagnostics, or you move the charging base, redo it with one click: **Map page → dock icon → Set docking point** while the robot is charging on the dock under RTK Fixed. The service refuses a Float fix, because the dock pose is what the docking approach steers to within a few centimetres.

The heading is a map-frame angle solved from the calibration reverse leg, not a compass bearing — do not type one by hand.`,
        fr: `La position de la station a été capturée pendant la course de calibration de l'assistant. Si elle est *Missing* dans Diagnostics, ou si vous déplacez la base de charge, refaites-la en un clic : **page Carte → icône station → Définir le point de station** pendant que le robot charge sur la station en RTK Fixed. Le service refuse un fix Float, car c'est cette position que l'approche de station vise à quelques centimètres près.

Le cap est un angle dans le repère de la carte, résolu à partir de la marche arrière de calibration, pas un relèvement de boussole — ne le saisissez pas à la main.`,
      },
    },
  ],
});
