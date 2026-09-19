MOWGLI_MANUAL.chapters.push({
  id: "onboarding",
  icon: "🧙",
  title: { en: "Onboarding wizard", fr: "Assistant de démarrage" },
  media: {
    type: "img",
    src: "../gui-walkthrough/screenshots/onboarding/01-welcome.png",
    alt: { en: "Onboarding wizard welcome screen", fr: "Écran d'accueil de l'assistant" },
  },
  steps: [
    {
      id: "onb-welcome",
      title: { en: "Welcome — what the wizard does", fr: "Bienvenue — ce que fait l'assistant" },
      body: {
        en: `The wizard walks through nine steps, front to back, so that each step has what it needs from the previous one:

1. **Welcome**
2. **Robot model** — chassis preset
3. **Firmware** — flash the mainboard (chapter 8)
4. **NTRIP** — correction source
5. **GPS** — receiver, port, baud
6. **Datum** — map origin, requires RTK Fixed
7. **Sensors** — antenna / IMU / LiDAR offsets, LiDAR on/off
8. **Calibration** — IMU mounting yaw + dock pose (the robot drives)
9. **Complete** — a readiness checklist that gates "Finish & apply"

Nothing is final: every value is editable later in **Settings**. The wizard can be reopened from Settings at any time (\`#/onboarding\`).

Do the first five steps indoors if you like; from **Datum** on, the mower must be **outside, on its dock, with a clear sky**.`,
        fr: `L'assistant enchaîne neuf étapes, de l'avant vers l'arrière, pour que chacune dispose de ce que la précédente a produit :

1. **Bienvenue**
2. **Modèle de robot** — préréglage châssis
3. **Firmware** — flasher la carte mère (chapitre 8)
4. **NTRIP** — source de corrections
5. **GPS** — récepteur, port, vitesse
6. **Datum** — origine de la carte, exige RTK Fixed
7. **Capteurs** — offsets antenne / IMU / LiDAR, LiDAR actif ou non
8. **Calibration** — cap de montage IMU + position de la station (le robot roule)
9. **Terminé** — une checklist de préparation qui conditionne « Terminer et appliquer »

Rien n'est définitif : chaque valeur reste modifiable dans **Réglages**. L'assistant se rouvre depuis Réglages à tout moment (\`#/onboarding\`).

Faites les cinq premières étapes à l'intérieur si vous voulez ; à partir de **Datum**, la tondeuse doit être **dehors, sur sa station, ciel dégagé**.`,
      },
    },
    {
      id: "onb-model",
      title: { en: "Robot model", fr: "Modèle de robot" },
      media: {
        type: "img",
        src: "../gui-walkthrough/screenshots/onboarding/02-robot-model.png",
        alt: { en: "Robot model step", fr: "Étape Modèle de robot" },
      },
      body: {
        en: `Pick your chassis. The preset fills wheel radius, wheel track, blade radius / cutting width, encoder ticks per metre, battery thresholds and the default sensor offsets. Three fields are shown for a sanity check:

| Field | YardForce 500 / 500B |
|---|---|
| Wheel radius | 0.100 m (measured; older presets said 0.04475) |
| Wheel track | 0.325 m |
| Blade radius | 0.090 m → tool width 0.18 m |

Leave them unless you have changed the wheels. *Custom Robot* exposes every parameter.`,
        fr: `Choisissez votre châssis. Le préréglage remplit le rayon des roues, la voie, le rayon de lame / largeur de coupe, les ticks encodeur par mètre, les seuils batterie et les offsets capteurs par défaut. Trois champs sont affichés pour vérification :

| Champ | YardForce 500 / 500B |
|---|---|
| Rayon de roue | 0,100 m (mesuré ; les anciens préréglages disaient 0,04475) |
| Voie | 0,325 m |
| Rayon de lame | 0,090 m → largeur de coupe 0,18 m |

Laissez-les tels quels sauf si vous avez changé les roues. *Robot custom* expose tous les paramètres.`,
      },
    },
    {
      id: "onb-firmware",
      title: { en: "Firmware", fr: "Firmware" },
      media: {
        type: "img",
        src: "../gui-walkthrough/screenshots/onboarding/05-firmware.png",
        alt: { en: "Firmware step", fr: "Étape Firmware" },
      },
      body: {
        en: `Flash the mainboard as described in **chapter 8, step 3**. When the handshake shows a firmware version and *protocol compatible*, continue.

If the board reports *incompatible* after a successful flash, the USB re-enumeration failed — see chapter 13 (*No firmware data after flashing*).`,
        fr: `Flashez la carte mère comme décrit au **chapitre 8, étape 3**. Quand la poignée de main affiche une version de firmware et *protocole compatible*, continuez.

Si la carte se dit *incompatible* après un flash réussi, la ré-énumération USB a échoué — voir chapitre 13 (*Plus de données firmware après le flash*).`,
      },
    },
    {
      id: "onb-ntrip-gps",
      title: { en: "NTRIP and GPS", fr: "NTRIP et GPS" },
      media: {
        type: "img",
        src: "../gui-walkthrough/screenshots/onboarding/07-ntrip.png",
        alt: { en: "NTRIP step: pick a network and the nearest base on the map", fr: "Étape NTRIP : choisir un réseau et la base la plus proche sur la carte" },
        caption: { en: "NTRIP step — pick a free network and click the nearest base station; the GPS step follows.", fr: "Étape NTRIP — choisissez un réseau gratuit et cliquez la base la plus proche ; l'étape GPS suit." },
      },
      body: {
        en: `**NTRIP** — the values you prepared in chapter 6 (host, port, mount point, credentials). The installer may have filled them already; check and save.

**GPS** — receiver family (auto / u-blox / Unicore), serial device and baud as chosen in the installer, plus a **signal profile** (leave the default multi-band profile). Saving this step restarts the GPS container if anything changed; the live status card at the top then shows the fix type:

- *No fix* → *3D fix* within a minute outdoors;
- *RTK Float* once corrections arrive (the \`/rtcm\` counter moves);
- *RTK Fixed* after 1–5 minutes with a good sky view.

You need **RTK Fixed** for the next step. If it never comes, jump to chapter 13 (*Never reaches RTK Fixed*) before going further — everything after this depends on it.`,
        fr: `**NTRIP** — les valeurs préparées au chapitre 6 (hôte, port, point de montage, identifiants). L'installateur les a peut-être déjà remplies ; vérifiez et enregistrez.

**GPS** — famille de récepteur (auto / u-blox / Unicore), périphérique série et vitesse choisis dans l'installateur, plus un **profil de signaux** (laissez le profil multi-bandes par défaut). Enregistrer cette étape redémarre le conteneur GPS si quelque chose a changé ; la carte d'état en haut affiche ensuite le type de fix :

- *No fix* → *3D fix* en une minute dehors ;
- *RTK Float* dès que les corrections arrivent (le compteur \`/rtcm\` bouge) ;
- *RTK Fixed* après 1–5 minutes avec un bon ciel.

Il vous faut **RTK Fixed** pour l'étape suivante. S'il n'arrive jamais, passez au chapitre 13 (*N'atteint jamais RTK Fixed*) avant d'aller plus loin — tout ce qui suit en dépend.`,
      },
    },
    {
      id: "onb-datum",
      title: { en: "Datum — the map origin", fr: "Datum — l'origine de la carte" },
      media: {
        type: "img",
        src: "../gui-walkthrough/screenshots/onboarding/08-datum.png",
        alt: { en: "Datum step of the onboarding wizard", fr: "Étape Datum de l'assistant" },
        caption: { en: "The capture button stays disabled until the receiver reports RTK Fixed.", fr: "Le bouton de capture reste désactivé tant que le récepteur n'annonce pas RTK Fixed." },
      },
      body: {
        en: `The **datum** is the latitude/longitude that becomes (0, 0) of the robot's map. Every area polygon and the dock pose are stored relative to it, so it must be set **once** and then never change (if it does, the map is re-projected automatically, but do not make a habit of it).

With the mower on its dock and the status card showing **RTK Fixed**, click **Use current position**. The button is disabled until the fix is Fixed — on purpose: a Float datum would put your whole map off by decimetres.

You can type coordinates instead (9 decimals), but the current RTK position is better than anything you read off a map.

The wizard refuses to leave this step with a (0, 0) datum.`,
        fr: `Le **datum** est la latitude/longitude qui devient le (0, 0) de la carte du robot. Chaque polygone de zone et la position de la station sont stockés par rapport à lui, il doit donc être fixé **une fois** et ne plus changer (s'il change, la carte est reprojetée automatiquement, mais n'en prenez pas l'habitude).

Tondeuse sur sa station et carte d'état en **RTK Fixed**, cliquez **Utiliser la position actuelle**. Le bouton est désactivé tant que le fix n'est pas Fixed — volontairement : un datum en Float décalerait toute votre carte de plusieurs décimètres.

Vous pouvez saisir des coordonnées (9 décimales), mais la position RTK actuelle vaut mieux que tout ce que vous lirez sur une carte.

L'assistant refuse de quitter cette étape avec un datum (0, 0).`,
      },
    },
    {
      id: "onb-sensors",
      title: { en: "Sensors — offsets", fr: "Capteurs — offsets" },
      media: {
        type: "img",
        src: "../gui-walkthrough/screenshots/settings/03-sensors.png",
        alt: { en: "Sensors settings with the robot diagram", fr: "Réglages Capteurs avec le schéma du robot" },
        caption: { en: "The same sensor-placement editor is available later under Settings → Sensors.", fr: "Le même éditeur de placement des capteurs est disponible ensuite dans Réglages → Capteurs." },
      },
      body: {
        en: `Enter what you measured in chapters 5–7 on the robot diagram: GNSS antenna (\`gps_x/y/z\`), IMU position, LiDAR position and yaw, and whether the **LiDAR is enabled**.

Conventions (see the axes diagram in chapter 5): origin at the rear axle centre, X forward, Y **left**, Z up, metres. A sensor on the right has a negative Y.

Leave \`imu_yaw\` alone — the next step solves it. If you use a magnetometer, tick *use magnetometer* so the readiness list includes its calibration.

> [!NOTE] LiDAR switch
> This toggle is what the robot software reads (\`lidar_enabled\`). The installer's LiDAR choice only decides whether the driver container runs; both must agree. With the toggle on and no container, the diagnostics warn "no scan received".`,
        fr: `Saisissez ce que vous avez mesuré aux chapitres 5–7 sur le schéma du robot : antenne GNSS (\`gps_x/y/z\`), position de l'IMU, position et cap du LiDAR, et si le **LiDAR est activé**.

Conventions (voir le schéma des axes au chapitre 5) : origine au centre de l'essieu arrière, X vers l'avant, Y vers la **gauche**, Z vers le haut, en mètres. Un capteur à droite a un Y négatif.

Ne touchez pas à \`imu_yaw\` — l'étape suivante le résout. Si vous utilisez un magnétomètre, cochez *utiliser le magnétomètre* pour que la liste de préparation inclue sa calibration.

> [!NOTE] Interrupteur LiDAR
> Ce bouton est ce que lit le logiciel du robot (\`lidar_enabled\`). Le choix LiDAR de l'installateur décide seulement si le conteneur pilote tourne ; les deux doivent être cohérents. Bouton actif sans conteneur, les diagnostics avertissent « no scan received ».`,
      },
    },
    {
      id: "onb-calibration",
      title: { en: "Calibration — IMU yaw and dock pose", fr: "Calibration — cap IMU et position de la station" },
      body: {
        en: `> [!WARNING] The robot moves in this step
> Be next to it, blades still off, one metre clear in front of and behind the dock.

The IMU's mounting angle relative to the robot's forward axis cannot be measured accurately by hand. The wizard solves it by driving: the robot backs **0.6 m** off the dock, drives forward again, and compares the gyro-integrated heading with the GNSS track. Click **Start**, watch it, then **Apply** the solved \`imu_yaw\`.

As a side effect the same run captures the **dock pose** (\`dock_pose_x/y/yaw\`): where the charger contacts are and which way the robot faces on them. That is what the docking behaviour aims for later. Both values are written into the robot's config file; the Diagnostics page shows *Dock pose: Present* afterwards.

Requirements: RTK Fixed, the robot charging on the dock, no emergency latched. If the run aborts, check the *Emergency* indicator on the dashboard (a stop button pressed, a lift sensor) and retry.`,
        fr: `> [!WARNING] Le robot bouge à cette étape
> Restez à côté, lames toujours retirées, un mètre libre devant et derrière la station.

L'angle de montage de l'IMU par rapport à l'axe du robot ne se mesure pas précisément à la main. L'assistant le résout en roulant : le robot recule de **0,6 m** hors de la station, revient, et compare le cap intégré du gyro à la trajectoire GNSS. Cliquez **Démarrer**, surveillez, puis **Appliquer** le \`imu_yaw\` résolu.

Au passage, la même course capture la **position de la station** (\`dock_pose_x/y/yaw\`) : où sont les contacts du chargeur et dans quel sens le robot y est orienté. C'est ce que vise ensuite le retour en station. Les deux valeurs sont écrites dans le fichier de configuration du robot ; la page Diagnostics affiche ensuite *Dock pose : Present*.

Prérequis : RTK Fixed, robot en charge sur la station, pas d'urgence verrouillée. Si la course s'interrompt, vérifiez l'indicateur *Urgence* du tableau de bord (bouton STOP enfoncé, capteur de levage) et réessayez.`,
      },
    },
    {
      id: "onb-complete",
      title: { en: "Complete — the readiness checklist", fr: "Terminé — la checklist de préparation" },
      body: {
        en: `The last step is a live checklist. Required items gate **Finish & apply**; each failing one has a button that jumps back to the step that fixes it.

| Check | Comes from |
|---|---|
| RTK-Fixed GPS | receiver + antenna + NTRIP |
| NTRIP corrections flowing | NTRIP step |
| Map origin (datum) set | Datum step |
| Localizer running / confident | starts once the datum exists |
| Firmware compatible | Firmware step |
| Dock pose captured | Calibration step |
| IMU calibrated (bias) | automatic on the dock |
| IMU mounting yaw solved | Calibration step |
| Magnetometer calibrated | only if enabled |
| At least one mowing area | recorded on the Map page — chapter 12 |

**Finish & apply** writes the settings and restarts the ROS stack. Then read chapter 11: the wizard does **not** run drive tuning, and it does not record an area.`,
        fr: `La dernière étape est une checklist en direct. Les éléments requis conditionnent **Terminer et appliquer** ; chacun en échec a un bouton qui renvoie à l'étape qui le corrige.

| Vérification | Vient de |
|---|---|
| GPS RTK Fixed | récepteur + antenne + NTRIP |
| Corrections NTRIP reçues | étape NTRIP |
| Origine de carte (datum) définie | étape Datum |
| Localisateur actif / confiant | démarre dès que le datum existe |
| Firmware compatible | étape Firmware |
| Position de station capturée | étape Calibration |
| IMU calibrée (biais) | automatique sur la station |
| Cap de montage IMU résolu | étape Calibration |
| Magnétomètre calibré | seulement si activé |
| Au moins une zone de tonte | enregistrée sur la page Carte — chapitre 12 |

**Terminer et appliquer** écrit les réglages et redémarre la pile ROS. Lisez ensuite le chapitre 11 : l'assistant ne lance **pas** le réglage de traction, et il n'enregistre pas de zone.`,
      },
    },
  ],
});
