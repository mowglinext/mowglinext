MOWGLI_MANUAL.chapters.push({
  id: "lidar",
  icon: "🔴",
  title: { en: "LiDAR (optional)", fr: "LiDAR (optionnel)" },
  steps: [
    {
      id: "lidar-skip",
      title: { en: "Building without LiDAR", fr: "Monter sans LiDAR" },
      when: { lidar: ["no"] },
      body: {
        en: `You chose a build without LiDAR. Nothing to do in this chapter:

- answer **None** to the LiDAR question in the installer (chapter 9), and leave *LiDAR* off in the wizard;
- the stack runs the "no LiDAR" navigation profile: obstacle avoidance is off and the robot relies on the recorded boundary and the stop/lift sensors;
- you can add a LiDAR any time later: rerun the installer, switch it on in Settings → Sensors, done.

Switch **Your build → LiDAR** in the sidebar if you change your mind.`,
        fr: `Vous avez choisi un montage sans LiDAR. Rien à faire dans ce chapitre :

- répondez **None** à la question LiDAR de l'installateur (chapitre 9), et laissez *LiDAR* désactivé dans l'assistant ;
- la pile utilise le profil de navigation « sans LiDAR » : l'évitement d'obstacles est désactivé et le robot s'appuie sur la limite enregistrée et les capteurs STOP / levage ;
- vous pourrez ajouter un LiDAR plus tard : relancer l'installateur, l'activer dans Réglages → Capteurs, c'est tout.

Changez **Votre montage → LiDAR** dans la barre latérale si vous changez d'avis.`,
      },
    },
    {
      id: "lidar-mount",
      title: { en: "Mount the LD19", fr: "Fixer le LD19" },
      when: { lidar: ["yes"] },
      media: { type: "img", src: "img/axes.svg", alt: { en: "Sensor offsets", fr: "Positions des capteurs" } },
      body: {
        en: `The LD19 is a 360° 2D scanner. It has to see **over** the shell all around, so it sits on the highest point of the robot on a printed post, typically above the rear axle:

- **Height** ≈ 0.30 m above ground (\`lidar_z\`). Higher sees further over grass; lower catches low obstacles. 0.25–0.35 m is the useful range.
- **Level.** Tilt turns the lawn into an obstacle.
- **Position:** the 500 preset assumes it is straight above the axle, slightly left (\`lidar_x 0.00\`, \`lidar_y 0.025\`). Measure yours.
- **Orientation:** the LD19's cable exit marks its 0° direction. Mount it whichever way is convenient and set \`lidar_yaw\` accordingly — with the cable pointing backwards the preset value is \`−3.1416\` (180°).
- Keep the GNSS antenna out of its plane if you can, or accept a small blind sector — the scan filter ignores returns inside the robot footprint anyway.

Protect it from rain: a printed hood that leaves the rotating window clear. Do not enclose it in clear plastic; the laser does not like windows.`,
        fr: `Le LD19 est un scanner 2D à 360°. Il doit voir **par-dessus** la coque tout autour, donc il se place au point le plus haut du robot sur un mât imprimé, typiquement au-dessus de l'essieu arrière :

- **Hauteur** ≈ 0,30 m du sol (\`lidar_z\`). Plus haut voit plus loin par-dessus l'herbe ; plus bas attrape les obstacles bas. 0,25–0,35 m est la plage utile.
- **De niveau.** Une inclinaison transforme la pelouse en obstacle.
- **Position :** le préréglage 500 le suppose à l'aplomb de l'essieu, légèrement à gauche (\`lidar_x 0.00\`, \`lidar_y 0.025\`). Mesurez le vôtre.
- **Orientation :** la sortie de câble du LD19 marque sa direction 0°. Montez-le comme c'est pratique et réglez \`lidar_yaw\` en conséquence — câble vers l'arrière, le préréglage vaut \`−3.1416\` (180°).
- Gardez l'antenne GNSS hors de son plan si possible, ou acceptez un petit secteur aveugle — le filtre de scan ignore de toute façon les retours dans l'empreinte du robot.

Protégez-le de la pluie : une casquette imprimée qui laisse la fenêtre tournante dégagée. Ne l'enfermez pas dans du plastique transparent ; le laser n'aime pas les vitres.`,
      },
      parts: [{ qty: 1, name: { en: "LD19 + printed post and rain hood", fr: "LD19 + mât imprimé et casquette anti-pluie" } }],
    },
    {
      id: "lidar-wire",
      title: { en: "Wire the LiDAR", fr: "Câbler le LiDAR" },
      when: { lidar: ["yes"] },
      media: { type: "img", src: "img/pi-header.svg", alt: { en: "Pi header pins used", fr: "Broches du connecteur Pi utilisées" } },
      body: {
        en: `The LD19 outputs a **230400 baud UART, 3.3 V logic, TX only**, and needs 5 V. Two options; both end up as \`/dev/lidar\` for the driver container.

**UART on the Pi header (installer default \`/dev/ttyAMA5\` = UART5):**

| LD19 lead | Pi header |
|---|---|
| 5V (red) | pin 2 (5 V) |
| GND (black) | pin 9 (GND) |
| TX (data) | pin 33 (GPIO 13, RXD5) |
| PWM | not connected (the LD19 self-regulates) |

Choose *LDLiDAR / UART* in the composer. The port appears after the installer's reboot.

**USB:** use the small USB-to-serial adapter board that ships with the LD19, plug it into a Pi USB 2 port and choose *LDLiDAR / USB*. Simpler, one more USB cable.

The LD19 draws ~300 mA at start-up — the DC-DC sizing in chapter 4 already accounts for it.

> [!NOTE]
> RPLiDAR (A1/A2/C1) and STL27L use the same wiring pattern with their own baud rate; the installer sets it from the type you select.`,
        fr: `Le LD19 sort un **UART 230400 bauds, logique 3,3 V, TX seulement**, et demande du 5 V. Deux options ; les deux finissent en \`/dev/lidar\` pour le conteneur pilote.

**UART sur le connecteur du Pi (défaut installateur \`/dev/ttyAMA5\` = UART5) :**

| Fil LD19 | Connecteur Pi |
|---|---|
| 5V (rouge) | broche 2 (5 V) |
| GND (noir) | broche 9 (GND) |
| TX (données) | broche 33 (GPIO 13, RXD5) |
| PWM | non connecté (le LD19 se régule seul) |

Choisissez *LDLiDAR / UART* dans le composeur. Le port apparaît après le redémarrage de l'installateur.

**USB :** utilisez la petite carte adaptateur USB-série livrée avec le LD19, branchez-la sur un port USB 2 du Pi et choisissez *LDLiDAR / USB*. Plus simple, un câble USB de plus.

Le LD19 consomme ~300 mA au démarrage — le dimensionnement du DC-DC au chapitre 4 en tient déjà compte.

> [!NOTE]
> RPLiDAR (A1/A2/C1) et STL27L suivent le même schéma avec leur propre vitesse ; l'installateur la règle selon le type choisi.`,
      },
      parts: [{ qty: 3, name: { en: "Dupont wires (5 V, GND, TX) or the LD19 USB adapter", fr: "Fils Dupont (5 V, GND, TX) ou l'adaptateur USB du LD19" } }],
    },
    {
      id: "lidar-what",
      title: { en: "What the LiDAR is used for", fr: "À quoi sert le LiDAR" },
      when: { lidar: ["yes"] },
      body: {
        en: `Knowing what it does helps you judge whether it works later:

- **Obstacle avoidance while mowing.** The coverage controller (FTC) deviates the stripe **laterally** around anything the scan sees inside the mowing zone, and comes back to the line. A collision monitor stops the robot if something appears right in front of it.
- **Transit and docking:** the local costmap keeps the planner off obstacles.
- **Obstacle memory:** repeatedly-seen obstacles show up on the Map page as proposals you can promote to permanent no-go zones.
- **Map anchor (optional, off by default):** under good RTK the localizer learns georeferenced tiles of fixed features (walls, hedges); after a **complete** GNSS outage it can localize against them for a while instead of drifting on dead reckoning. Enable it in Settings → Localization once the rest works.

It does **not** replace RTK: coverage is still planned and driven from the GNSS position.`,
        fr: `Savoir ce qu'il fait vous aidera à juger plus tard s'il fonctionne :

- **Évitement d'obstacles pendant la tonte.** Le contrôleur de couverture (FTC) dévie la bande **latéralement** autour de tout ce que le scan voit dans la zone de tonte, puis revient sur la ligne. Un moniteur de collision arrête le robot si quelque chose surgit juste devant.
- **Transit et retour station :** la costmap locale tient le planificateur à l'écart des obstacles.
- **Mémoire d'obstacles :** les obstacles vus à répétition apparaissent sur la page Carte comme des propositions que vous pouvez promouvoir en zones interdites permanentes.
- **Ancre de carte (optionnelle, désactivée par défaut) :** sous bon RTK, le localisateur apprend des tuiles géoréférencées des éléments fixes (murs, haies) ; après une perte **complète** du GNSS, il peut se localiser dessus un moment au lieu de dériver en estime. Activez-la dans Réglages → Localisation quand le reste fonctionne.

Il ne remplace **pas** le RTK : la couverture reste planifiée et conduite à partir de la position GNSS.`,
      },
    },
  ],
});
