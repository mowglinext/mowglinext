MOWGLI_MANUAL.chapters.push({
  id: "first-mow",
  icon: "🌿",
  title: { en: "Map and first mow", fr: "Carte et première tonte" },
  media: {
    type: "img",
    src: "../gui-walkthrough/screenshots/map/01-map-overview.png",
    alt: { en: "Map page", fr: "Page Carte" },
  },
  steps: [
    {
      id: "mow-record",
      title: { en: "Record a mowing area", fr: "Enregistrer une zone de tonte" },
      body: {
        en: `Areas are recorded by **driving the boundary** from the web GUI on your phone — no game controller.

1. Blades still off. Open the **Map** page, More menu → **Record area**.
2. The robot enters recording mode. Use the on-screen joystick to drive it **along the edge of the lawn**, keeping the blade disc centre where you want the cut to end — the coverage planner insets the first stripe by half a cutting width from the line you record, and keeps the chassis inside it.
3. Close the loop back to the start and **finish**. The polygon is simplified and saved.
4. Repeat for every separate lawn. Areas are mowed one after the other.

Tips from the field:

- Drive **slowly** on curves; the polygon follows the fused RTK pose, so a fast corner rounds off.
- Keep **RTK Fixed** during the whole loop. If it drops to Float, stop, wait for Fixed, continue.
- Leave a margin along walls and fences: the robot is 0.45 m wide and does not know the wall is there without a LiDAR.
- Obstacles (trees, flower beds) inside an area are drawn afterwards as **no-go zones** on the Map page — no need to drive around them.`,
        fr: `Les zones s'enregistrent en **conduisant le long de la limite** depuis l'interface web sur votre téléphone — pas de manette.

1. Lames toujours retirées. Ouvrez la page **Carte**, menu Plus → **Enregistrer une zone**.
2. Le robot passe en mode enregistrement. Avec le joystick à l'écran, conduisez-le **le long du bord de la pelouse**, en gardant le centre du disque de lame là où vous voulez que la coupe s'arrête — le planificateur décale la première bande d'une demi-largeur de coupe vers l'intérieur de la ligne enregistrée, et garde le châssis dedans.
3. Refermez la boucle au point de départ et **terminez**. Le polygone est simplifié et sauvegardé.
4. Répétez pour chaque pelouse séparée. Les zones sont tondues l'une après l'autre.

Conseils du terrain :

- Roulez **lentement** dans les courbes ; le polygone suit la position RTK fusionnée, un virage rapide arrondit le coin.
- Gardez **RTK Fixed** pendant toute la boucle. S'il passe en Float, arrêtez, attendez le Fixed, continuez.
- Laissez une marge le long des murs et clôtures : le robot fait 0,45 m de large et ne sait pas que le mur est là sans LiDAR.
- Les obstacles (arbres, massifs) à l'intérieur d'une zone se dessinent ensuite comme **zones interdites** sur la page Carte — inutile de les contourner en roulant.`,
      },
    },
    {
      id: "mow-edit",
      title: { en: "Edit areas, no-go zones and the dock", fr: "Modifier les zones, zones interdites et la station" },
      media: {
        type: "img",
        src: "../gui-walkthrough/screenshots/map/03-edit-mode.png",
        alt: { en: "Map edit mode", fr: "Mode édition de la carte" },
      },
      body: {
        en: `The Map page's **edit mode** lets you drag vertices, rename areas, set a per-area **mow angle** (stripe direction; *auto* alternates it 90° every session — cross-hatch), and draw **no-go zones** inside an area. Save when done; the coverage plan is recomputed at the next start.

Two things the map also shows:

- the **dock** marker with its heading arrow — it should sit on the charger and point the way the robot faces when docked;
- **mow progress** shading during and after a run, so you can see what was cut.

Areas, no-go zones and the dock pose are stored relative to the datum in the robot's map volume. Back it up from **Settings → Advanced → Export** once you are happy.`,
        fr: `Le **mode édition** de la page Carte permet de déplacer les sommets, renommer les zones, fixer un **angle de tonte** par zone (direction des bandes ; *auto* l'alterne de 90° à chaque session — tonte croisée), et dessiner des **zones interdites** dans une zone. Enregistrez ; le plan de couverture est recalculé au prochain démarrage.

La carte montre aussi :

- le marqueur de **station** avec sa flèche de cap — il doit être sur le chargeur et pointer dans le sens du robot en station ;
- la **progression de tonte** pendant et après un passage, pour voir ce qui a été coupé.

Zones, zones interdites et position de station sont stockées par rapport au datum dans le volume de carte du robot. Sauvegardez-les depuis **Réglages → Avancé → Exporter** une fois satisfait.`,
      },
    },
    {
      id: "mow-blades",
      title: { en: "Blades back on — final checklist", fr: "Remise des lames — checklist finale" },
      body: {
        en: `Before the first autonomous mow, confirm every line. These are the things the wizard cannot guarantee by itself:

- [ ] **RTK Fixed** on the dock and a real **datum** (Settings → GPS shows non-zero lat/lon)
- [ ] **IMU bias** calibrated and **IMU yaw** solved (Diagnostics → Calibration status)
- [ ] **Dock pose** Present
- [ ] **Drive tuning** run (chapter 11) — or accept wavy stripes for a first try
- [ ] **At least one area** recorded, no-go zones drawn around obstacles
- [ ] Shell closed, every cable tied away from the blade disc and the wheels
- [ ] Wi-Fi reaches the whole lawn (walk it with your phone showing the dashboard)
- [ ] Nobody on the lawn

Now switch the mower off, **fit the three blades**, switch it back on and put it on the dock. The panel's STOP button and the lift sensors are live: press STOP once to make sure the dashboard shows the emergency, then release it (the emergency clears automatically once the robot is charging on the dock).`,
        fr: `Avant la première tonte autonome, confirmez chaque ligne. Ce sont les points que l'assistant ne peut pas garantir seul :

- [ ] **RTK Fixed** sur la station et un vrai **datum** (Réglages → GPS affiche une lat/lon non nulle)
- [ ] **Biais IMU** calibré et **cap IMU** résolu (Diagnostics → État des calibrations)
- [ ] **Position de station** Present
- [ ] **Réglage de traction** effectué (chapitre 11) — ou acceptez des bandes ondulées pour un premier essai
- [ ] **Au moins une zone** enregistrée, zones interdites dessinées autour des obstacles
- [ ] Capot fermé, tous les câbles attachés loin du disque de lame et des roues
- [ ] Le Wi-Fi couvre toute la pelouse (parcourez-la avec le tableau de bord ouvert sur votre téléphone)
- [ ] Personne sur la pelouse

Éteignez maintenant la tondeuse, **remontez les trois lames**, rallumez-la et posez-la sur la station. Le bouton STOP du panneau et les capteurs de levage sont actifs : appuyez une fois sur STOP pour vérifier que le tableau de bord affiche l'urgence, puis relâchez (l'urgence s'efface automatiquement dès que le robot charge sur la station).`,
      },
    },
    {
      id: "mow-start",
      title: { en: "Start the first mow", fr: "Lancer la première tonte" },
      media: {
        type: "img",
        src: "../gui-walkthrough/screenshots/dashboard/01-overview.png",
        alt: { en: "Dashboard during a mow", fr: "Tableau de bord pendant une tonte" },
      },
      body: {
        en: `Press **Start** on the dashboard. What the robot does, so you can tell normal from wrong:

1. Clears the emergency latch if one is still held, waits for a GNSS fix.
2. **Undocks** by backing up 1.5 m, then waits for **RTK Fixed** (up to 20 s).
3. Plans the whole area once (headland rings, then stripes joined by turn-around arcs), transits **blade off** to the start of the first path.
4. Spins the blade up and drives each path end-to-end. Stripes should be straight and evenly spaced (0.16 m apart with a 0.18 m blade).
5. Between areas, and at the end, drives blade-off to the dock and docks. Battery low → docks, recharges, resumes where it stopped.

Watch the first lap of the headland ring closely. Stop it with **Stop** on the dashboard or the panel's STOP button if it heads for the fence: that is either a wrong datum (map offset everywhere), a wrong antenna offset (position swings in turns) or a bad IMU yaw (heads off at an angle from the start).

Progress is persisted, so an interrupted mow resumes at the same spot. The **Statistics** page logs every session.`,
        fr: `Appuyez sur **Démarrer** sur le tableau de bord. Ce que fait le robot, pour distinguer le normal de l'anormal :

1. Efface l'urgence si elle est encore verrouillée, attend un fix GNSS.
2. **Sort de la station** en reculant de 1,5 m, puis attend **RTK Fixed** (jusqu'à 20 s).
3. Planifie toute la zone une fois (anneaux de bordure, puis bandes reliées par des demi-tours), transite **lame arrêtée** vers le début du premier chemin.
4. Lance la lame et parcourt chaque chemin d'un bout à l'autre. Les bandes doivent être droites et régulières (espacées de 0,16 m avec une lame de 0,18 m).
5. Entre les zones et à la fin, rentre lame arrêtée à la station et s'y accoste. Batterie faible → rentre, recharge, reprend où il s'est arrêté.

Surveillez de près le premier tour de bordure. Arrêtez-le avec **Stop** sur le tableau de bord ou le bouton STOP du panneau s'il file vers la clôture : c'est soit un datum faux (carte décalée partout), soit un offset d'antenne faux (position qui balance dans les virages), soit un mauvais cap IMU (part de travers dès le départ).

La progression est persistée : une tonte interrompue reprend au même endroit. La page **Statistiques** journalise chaque session.`,
      },
    },
    {
      id: "mow-daily",
      title: { en: "Daily use: schedule, notifications, updates", fr: "Au quotidien : planning, notifications, mises à jour" },
      body: {
        en: `Once the first mow works, the GUI has everything for routine operation:

- **Schedule** — days, start time and area per schedule; optional rain hold and a soil-moisture gate.
- **Notifications** (Settings) — Telegram, Pushover, ntfy or a webhook: started, finished, docked, stuck, emergency, low battery.
- **Home Assistant / MQTT** — status, battery, coverage and a command topic, with the bundled broker.
- **Remote access** — an optional Tailscale sidecar to reach the GUI from anywhere, without opening a router port.
- **Settings → Updates** — checks for releases daily; one reviewed click updates the whole stack with backup and rollback, and reflashes the firmware through the ST-Link when a release needs it.

Winter: switch the mower off, leave the compute board on the SSD, and the map survives. On the first spring boot the datum, dock pose and calibrations are still there; only a dock re-calibration is worth redoing if the base moved.`,
        fr: `Une fois la première tonte réussie, l'interface a tout pour l'exploitation courante :

- **Planning** — jours, heure de départ et zone par planning ; pause pluie optionnelle et seuil d'humidité du sol.
- **Notifications** (Réglages) — Telegram, Pushover, ntfy ou webhook : démarré, terminé, en station, bloqué, urgence, batterie faible.
- **Home Assistant / MQTT** — état, batterie, couverture et un topic de commande, avec le broker intégré.
- **Accès à distance** — un sidecar Tailscale optionnel pour joindre l'interface de partout, sans ouvrir de port sur la box.
- **Réglages → Mises à jour** — vérifie les releases chaque jour ; un clic après revue met à jour toute la pile avec sauvegarde et retour arrière, et reflashe le firmware via le ST-Link quand une release l'exige.

Hiver : éteignez la tondeuse, laissez la carte de calcul sur son SSD, la carte survit. Au premier démarrage du printemps, datum, position de station et calibrations sont toujours là ; seule une recalibration de station vaut le coup si la base a bougé.`,
      },
    },
  ],
});
