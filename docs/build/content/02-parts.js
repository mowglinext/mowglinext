MOWGLI_MANUAL.chapters.push({
  id: "parts",
  icon: "🧰",
  title: { en: "Parts list", fr: "Liste du matériel" },
  media: { type: "img", src: "img/overview.svg", alt: { en: "System overview", fr: "Vue d'ensemble du système" } },
  steps: [
    {
      id: "parts-compute",
      title: { en: "Compute board and storage", fr: "Carte de calcul et stockage" },
      body: {
        en: `The compute board runs the whole navigation stack in Docker. Any 64-bit Linux board with USB, a 40-pin header and Docker support works; the installer supports \`arm64\` and \`amd64\`.

| Board | Verdict |
|---|---|
| **Raspberry Pi 5** (4 GB or more) with an **NVMe SSD** (M.2 HAT) | **Recommended.** Fast, well supported, plenty of headroom for LiDAR. |
| **Orange Pi 5B** or another RK3588 / RK3566 board with eMMC or NVMe | **Recommended** alternative — same class of performance, on-board storage. |
| Raspberry Pi 4 (4 GB) | Known to work, but at ~75 % CPU with LiDAR and a real risk of the SD card wearing out. Only if you already own one. |

> [!WARNING] Avoid running from a micro-SD card
> The stack writes logs, maps, saved graphs and recordings continuously. SD cards fail after a season of that, and a full card silently freezes the robot (it happened to the maintainers). Boot from **NVMe, USB SSD or eMMC**. If you must start on SD, use a 32 GB "high endurance" card and move to SSD later.

Minimum: 4 cores, 4 GB RAM, 16 GB storage. Reserve a power budget of **5 V / 5 A** for a Pi 5 (3 A for a Pi 4).`,
        fr: `La carte de calcul fait tourner toute la pile de navigation dans Docker. N'importe quelle carte Linux 64 bits avec USB, connecteur 40 broches et Docker convient ; l'installateur gère \`arm64\` et \`amd64\`.

| Carte | Verdict |
|---|---|
| **Raspberry Pi 5** (4 Go ou plus) avec un **SSD NVMe** (HAT M.2) | **Recommandé.** Rapide, bien supporté, de la marge pour le LiDAR. |
| **Orange Pi 5B** ou une autre carte RK3588 / RK3566 avec eMMC ou NVMe | **Recommandé** en alternative — même classe de performance, stockage embarqué. |
| Raspberry Pi 4 (4 Go) | Fonctionne, mais à ~75 % de CPU avec LiDAR et avec un vrai risque d'usure de la carte SD. Seulement si vous en avez déjà un. |

> [!WARNING] Évitez de démarrer sur micro-SD
> La pile écrit en continu des logs, cartes, graphes sauvegardés et enregistrements. Une carte SD lâche après une saison de ce régime, et une carte pleine fige le robot en silence (c'est arrivé aux mainteneurs). Démarrez sur **NVMe, SSD USB ou eMMC**. Si vous devez commencer sur SD, prenez une carte 32 Go « haute endurance » et passez au SSD ensuite.

Minimum : 4 cœurs, 4 Go de RAM, 16 Go de stockage. Prévoyez un budget d'alimentation de **5 V / 5 A** pour un Pi 5 (3 A pour un Pi 4).`,
      },
      parts: [
        { qty: 1, name: { en: "Raspberry Pi 5 (4/8 GB) or Orange Pi 5B", fr: "Raspberry Pi 5 (4/8 Go) ou Orange Pi 5B" } },
        { qty: 1, name: { en: "NVMe SSD + M.2 HAT (Pi 5), or USB SSD / eMMC module", fr: "SSD NVMe + HAT M.2 (Pi 5), ou SSD USB / module eMMC" } },
        { qty: 1, name: { en: "Active cooler / heatsink for the board", fr: "Ventilateur / dissipateur pour la carte" } },
      ],
    },
    {
      id: "parts-power",
      title: { en: "Power converter and cables", fr: "Convertisseur d'alimentation et câbles" },
      body: {
        en: `The mower battery is ~29 V. A DC-DC buck converter brings it down to 5.1 V for the compute board.

| Part | Notes |
|---|---|
| DC-DC buck converter, **5 A** (XL4015 / XL4016) or 3 A (LM2596) | Adjustable output; 5 A for a Pi 5, 3 A minimum for a Pi 4. Pick one with a screw terminal or solder pads. |
| 470–1000 µF / 16 V electrolytic capacitor | Optional, across the 5 V output if the board browns out when the motors start. |
| USB **A-to-A** cable, 30–50 cm | Pi ↔ mainboard J14. Shielded if you can. |
| Silicone wire 0.5–0.75 mm² (20–18 AWG), red + black | Battery tap → DC-DC → board. |
| Dupont / JST-XH / HY2.0 leads, 20 cm | IMU on J18, UART links to the Pi header. |
| Heat-shrink, cable ties, double-sided foam tape | Keep everything away from the blade and the wheels. |

> [!TIP] Cheap wins
> The mainboard's red **J18** header accepts standard 2.0 mm pitch (HY2.0 / JST-PH-like) housings. The old Mowgli guide's 4P–9P HY2.0 leads still fit and save soldering on the mainboard.`,
        fr: `La batterie de la tondeuse fait ~29 V. Un convertisseur abaisseur DC-DC la ramène à 5,1 V pour la carte de calcul.

| Pièce | Remarques |
|---|---|
| Convertisseur DC-DC abaisseur, **5 A** (XL4015 / XL4016) ou 3 A (LM2596) | Sortie réglable ; 5 A pour un Pi 5, 3 A minimum pour un Pi 4. Avec bornier à vis ou pastilles à souder. |
| Condensateur électrolytique 470–1000 µF / 16 V | Optionnel, sur la sortie 5 V si la carte redémarre au démarrage des moteurs. |
| Câble USB **A vers A**, 30–50 cm | Pi ↔ carte mère J14. Blindé si possible. |
| Fil silicone 0,5–0,75 mm² (20–18 AWG), rouge + noir | Prise batterie → DC-DC → carte. |
| Fils Dupont / JST-XH / HY2.0, 20 cm | IMU sur J18, liaisons UART vers le connecteur du Pi. |
| Gaine thermo, colliers, mousse double-face | Tout doit rester loin de la lame et des roues. |

> [!TIP] Bon plan
> Le connecteur rouge **J18** de la carte mère accepte les boîtiers au pas de 2,0 mm (HY2.0 / type JST-PH). Les fils HY2.0 4P–9P du guide Mowgli d'origine conviennent toujours et évitent de souder sur la carte mère.`,
      },
      parts: [
        { qty: 1, name: { en: "DC-DC buck converter 29 V → 5.1 V, 5 A", fr: "Convertisseur DC-DC 29 V → 5,1 V, 5 A" } },
        { qty: 1, name: { en: "USB A-to-A cable", fr: "Câble USB A vers A" } },
        { qty: 1, name: { en: "Assorted wire, Dupont / HY2.0 leads, heat-shrink", fr: "Fil, fils Dupont / HY2.0, gaine thermo" } },
      ],
    },
    {
      id: "parts-gnss",
      title: { en: "RTK GNSS receiver and antenna", fr: "Récepteur GNSS RTK et antenne" },
      body: {
        en: `This is the sensor that makes the whole project work: a multi-band RTK receiver gives 1–3 cm positioning when it receives corrections from a base station.

| Receiver | Notes |
|---|---|
| **u-blox ZED-F9P** on a simpleRTK2B (ArduSimple) or equivalent board | The reference receiver. USB or UART. Widely documented. |
| **Unicore UM980 / UM981 / UM982** (e.g. WitMotion WTRTK-982) | Cheaper, fully supported through the Universal GNSS driver. UM982 = dual antenna heading. |
| Generic NMEA receivers | Supported by the driver, but without RTK you will not get usable coverage. |

| Antenna | Notes |
|---|---|
| Multi-band helical or patch (BT-560, BT-603, HA-901A, ArduSimple survey…) | Must be **L1/L2 (or L1/L5) multi-band** — a single-band antenna never reaches RTK Fixed. |
| SMA cable, 30–50 cm | Match the connector on your receiver board (SMA vs u.FL). |
| Ground plane | A 10 cm metal disc under a patch antenna helps a lot. |

**Corrections (NTRIP).** You need a base station within ~30 km. In France the [Centipède RTK](https://map.centipede-rtk.org/) network is free; elsewhere try [RTK2GO](http://www.rtk2go.com/), a regional service, or your own base. Have the caster host, port, mount point and credentials ready for the installer.`,
        fr: `C'est le capteur qui fait marcher tout le projet : un récepteur RTK multi-bandes donne une position à 1–3 cm quand il reçoit les corrections d'une base.

| Récepteur | Remarques |
|---|---|
| **u-blox ZED-F9P** sur une simpleRTK2B (ArduSimple) ou carte équivalente | Le récepteur de référence. USB ou UART. Très documenté. |
| **Unicore UM980 / UM981 / UM982** (ex. WitMotion WTRTK-982) | Moins cher, entièrement pris en charge par le pilote Universal GNSS. UM982 = cap par double antenne. |
| Récepteurs NMEA génériques | Gérés par le pilote, mais sans RTK vous n'aurez pas de tonte exploitable. |

| Antenne | Remarques |
|---|---|
| Hélicoïdale ou patch multi-bandes (BT-560, BT-603, HA-901A, survey ArduSimple…) | Doit être **multi-bandes L1/L2 (ou L1/L5)** — une antenne mono-bande n'atteint jamais le RTK Fixed. |
| Câble SMA, 30–50 cm | Selon le connecteur de votre carte (SMA ou u.FL). |
| Plan de masse | Un disque métallique de 10 cm sous une antenne patch aide beaucoup. |

**Corrections (NTRIP).** Il vous faut une base à moins de ~30 km. En France le réseau [Centipède RTK](https://map.centipede-rtk.org/) est gratuit ; ailleurs, [RTK2GO](http://www.rtk2go.com/), un service régional, ou votre propre base. Ayez sous la main l'hôte du caster, le port, le point de montage et les identifiants pour l'installateur.`,
      },
      parts: [
        { qty: 1, name: { en: "RTK receiver: ZED-F9P board or UM980/982 board", fr: "Récepteur RTK : carte ZED-F9P ou UM980/982" } },
        { qty: 1, name: { en: "Multi-band GNSS antenna + SMA cable", fr: "Antenne GNSS multi-bandes + câble SMA" } },
        { qty: 1, name: { en: "NTRIP account / mount point near you", fr: "Compte NTRIP / point de montage proche" } },
      ],
    },
    {
      id: "parts-imu",
      title: { en: "IMU", fr: "IMU" },
      body: {
        en: `The IMU (gyroscope + accelerometer, optionally magnetometer) plugs into the mainboard's J18 header over I²C; the firmware auto-detects it at boot.

| IMU | Notes |
|---|---|
| **WitMotion WT901** (I²C variant) | Recommended: gyro + accel + magnetometer in one module, 5 V tolerant, robust. |
| MPU6050 breakout (GY-521) | Cheapest, gyro + accel only. Works well. |
| LSM6DS* breakout, ICM-45686 | Also auto-detected. |
| LIS3MDL magnetometer | Optional add-on when the accel/gyro chip has no compass. |

A magnetometer is **optional**: heading is fused from the gyro, wheel odometry and the GNSS course over ground, and the GUI only offers the compass calibration if you enable it.`,
        fr: `L'IMU (gyroscope + accéléromètre, magnétomètre optionnel) se branche sur le connecteur J18 de la carte mère en I²C ; le firmware la détecte automatiquement au démarrage.

| IMU | Remarques |
|---|---|
| **WitMotion WT901** (version I²C) | Recommandée : gyro + accéléro + magnétomètre dans un seul module, tolérante au 5 V, robuste. |
| Module MPU6050 (GY-521) | Le moins cher, gyro + accéléro seulement. Fonctionne bien. |
| Module LSM6DS*, ICM-45686 | Détectés aussi. |
| Magnétomètre LIS3MDL | Complément optionnel quand la puce gyro/accéléro n'a pas de boussole. |

Le magnétomètre est **optionnel** : le cap est fusionné à partir du gyro, de l'odométrie des roues et de la route GNSS, et l'interface ne propose la calibration boussole que si vous l'activez.`,
      },
      parts: [{ qty: 1, name: { en: "WT901 (I²C) or MPU6050 module + 4-wire lead", fr: "Module WT901 (I²C) ou MPU6050 + fil 4 conducteurs" } }],
    },
    {
      id: "parts-lidar",
      title: { en: "LiDAR (optional)", fr: "LiDAR (optionnel)" },
      body: {
        en: `A 2D LiDAR lets the robot **avoid obstacles** (the coverage path is deviated laterally around them and the collision monitor stops it before contact) and, optionally, learn a map of fixed features that anchors the position if RTK drops out for a while.

| LiDAR | Notes |
|---|---|
| **LDRobot LD19** (also LD06, LD14) | Reference unit, ~80 €. UART 230400 baud, 5 V. Ships with a small adapter board; keep it. |
| Slamtec RPLiDAR A1 / A2 / C1 | Supported via the \`rplidar\` driver. |
| LDRobot STL27L | Experimental driver — not field-tested. |

Without LiDAR the robot mows perfectly well inside the recorded boundary, but it will bump into the garden chair you forgot on the lawn. Most builders add it in a second phase; the installer and the GUI let you turn it on later.`,
        fr: `Un LiDAR 2D permet au robot **d'éviter les obstacles** (la trajectoire de tonte est déviée latéralement autour d'eux et le moniteur de collision l'arrête avant contact) et, en option, d'apprendre une carte des éléments fixes qui ancre la position si le RTK décroche un moment.

| LiDAR | Remarques |
|---|---|
| **LDRobot LD19** (aussi LD06, LD14) | Le modèle de référence, ~80 €. UART 230400 bauds, 5 V. Livré avec une petite carte adaptateur ; gardez-la. |
| Slamtec RPLiDAR A1 / A2 / C1 | Pris en charge par le pilote \`rplidar\`. |
| LDRobot STL27L | Pilote expérimental — non testé sur le terrain. |

Sans LiDAR, le robot tond très bien à l'intérieur de la zone enregistrée, mais il percutera la chaise de jardin oubliée sur la pelouse. La plupart des monteurs l'ajoutent dans un second temps ; l'installateur et l'interface permettent de l'activer plus tard.`,
      },
      parts: [{ qty: 1, name: { en: "LDRobot LD19 with its adapter board (optional)", fr: "LDRobot LD19 avec sa carte adaptateur (optionnel)" } }],
    },
    {
      id: "parts-tools",
      title: { en: "Programmer, tools and software", fr: "Programmateur, outils et logiciels" },
      body: {
        en: `| Item | Why |
|---|---|
| **ST-Link V2** USB dongle (clones are fine) | Flashes the mainboard. Stays in the robot, plugged into the Pi: the GUI reflashes future firmware through it. |
| 4 female-female Dupont wires | ST-Link → J9. |
| Multimeter | Set the DC-DC to 5.1 V **before** connecting the board. Non-negotiable. |
| Torx T20 / T25 screwdriver, Phillips | Opening the YardForce shell. |
| Soldering iron or crimp tool | A few joints. |
| Micro-SD / USB adapter, or the M.2 HAT | To flash the OS image from your computer. |

Software on your computer:

- [Raspberry Pi Imager](https://www.raspberrypi.com/software/) (or balenaEtcher for non-Pi boards) to write the OS image;
- an SSH client: the built-in terminal on macOS / Linux, or Windows Terminal / PowerShell (\`ssh\` is built in), MobaXterm if you prefer a GUI;
- a phone or laptop on the same Wi-Fi as the mower for the web interface.

> [!NOTE] Wi-Fi coverage
> The robot streams status to the GUI and pulls RTK corrections over Wi-Fi. Make sure your network **covers the whole lawn** — a mesh node or an outdoor access point near the dock is a common upgrade. The mower keeps mowing through short dropouts, but NTRIP corrections need the link.`,
        fr: `| Élément | Pourquoi |
|---|---|
| Dongle USB **ST-Link V2** (les clones conviennent) | Flashe la carte mère. Il reste dans le robot, branché au Pi : l'interface reflashera les futurs firmwares par lui. |
| 4 fils Dupont femelle-femelle | ST-Link → J9. |
| Multimètre | Régler le DC-DC à 5,1 V **avant** de brancher la carte. Non négociable. |
| Tournevis Torx T20 / T25, cruciforme | Ouvrir la coque YardForce. |
| Fer à souder ou pince à sertir | Quelques connexions. |
| Adaptateur micro-SD / USB, ou le HAT M.2 | Pour écrire l'image système depuis votre ordinateur. |

Logiciels sur votre ordinateur :

- [Raspberry Pi Imager](https://www.raspberrypi.com/software/) (ou balenaEtcher pour les cartes non-Pi) pour écrire l'image système ;
- un client SSH : le terminal intégré sur macOS / Linux, ou Windows Terminal / PowerShell (\`ssh\` est intégré), MobaXterm si vous préférez une interface ;
- un téléphone ou un portable sur le même Wi-Fi que la tondeuse pour l'interface web.

> [!NOTE] Couverture Wi-Fi
> Le robot envoie son état à l'interface et reçoit les corrections RTK par Wi-Fi. Assurez-vous que votre réseau **couvre toute la pelouse** — un nœud mesh ou un point d'accès extérieur près de la station est une amélioration classique. La tondeuse continue pendant les coupures courtes, mais les corrections NTRIP ont besoin du lien.`,
      },
      parts: [
        { qty: 1, name: { en: "ST-Link V2 dongle + 4 Dupont F-F wires", fr: "Dongle ST-Link V2 + 4 fils Dupont F-F" } },
        { qty: 1, name: { en: "Multimeter, Torx screwdrivers, soldering iron", fr: "Multimètre, tournevis Torx, fer à souder" } },
      ],
    },
    {
      id: "parts-3d",
      title: { en: "3D-printed brackets", fr: "Supports imprimés en 3D" },
      media: {
        type: "img",
        src: "img/print-pi-bracket.jpg",
        alt: { en: "3D-printed Raspberry Pi bracket inside a YardForce 500B", fr: "Support Raspberry Pi imprimé en 3D dans une YardForce 500B" },
        caption: { en: "Modular Pi + GNSS bracket in a 500B.", fr: "Support modulaire Pi + GNSS dans une 500B." },
        credit: { en: "Juditech3D (mowgli-docs, GPLv3)", fr: "Juditech3D (mowgli-docs, GPLv3)" },
      },
      body: {
        en: `The community designed brackets that hold the compute board, the DC-DC and the GNSS board inside the YardForce shell, and antenna mounts for the top cover. They are free on MakerWorld:

- **Juditech3D** — modular support V2 (Pi 3/4/5, F9P / UM980 boards, LM2596 / XL4015 / XL4016 DC-DC, blank module), weighted wheels, charging-base riser: [MakerWorld @Juditech3D](https://makerworld.com/en/@juditech3d)
- **Pepeuch** — all-in-one block Pi + GNSS + DC-DC + F9P for the 500B with the antenna inside the shell (\`gps_x 0.295\`), and the SA-series motor adapter for a Classic 500B chassis: [MakerWorld @Pepeuch](https://makerworld.com/fr/@user_3228887730)
- STL collection: [github.com/Mowglifrenchtouch/mowgli-3d-parts](https://github.com/Mowglifrenchtouch/mowgli-3d-parts)

Print settings for structural parts: **PETG, ASA or ABS** (the shell gets hot in the sun and damp at night), 30–50 % infill, 3–6 walls. PLA deforms in a parked mower in July.

No printer? The French community prints on request — ask on Telegram.`,
        fr: `La communauté a conçu des supports pour la carte de calcul, le DC-DC et la carte GNSS à l'intérieur de la coque YardForce, et des supports d'antenne pour le capot. Ils sont gratuits sur MakerWorld :

- **Juditech3D** — support modulaire V2 (Pi 3/4/5, cartes F9P / UM980, DC-DC LM2596 / XL4015 / XL4016, module vierge), roues lestables, rehausse de base de charge : [MakerWorld @Juditech3D](https://makerworld.com/en/@juditech3d)
- **Pepeuch** — bloc tout-en-un Pi + GNSS + DC-DC + F9P pour la 500B avec l'antenne dans la coque (\`gps_x 0.295\`), et l'adaptateur moteurs série SA pour châssis Classic 500B : [MakerWorld @Pepeuch](https://makerworld.com/fr/@user_3228887730)
- Collection STL : [github.com/Mowglifrenchtouch/mowgli-3d-parts](https://github.com/Mowglifrenchtouch/mowgli-3d-parts)

Paramètres d'impression pour les pièces structurelles : **PETG, ASA ou ABS** (la coque chauffe au soleil et prend l'humidité la nuit), remplissage 30–50 %, 3–6 parois. Le PLA se déforme dans une tondeuse garée en juillet.

Pas d'imprimante ? La communauté française imprime sur demande — demandez sur Telegram.`,
      },
      parts: [{ qty: 1, name: { en: "Bracket set for your board + GNSS + DC-DC, antenna mount", fr: "Jeu de supports carte + GNSS + DC-DC, support d'antenne" } }],
    },
  ],
});
