MOWGLI_MANUAL.chapters.push({
  id: "firmware",
  icon: "💾",
  title: { en: "Mainboard firmware", fr: "Firmware de la carte mère" },
  media: { type: "img", src: "img/j9-stlink.svg", alt: { en: "ST-Link to J9 wiring", fr: "Câblage ST-Link vers J9" } },
  steps: [
    {
      id: "fw-how",
      title: { en: "How flashing works in MowgliNext", fr: "Comment se passe le flash dans MowgliNext" },
      body: {
        en: `Unlike the original Mowgli, you do not compile anything and you do not need a Windows PC:

1. An **ST-Link V2** dongle is wired to the mainboard's **J9** SWD header and plugged into a **USB port of the compute board**. It stays there permanently.
2. The web GUI's onboarding wizard has a **Flash firmware** step. It downloads the prebuilt binary for your board from the latest GitHub release, verifies its SHA-256, flashes it with OpenOCD (\`program … verify reset exit\`), then reads the board's version handshake to confirm.
3. Later releases are flashed the same way from **Settings → Updates**. Tuning values (wheel gains, wheel base, safety limits) are pushed over USB at every connect, so a tuning change **never** needs a reflash.

There is an *expert* option to build from source (a branch + rendered \`board.h\`) — you will not need it.

> [!NOTE] Panel firmware
> The GUI asks for a panel type only to pick the right prebuilt binary. It never flashes the panel board itself.`,
        fr: `Contrairement au Mowgli d'origine, vous ne compilez rien et vous n'avez pas besoin d'un PC Windows :

1. Un dongle **ST-Link V2** est câblé sur le connecteur SWD **J9** de la carte mère et branché sur un **port USB de la carte de calcul**. Il y reste en permanence.
2. L'assistant de démarrage de l'interface web a une étape **Flasher le firmware**. Elle télécharge le binaire précompilé pour votre carte depuis la dernière release GitHub, vérifie son SHA-256, le flashe avec OpenOCD (\`program … verify reset exit\`), puis lit la poignée de main de version de la carte pour confirmer.
3. Les releases suivantes se flashent de la même façon depuis **Réglages → Mises à jour**. Les valeurs de réglage (gains des roues, voie, limites de sécurité) sont envoyées par USB à chaque connexion, donc un changement de réglage ne demande **jamais** de reflash.

Il existe une option *expert* pour compiler depuis les sources (une branche + un \`board.h\` généré) — vous n'en aurez pas besoin.

> [!NOTE] Firmware du panneau
> L'interface demande le type de panneau uniquement pour choisir le bon binaire précompilé. Elle ne flashe jamais la carte panneau elle-même.`,
      },
    },
    {
      id: "fw-stlink",
      title: { en: "Wire the ST-Link to J9", fr: "Câbler le ST-Link sur J9" },
      media: {
        type: "img",
        src: "img/stlink-500b.jpg",
        alt: { en: "ST-Link V2 dongle wired to a YardForce 500B mainboard", fr: "Dongle ST-Link V2 câblé sur une carte mère YardForce 500B" },
        caption: { en: "ST-Link V2 on a 500B. Match the labels, not the positions.", fr: "ST-Link V2 sur une 500B. Suivez les étiquettes, pas les positions." },
        credit: { en: "Juditech3D (mowgli-docs, GPLv3)", fr: "Juditech3D (mowgli-docs, GPLv3)" },
      },
      body: {
        en: `Mower **off**, battery disconnected. Four Dupont wires between the dongle and the 4-pin **J9** header next to the STM32:

| ST-Link | J9 |
|---|---|
| GND | GND |
| SWCLK | SWCL |
| SWDIO | SWDA |
| 3.3V | 3V3 |

Clone dongles print their pinout on the case: **read the label**, positions vary between batches. The 3V3 wire is only a voltage sense here; connect it anyway.

Route the dongle so it can stay in the robot (tape it to the bracket) and plug it into a USB 2 port of the compute board. Then reconnect the battery and switch the mower on.

> [!TIP] Bench check
> From SSH, \`lsusb\` should now show \`0483:3748 STMicroelectronics ST-LINK/V2\` next to the mainboard's \`0483:5740\`.`,
        fr: `Tondeuse **éteinte**, batterie débranchée. Quatre fils Dupont entre le dongle et le connecteur 4 broches **J9** à côté du STM32 :

| ST-Link | J9 |
|---|---|
| GND | GND |
| SWCLK | SWCL |
| SWDIO | SWDA |
| 3.3V | 3V3 |

Les dongles clones impriment leur brochage sur le boîtier : **lisez l'étiquette**, les positions changent d'un lot à l'autre. Le fil 3V3 ne sert ici qu'à mesurer la tension ; branchez-le quand même.

Placez le dongle pour qu'il reste dans le robot (scotché au support) et branchez-le sur un port USB 2 de la carte de calcul. Rebranchez ensuite la batterie et allumez la tondeuse.

> [!TIP] Vérification
> En SSH, \`lsusb\` doit maintenant afficher \`0483:3748 STMicroelectronics ST-LINK/V2\` à côté du \`0483:5740\` de la carte mère.`,
      },
      parts: [
        { qty: 1, name: { en: "ST-Link V2 dongle", fr: "Dongle ST-Link V2" } },
        { qty: 4, name: { en: "Dupont F-F wires", fr: "Fils Dupont F-F" } },
      ],
    },
    {
      id: "fw-flash",
      title: { en: "Flash from the GUI", fr: "Flasher depuis l'interface" },
      media: {
        type: "img",
        src: "../gui-walkthrough/screenshots/onboarding/06-flash-firmware.png",
        alt: { en: "Flash firmware step of the onboarding wizard", fr: "Étape Flasher le firmware de l'assistant" },
        caption: { en: "Onboarding → Firmware: prebuilt source, board and panel type, Flash.", fr: "Assistant → Firmware : source précompilée, type de carte et de panneau, Flasher." },
      },
      body: {
        en: `This step happens **after the software install** (chapter 9), inside the onboarding wizard — it is described here because it is the mainboard's step. Come back when the GUI is up.

1. Open \`http://<mower-ip>:4006\`. The wizard starts on first visit; the **Firmware** step is the third one.
2. Firmware source: **Prebuilt** (default). Board: **YardForce 500**, **YardForce 500B** or **YardForce NX100i** (the wizard pre-selects it from the model you chose). Panel: the one on your mower.
3. Click **Flash**. Watch the log: download → sha256 OK → OpenOCD \`** Programming Finished **\` → \`** Verified OK **\` → board reset → handshake shows the new protocol and firmware version.
4. The board chirps twice and the LED **D3** near the STM32 blinks. The panel may look different from stock (fewer LEDs lit) — normal.

> [!WARNING] After the flash
> The board re-enumerates over USB. On some boards this occasionally fails (EMI) and every firmware topic goes silent even though the GUI is fine. Chapter 13 has the two-line fix; a power cycle also works.`,
        fr: `Cette étape a lieu **après l'installation logicielle** (chapitre 9), dans l'assistant de démarrage — elle est décrite ici parce que c'est l'étape de la carte mère. Revenez-y quand l'interface est en ligne.

1. Ouvrez \`http://<ip-tondeuse>:4006\`. L'assistant démarre à la première visite ; l'étape **Firmware** est la troisième.
2. Source du firmware : **Précompilé** (défaut). Carte : **YardForce 500**, **YardForce 500B** ou **YardForce NX100i** (l'assistant la présélectionne d'après le modèle choisi). Panneau : celui de votre tondeuse.
3. Cliquez **Flasher**. Suivez le journal : téléchargement → sha256 OK → OpenOCD \`** Programming Finished **\` → \`** Verified OK **\` → reset de la carte → la poignée de main affiche les nouvelles versions de protocole et de firmware.
4. La carte émet deux bips et la LED **D3** près du STM32 clignote. Le panneau peut différer de l'origine (moins de voyants allumés) — normal.

> [!WARNING] Après le flash
> La carte se ré-énumère en USB. Sur certaines cartes cela échoue parfois (perturbations) et tous les topics firmware se taisent alors que l'interface va bien. Le chapitre 13 donne la correction en deux lignes ; un cycle d'alimentation fonctionne aussi.`,
      },
    },
    {
      id: "fw-500b",
      title: { en: "500B specifics", fr: "Particularités de la 500B" },
      when: { model: ["yf500b"] },
      body: {
        en: `The 500B mainboard carries an **STM32F401** and drives its blade motor over a different UART. Select **YardForce 500B** as the board in the Firmware step; the prebuilt \`Yardforce500B\` build handles the panel and the blade ESC.

Notes:

- There is no UART debug tap on the 500B build (it traces over SWO); the GUI's firmware log stream is what you use instead.
- The keypad and LEDs are driven by the firmware's panel support. If your panel misbehaves after the flash, report the panel marking (\`RM-ECOW-…\`) in an issue.
- Pepeuch's printed block was designed for this chassis and puts the antenna inside the shell at \`gps_x 0.295\`.`,
        fr: `La carte mère 500B embarque un **STM32F401** et pilote son moteur de lame sur un autre UART. Choisissez **YardForce 500B** comme carte à l'étape Firmware ; le build précompilé \`Yardforce500B\` gère le panneau et l'ESC de lame.

Notes :

- Pas de prise UART de debug sur le build 500B (il trace en SWO) ; utilisez le flux de journal firmware de l'interface à la place.
- Le clavier et les voyants sont pilotés par le support panneau du firmware. Si votre panneau se comporte mal après le flash, signalez son marquage (\`RM-ECOW-…\`) dans un ticket.
- Le bloc imprimé de Pepeuch a été conçu pour ce châssis et place l'antenne dans la coque à \`gps_x 0.295\`.`,
      },
    },
    {
      id: "fw-nx100i",
      title: { en: "NX100i specifics", fr: "Particularités de la NX100i" },
      when: { model: ["nx100i"] },
      body: {
        en: `The NX100i uses the **same GForce STM32F401 mainboard** as the 500B — same pinout, same PAC5210 drive and PAC5223 blade controllers and UART protocols — with its own stop-button wiring. Select **YardForce NX100i** as the model in the wizard; it maps to the \`YardforceNX100i\` firmware build (board STM32F401, panel "500B classic"). Mowgli-Next drove this chassis on the bench with no firmware change to the 500B code; the blade motor has **not yet** been tested.

**Connectors differ from the 500B:**

- **SWD is J12**, the 4-pin header near the top-left corner mounting screw (one pin labelled GND) — not J9. The leads are nudge-sensitive: if OpenOCD reports \`init mode failed\`, reseat them.
- **The USB link is the USB port in the battery compartment** (the factory update port). Use a USB-A to USB-A *data* cable. Under stock firmware it enumerates as a \`RobotMower_Server\` HID device, under Mowgli as \`Mowgli\`.

**Main-switch rule.** The push-in/out main switch is the battery feed for the whole board, but the USB cable back-powers the STM32 logic with 5 V. So with the switch **off**, SWD and the Mowgli USB link look healthy while the motor controllers and the hood are unpowered (no odometry, blade error counter climbing, battery reading ~0.6 V). Stock firmware blocks SWD and USB while the switch is on; Mowgli does not.

1. **Back up first** — ST-Link on J12, switch off, OpenOCD \`dump_image <file> 0x08000000 0x40000\` (256 KB; stock identifies as \`GRM V4.03_2021 / RobotMower_Server\`, bootloader \`RM Boot_V1.00\`).
2. **Flash with the switch OFF.**
3. **Test motors with the switch ON.** Expect a brief brownout / MCU reset at switch-on.

**Hood, stop and lift.** The hood carries the big stop button, the LCD + 18-button panel and the ultrasonic collision sensors on one cable. The stop button has two contacts: PC0 driven high when pressed, and PC8 normally closed to ground that opens when pressed — the NX100i build pulls PC8 up, so an **unplugged hood reads as "stop pressed"** (fail-safe). The lift sensors (PD0 / PD1) are on the chassis side and read "lifted" until the hood is properly clipped onto its posts. The bumper hall-stop lines (PD2 / PD3) are unconnected on this chassis; collision detection from the hood ultrasonics is future work.

> [!NOTE] Blank panel
> The NX100i panel protocol differs from the 500B keypad. Under Mowgli the LCD lights up but stays blank; you control the mower from the GUI. A panel driver is future work.

**External IMU.** J18 pins 5 (SCL, PB3) and 6 (SDA, PB4) are verified on this chassis; power the module from the neighbouring J17 (5 V / GND) because the hood harness fills J18. The 5 V there is a switched accessory rail that only comes up at the end of the firmware's init, so the \`YardforceNX100i\` build probes the IMU **after** the motor-controller init — an MPU-6050 (GY-521) was detected and streamed at ~90 Hz on the bench. Mount it chip-side up with the X arrow forward; a chip-side-down mount needs the \`YardforceNX100i_MPU6050_Roll180\` (arrow forward) or \`_Pitch180\` (arrow backward) build, otherwise the firmware yaw loop steers the wrong way and the mower cannot drive straight (field-confirmed 2026-10-15). Pin 8 of J18 still carries ~29 V: confirm pin 1 before plugging anything in.

> [!DANGER] Still unverified
> The tilt-switch polarity on PA8 has not been verified on this chassis.`,
        fr: `La NX100i utilise la **même carte mère GForce STM32F401** que la 500B — même brochage, mêmes contrôleurs PAC5210 (traction) et PAC5223 (lame) et mêmes protocoles UART — avec son propre câblage du bouton stop. Choisissez **YardForce NX100i** comme modèle dans l'assistant ; il correspond au build firmware \`YardforceNX100i\` (carte STM32F401, panneau « 500B classic »). Mowgli-Next a fait rouler ce châssis sur banc sans modifier le code firmware de la 500B ; le moteur de lame n'a **pas encore** été testé.

**Les connecteurs diffèrent de la 500B :**

- **Le SWD est J12**, le connecteur 4 broches près de la vis de fixation en haut à gauche (une broche marquée GND) — pas J9. Les fils sont sensibles aux mouvements : si OpenOCD affiche \`init mode failed\`, rebranchez-les.
- **La liaison USB est le port USB du compartiment batterie** (port de mise à jour usine). Utilisez un câble USB-A vers USB-A *de données*. Sous firmware d'origine il s'énumère comme périphérique HID \`RobotMower_Server\`, sous Mowgli comme \`Mowgli\`.

**Règle de l'interrupteur principal.** L'interrupteur principal (bouton poussoir) alimente toute la carte depuis la batterie, mais le câble USB alimente aussi la logique du STM32 en 5 V. Interrupteur **éteint**, le SWD et la liaison USB Mowgli semblent donc sains alors que les contrôleurs moteurs et le capot ne sont pas alimentés (pas d'odométrie, compteur d'erreurs lame qui monte, batterie lue à ~0,6 V). Le firmware d'origine bloque le SWD et l'USB quand l'interrupteur est allumé ; Mowgli non.

1. **Sauvegardez d'abord** — ST-Link sur J12, interrupteur éteint, OpenOCD \`dump_image <fichier> 0x08000000 0x40000\` (256 Ko ; l'origine s'identifie comme \`GRM V4.03_2021 / RobotMower_Server\`, bootloader \`RM Boot_V1.00\`).
2. **Flashez interrupteur ÉTEINT.**
3. **Testez les moteurs interrupteur ALLUMÉ.** Attendez-vous à un bref brownout / reset du MCU à l'allumage.

**Capot, stop et levage.** Le capot porte le gros bouton stop, le panneau LCD à 18 boutons et les capteurs ultrasons de collision, sur un seul câble. Le bouton stop a deux contacts : PC0 mis à l'état haut quand on appuie, et PC8 normalement fermé à la masse qui s'ouvre à l'appui — le build NX100i tire PC8 vers le haut, donc un **capot débranché est lu comme « stop appuyé »** (sécurité positive). Les capteurs de levage (PD0 / PD1) sont côté châssis et lisent « soulevée » tant que le capot n'est pas bien clipsé sur ses plots. Les lignes hall-stop du pare-chocs (PD2 / PD3) ne sont pas connectées sur ce châssis ; la détection de collision par les ultrasons du capot est un travail futur.

> [!NOTE] Panneau vide
> Le protocole du panneau NX100i diffère du clavier 500B. Sous Mowgli, le LCD s'allume mais reste vide ; vous pilotez la tondeuse depuis l'interface. Un pilote de panneau est un travail futur.

**IMU externe.** Les broches 5 (SCL, PB3) et 6 (SDA, PB4) de J18 sont vérifiées sur ce châssis ; alimentez le module depuis le connecteur voisin J17 (5 V / GND), le faisceau du capot occupant J18. Ce 5 V est un rail accessoire commuté qui n'apparaît qu'à la fin de l'initialisation du firmware ; le build \`YardforceNX100i\` sonde donc l'IMU **après** l'initialisation des contrôleurs moteurs — un MPU-6050 (GY-521) a été détecté et diffusé à ~90 Hz sur banc. Montez-le puce vers le haut, flèche X vers l'avant ; un montage puce vers le bas demande le build \`YardforceNX100i_MPU6050_Roll180\` (flèche vers l'avant) ou \`_Pitch180\` (flèche vers l'arrière), sinon la boucle de lacet du firmware corrige à l'envers et la tondeuse ne roule pas droit (confirmé sur le terrain le 2026-10-15). La broche 8 de J18 porte toujours ~29 V : confirmez la broche 1 avant de brancher quoi que ce soit.

> [!DANGER] Toujours non vérifié
> La polarité du capteur d'inclinaison sur PA8 n'a pas été vérifiée sur ce châssis.`,
      },
    },
    {
      id: "fw-other",
      title: { en: "SA650, 900 ECO and other boards", fr: "SA650, 900 ECO et autres cartes" },
      when: { model: ["other"] },
      body: {
        en: `The SA650 and 900 ECO share the Classic 500's **STM32F103** mainboard family: flash the **YardForce 500** build and pick the matching model preset in the wizard (chassis size and encoder resolution differ — 1050 ticks/m instead of 300).

The **LUV1000RI** has a GUI preset but **no firmware build**: its MCU and blade-UART wiring are not documented, and a guessed pinout can brick the board. Do not flash it with a 500 image. Help mapping it is welcome on GitHub.

For a **custom robot** on a supported board, choose *Custom Robot* and enter every dimension by hand.`,
        fr: `La SA650 et la 900 ECO partagent la famille de cartes mères **STM32F103** de la Classic 500 : flashez le build **YardForce 500** et choisissez le préréglage de modèle correspondant dans l'assistant (dimensions du châssis et résolution encodeur diffèrent — 1050 ticks/m au lieu de 300).

La **LUV1000RI** a un préréglage dans l'interface mais **pas de build firmware** : son MCU et le câblage UART de lame ne sont pas documentés, et un brochage deviné peut bloquer la carte. Ne la flashez pas avec une image 500. Toute aide pour la cartographier est bienvenue sur GitHub.

Pour un **robot custom** sur une carte prise en charge, choisissez *Robot custom* et saisissez chaque dimension à la main.`,
      },
    },
  ],
});
