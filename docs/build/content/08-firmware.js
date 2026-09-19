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
2. Firmware source: **Prebuilt** (default). Board: **YardForce 500** or **YardForce 500B** (the wizard pre-selects it from the model you chose). Panel: the one on your mower.
3. Click **Flash**. Watch the log: download → sha256 OK → OpenOCD \`** Programming Finished **\` → \`** Verified OK **\` → board reset → handshake shows the new protocol and firmware version.
4. The board chirps twice and the LED **D3** near the STM32 blinks. The panel may look different from stock (fewer LEDs lit) — normal.

> [!WARNING] After the flash
> The board re-enumerates over USB. On some boards this occasionally fails (EMI) and every firmware topic goes silent even though the GUI is fine. Chapter 13 has the two-line fix; a power cycle also works.`,
        fr: `Cette étape a lieu **après l'installation logicielle** (chapitre 9), dans l'assistant de démarrage — elle est décrite ici parce que c'est l'étape de la carte mère. Revenez-y quand l'interface est en ligne.

1. Ouvrez \`http://<ip-tondeuse>:4006\`. L'assistant démarre à la première visite ; l'étape **Firmware** est la troisième.
2. Source du firmware : **Précompilé** (défaut). Carte : **YardForce 500** ou **YardForce 500B** (l'assistant la présélectionne d'après le modèle choisi). Panneau : celui de votre tondeuse.
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
