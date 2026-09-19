MOWGLI_MANUAL.chapters.push({
  id: "gnss",
  icon: "🛰️",
  title: { en: "RTK GNSS", fr: "GNSS RTK" },
  media: { type: "img", src: "img/gnss-uart.svg", alt: { en: "Receiver wiring options", fr: "Options de câblage du récepteur" } },
  steps: [
    {
      id: "gnss-antenna",
      title: { en: "Mount the antenna", fr: "Fixer l'antenne" },
      body: {
        en: `The antenna decides whether you ever see **RTK Fixed**. Rules:

1. **Highest point, clear sky.** On the top cover, forward of the blade, with nothing metallic above it. The community brackets put it on the flat front area; Pepeuch's 500B block puts it just under the plastic shell, which also works (plastic is transparent to GNSS).
2. **Ground plane** under a patch antenna: a 10 cm metal disc, or the metal bracket it ships with. Helical antennas do not need one.
3. **Centred left-right** (\`gps_y = 0\`) if you can — it removes one measurement from the offset step.
4. **Short SMA cable**, no sharp bends, connector hand-tight. A loose SMA is the classic "never reaches Fixed" cause.
5. Keep the cable away from the wheel-motor and blade leads.

Measure and note, from the **rear axle centre** to the **antenna's centre**: forward distance (\`gps_x\`), lateral offset (\`gps_y\`, left positive) and height above the ground (\`gps_z\`). The 500 preset is 0.30 / 0.00 / 0.20 m.`,
        fr: `L'antenne décide si vous verrez un jour **RTK Fixed**. Règles :

1. **Point le plus haut, ciel dégagé.** Sur le capot, en avant de la lame, sans rien de métallique au-dessus. Les supports communautaires la mettent sur la zone plane avant ; le bloc 500B de Pepeuch la place juste sous la coque plastique, ce qui fonctionne aussi (le plastique est transparent au GNSS).
2. **Plan de masse** sous une antenne patch : un disque métallique de 10 cm, ou l'équerre métallique fournie. Les antennes hélicoïdales n'en ont pas besoin.
3. **Centrée gauche-droite** (\`gps_y = 0\`) si possible — cela supprime une mesure à l'étape des offsets.
4. **Câble SMA court**, sans pli serré, connecteur serré à la main. Un SMA desserré est la cause classique du « jamais Fixed ».
5. Éloignez le câble des fils des moteurs de roues et de lame.

Mesurez et notez, du **centre de l'essieu arrière** au **centre de l'antenne** : distance vers l'avant (\`gps_x\`), décalage latéral (\`gps_y\`, positif à gauche) et hauteur au-dessus du sol (\`gps_z\`). Le préréglage 500 vaut 0,30 / 0,00 / 0,20 m.`,
      },
      parts: [
        { qty: 1, name: { en: "Multi-band antenna + ground plane", fr: "Antenne multi-bandes + plan de masse" } },
        { qty: 1, name: { en: "SMA cable", fr: "Câble SMA" } },
      ],
    },
    {
      id: "gnss-usb",
      title: { en: "Connect the receiver over USB", fr: "Brancher le récepteur en USB" },
      when: { gps: ["usb"] },
      body: {
        en: `Fix the receiver board on its bracket next to the compute board, plug the SMA cable, then one USB cable to the board:

- **simpleRTK2B / ZED-F9P boards:** the micro-USB port labelled *POWER+GPS* (not *POWER+XBEE*). The F9P enumerates as \`u-blox AG ... u-blox GNSS receiver\`.
- **UM980 / UM982 boards** (WTRTK-982 and similar): the USB-C / CH340 port; enumerates as \`1a86 USB Serial\`.

Use a **short shielded cable** and a black USB 2 port on a Pi 5. No hub for a single receiver; if you must use one, it needs its own power supply.

Check from SSH:

\`\`\`
ls -l /dev/serial/by-id/
\`\`\`

You should see a \`usb-u-blox_AG_-_www.u-blox.com_u-blox_GNSS_receiver-if00\` or \`usb-1a86_USB_Serial-if00-port0\` symlink. The installer will pick that stable path, so re-plugging never changes the device name.`,
        fr: `Fixez la carte récepteur sur son support à côté de la carte de calcul, branchez le câble SMA, puis un câble USB vers la carte :

- **simpleRTK2B / cartes ZED-F9P :** le port micro-USB marqué *POWER+GPS* (pas *POWER+XBEE*). Le F9P apparaît comme \`u-blox AG ... u-blox GNSS receiver\`.
- **Cartes UM980 / UM982** (WTRTK-982 et similaires) : le port USB-C / CH340 ; apparaît comme \`1a86 USB Serial\`.

Utilisez un **câble blindé court** et un port USB 2 noir sur un Pi 5. Pas de hub pour un seul récepteur ; s'il en faut un, il doit avoir sa propre alimentation.

Vérifiez en SSH :

\`\`\`
ls -l /dev/serial/by-id/
\`\`\`

Vous devez voir un lien \`usb-u-blox_AG_-_www.u-blox.com_u-blox_GNSS_receiver-if00\` ou \`usb-1a86_USB_Serial-if00-port0\`. L'installateur choisira ce chemin stable, donc rebrancher ne change jamais le nom du périphérique.`,
      },
      parts: [{ qty: 1, name: { en: "Short shielded USB cable (micro-USB or USB-C)", fr: "Câble USB blindé court (micro-USB ou USB-C)" } }],
    },
    {
      id: "gnss-uart",
      title: { en: "Connect the receiver over UART", fr: "Brancher le récepteur en UART" },
      when: { gps: ["uart"] },
      media: { type: "img", src: "img/pi-header.svg", alt: { en: "Pi header pins used", fr: "Broches du connecteur Pi utilisées" } },
      body: {
        en: `The installer's default GNSS port is **/dev/ttyAMA4 = UART4** on a Raspberry Pi 4/5, i.e. GPIO 8 / 9. Wire the receiver's UART1 header to the Pi's 40-pin header:

| Receiver | Pi header |
|---|---|
| 5V IN (or 3V3 IN — check your board) | pin 4 (5 V) |
| GND | pin 6 (GND) |
| TX | pin 21 (GPIO 9, RXD4) |
| RX | pin 24 (GPIO 8, TXD4) |

TX goes to RX and vice-versa. The receiver's UART pins are **3.3 V logic** — never a 5 V level.

The Pi UARTs only exist once the installer has added \`enable_uart=1\` and \`dtoverlay=uart4\` (it adds uart1…uart5 and disables Bluetooth) and the board has rebooted; the installer lists ports "available after reboot" and reminds you. On Raspberry Pi OS Bookworm, \`ttyAMA4\` is uart4 — verify with \`ls /dev/ttyAMA*\` after the reboot.

**Baud rate.** The installer probes the receiver and can raise it to **921600**, the validated speed. Leave a fresh F9P at its factory 38400 and let the installer negotiate.

> [!NOTE] Other boards
> Orange Pi and other SBCs expose different UART names (\`/dev/ttyS*\`). The installer shows what it detects; pick the one you wired and check the vendor's pinout for the GPIO numbers.`,
        fr: `Le port GNSS par défaut de l'installateur est **/dev/ttyAMA4 = UART4** sur un Raspberry Pi 4/5, soit GPIO 8 / 9. Câblez le connecteur UART1 du récepteur au connecteur 40 broches du Pi :

| Récepteur | Connecteur Pi |
|---|---|
| 5V IN (ou 3V3 IN — selon votre carte) | broche 4 (5 V) |
| GND | broche 6 (GND) |
| TX | broche 21 (GPIO 9, RXD4) |
| RX | broche 24 (GPIO 8, TXD4) |

TX va sur RX et inversement. Les broches UART du récepteur sont en **logique 3,3 V** — jamais un niveau 5 V.

Les UART du Pi n'existent qu'une fois que l'installateur a ajouté \`enable_uart=1\` et \`dtoverlay=uart4\` (il ajoute uart1…uart5 et désactive le Bluetooth) et que la carte a redémarré ; l'installateur liste les ports « disponibles après redémarrage » et vous le rappelle. Sur Raspberry Pi OS Bookworm, \`ttyAMA4\` correspond à uart4 — vérifiez avec \`ls /dev/ttyAMA*\` après redémarrage.

**Vitesse.** L'installateur sonde le récepteur et peut monter la vitesse à **921600**, la valeur validée. Laissez un F9P neuf à ses 38400 d'usine et laissez l'installateur négocier.

> [!NOTE] Autres cartes
> Les Orange Pi et autres SBC exposent d'autres noms d'UART (\`/dev/ttyS*\`). L'installateur affiche ce qu'il détecte ; choisissez celui que vous avez câblé et vérifiez le brochage GPIO du constructeur.`,
      },
      parts: [{ qty: 4, name: { en: "Dupont F-F wires, 3.3 V-logic receiver", fr: "Fils Dupont F-F, récepteur en logique 3,3 V" } }],
    },
    {
      id: "gnss-offset",
      title: { en: "Antenna offset — what the numbers mean", fr: "Offset d'antenne — ce que signifient les valeurs" },
      media: {
        type: "img",
        src: "img/antenna-offset-photo.jpg",
        alt: { en: "Measuring the antenna offset on a YardForce 500", fr: "Mesure de l'offset d'antenne sur une YardForce 500" },
        caption: { en: "Offset X from the rear axle centre to the antenna.", fr: "Offset X du centre de l'essieu arrière à l'antenne." },
        credit: { en: "Juditech3D (mowgli-docs, GPLv3)", fr: "Juditech3D (mowgli-docs, GPLv3)" },
      },
      body: {
        en: `The localizer fuses the raw GNSS fix through a **lever-arm factor**: it knows the antenna is \`gps_x\` metres ahead of the axle and rotates that offset with the robot's heading. A wrong offset shows up as a position that swings sideways when the robot turns, and as a dock approach that is consistently off by the same few centimetres.

| Key | Meaning | YardForce 500 preset |
|---|---|---|
| \`gps_x\` | forward distance, rear axle → antenna centre | 0.30 m |
| \`gps_y\` | lateral offset, **left positive** | 0.00 m |
| \`gps_z\` | height above ground | 0.20 m |

You enter them in the onboarding wizard's **Sensors** step (or later in Settings → Sensors). Measure to the centre of the antenna's top surface, to the centimetre. The photo shows the measurement on a 500 with the community bracket.`,
        fr: `Le localisateur fusionne la position GNSS brute via un **facteur de bras de levier** : il sait que l'antenne est \`gps_x\` mètres devant l'essieu et tourne cet offset avec le cap du robot. Un offset faux se voit par une position qui balance latéralement quand le robot tourne, et par une approche de station décalée toujours des mêmes centimètres.

| Clé | Signification | Préréglage YardForce 500 |
|---|---|---|
| \`gps_x\` | distance vers l'avant, essieu arrière → centre de l'antenne | 0,30 m |
| \`gps_y\` | décalage latéral, **positif à gauche** | 0,00 m |
| \`gps_z\` | hauteur au-dessus du sol | 0,20 m |

Vous les saisissez à l'étape **Capteurs** de l'assistant (ou plus tard dans Réglages → Capteurs). Mesurez jusqu'au centre du dessus de l'antenne, au centimètre. La photo montre la mesure sur une 500 avec le support communautaire.`,
      },
    },
    {
      id: "gnss-ntrip",
      title: { en: "Prepare your NTRIP corrections", fr: "Préparer vos corrections NTRIP" },
      body: {
        en: `RTK needs a stream of corrections from a base station less than ~30 km away, delivered over the internet by an **NTRIP caster**. Have these four values ready before the installer asks:

| Field | Example (Centipède, France) |
|---|---|
| Host | \`caster.centipede.fr\` |
| Port | \`2101\` |
| Mount point | the base nearest to you, e.g. \`XXXX\` from the map |
| Username / password | \`centipede\` / \`centipede\` (any non-empty value) |

Where to find a base:

- France and neighbours: [Centipède RTK map](https://map.centipede-rtk.org/) — free, community bases.
- Worldwide: [RTK2GO](http://www.rtk2go.com/) (free, register), national geodetic services (often paid), or your own base station with a second F9P.
- The old guide's [RTK helper](https://lvawebprojects.ovh/rtk/rtk.php) shows your coordinates and the nearest bases at once.

> [!WARNING] Privacy
> Never post your datum coordinates or NTRIP credentials publicly (forums, issues, Telegram screenshots). The GUI's diagnostics export redacts them; a raw config file does not.`,
        fr: `Le RTK a besoin d'un flux de corrections d'une base à moins de ~30 km, livré par internet via un **caster NTRIP**. Ayez ces quatre valeurs prêtes avant que l'installateur les demande :

| Champ | Exemple (Centipède, France) |
|---|---|
| Hôte | \`caster.centipede.fr\` |
| Port | \`2101\` |
| Point de montage | la base la plus proche, ex. \`XXXX\` sur la carte |
| Identifiant / mot de passe | \`centipede\` / \`centipede\` (n'importe quelle valeur non vide) |

Où trouver une base :

- France et voisins : [carte Centipède RTK](https://map.centipede-rtk.org/) — gratuit, bases communautaires.
- Monde : [RTK2GO](http://www.rtk2go.com/) (gratuit, inscription), services géodésiques nationaux (souvent payants), ou votre propre base avec un second F9P.
- L'[outil RTK](https://lvawebprojects.ovh/rtk/rtk.php) de l'ancien guide affiche vos coordonnées et les bases les plus proches d'un coup.

> [!WARNING] Vie privée
> Ne publiez jamais vos coordonnées de datum ni vos identifiants NTRIP (forums, tickets, captures Telegram). L'export diagnostics de l'interface les masque ; un fichier de configuration brut non.`,
      },
    },
  ],
});
