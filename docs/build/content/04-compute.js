MOWGLI_MANUAL.chapters.push({
  id: "compute",
  icon: "🧠",
  title: { en: "Compute board", fr: "Carte de calcul" },
  media: { type: "img", src: "img/power.svg", alt: { en: "Powering the compute board", fr: "Alimentation de la carte de calcul" } },
  steps: [
    {
      id: "compute-os",
      title: { en: "Write the operating system", fr: "Écrire le système d'exploitation" },
      body: {
        en: `Do this on your computer, with the SSD (or eMMC / SD) attached through an adapter.

**Raspberry Pi 5 / 4** — Raspberry Pi Imager:

1. Choose device → your Pi model. Choose OS → **Raspberry Pi OS Lite (64-bit)**. No desktop: it only wastes memory on a headless robot.
2. Choose storage → the SSD. Click **Next**, then **Edit settings**:
   - hostname: \`mowgli\`;
   - username and password (write them down — there is no default password any more);
   - Wi-Fi SSID, password and country;
   - **Services → Enable SSH** with password authentication.
3. Write, then move the SSD to the Pi. On a Pi 5 with an M.2 HAT, boot order defaults to NVMe once no SD card is present.

**Orange Pi 5B / other boards** — write the vendor's **Ubuntu 22.04/24.04 server (64-bit)** or Armbian image with balenaEtcher, then create the user and enable SSH per the board's documentation. The installer only needs a 64-bit Debian/Ubuntu-family OS with \`systemd\` and \`sudo\`.

> [!TIP]
> Reserve a fixed IP for the board in your router (DHCP reservation on its MAC address). You will type this address a lot.`,
        fr: `À faire sur votre ordinateur, avec le SSD (ou eMMC / SD) branché via un adaptateur.

**Raspberry Pi 5 / 4** — Raspberry Pi Imager :

1. Choisir le modèle → votre Pi. Choisir l'OS → **Raspberry Pi OS Lite (64-bit)**. Pas de bureau : il ne fait que gaspiller de la mémoire sur un robot sans écran.
2. Choisir le stockage → le SSD. **Suivant**, puis **Modifier réglages** :
   - nom d'hôte : \`mowgli\` ;
   - utilisateur et mot de passe (notez-les — il n'y a plus de mot de passe par défaut) ;
   - SSID Wi-Fi, mot de passe et pays ;
   - **Services → Activer SSH** avec authentification par mot de passe.
3. Écrire, puis mettre le SSD dans le Pi. Sur un Pi 5 avec HAT M.2, l'ordre de démarrage passe sur le NVMe dès qu'il n'y a pas de carte SD.

**Orange Pi 5B / autres cartes** — écrivez l'image **Ubuntu 22.04/24.04 server (64 bits)** du constructeur ou Armbian avec balenaEtcher, puis créez l'utilisateur et activez SSH selon la documentation de la carte. L'installateur n'a besoin que d'un OS 64 bits de la famille Debian/Ubuntu avec \`systemd\` et \`sudo\`.

> [!TIP]
> Réservez une IP fixe pour la carte dans votre box (réservation DHCP sur son adresse MAC). Vous taperez cette adresse souvent.`,
      },
    },
    {
      id: "compute-ssh",
      title: { en: "First login over SSH", fr: "Première connexion en SSH" },
      body: {
        en: `Power the board from a normal USB-C supply for now (the mower is still open and unpowered). Wait a minute, then from your computer:

\`\`\`
ssh <user>@mowgli.local
# or, if mDNS does not resolve on your network:
ssh <user>@192.168.x.x
\`\`\`

Find the address in your router's client list if needed. Once logged in, bring the system up to date and reboot:

\`\`\`
sudo apt update && sudo apt full-upgrade -y
sudo reboot
\`\`\`

Everything else (Docker, udev rules, UART overlays, the containers) is done by the MowgliNext installer in chapter 9 — do **not** install Docker by hand.

> [!TIP] Windows users
> \`ssh\` is built into PowerShell and Windows Terminal on Windows 10/11. MobaXterm or PuTTY work too.`,
        fr: `Alimentez la carte avec une alimentation USB-C classique pour l'instant (la tondeuse est encore ouverte et hors tension). Attendez une minute, puis depuis votre ordinateur :

\`\`\`
ssh <utilisateur>@mowgli.local
# ou, si le mDNS ne résout pas sur votre réseau :
ssh <utilisateur>@192.168.x.x
\`\`\`

Trouvez l'adresse dans la liste des clients de votre box si besoin. Une fois connecté, mettez le système à jour et redémarrez :

\`\`\`
sudo apt update && sudo apt full-upgrade -y
sudo reboot
\`\`\`

Tout le reste (Docker, règles udev, overlays UART, les conteneurs) est fait par l'installateur MowgliNext au chapitre 9 — n'installez **pas** Docker à la main.

> [!TIP] Sous Windows
> \`ssh\` est intégré à PowerShell et Windows Terminal sur Windows 10/11. MobaXterm ou PuTTY conviennent aussi.`,
      },
    },
    {
      id: "compute-dcdc",
      title: { en: "Set the DC-DC converter to 5.1 V", fr: "Régler le convertisseur DC-DC à 5,1 V" },
      body: {
        en: `Do this on the bench **with nothing connected to the converter's output**.

1. Solder or screw two wires on the converter's **IN+ / IN−**. Temporarily feed it from any 12–30 V source (a bench supply, a 12 V adapter, or the mower battery through J3 with the mainboard off).
2. Put the multimeter on **OUT+ / OUT−** and turn the trimmer until it reads **5.10 V** (many boards ship at 12 V or more — a Pi does not survive that).
3. If the converter has a current-limit trimmer (XL4015 CC/CV boards), turn it fully clockwise (maximum).
4. Note the measured voltage on a piece of tape on the converter.

> [!DANGER]
> Never connect the compute board before the output is confirmed at 5.1 V. One turn of the trimmer in the wrong direction is the most common way builders have destroyed a Pi.`,
        fr: `À faire sur l'établi **sans rien de branché sur la sortie du convertisseur**.

1. Soudez ou vissez deux fils sur **IN+ / IN−** du convertisseur. Alimentez-le provisoirement en 12–30 V (alimentation de labo, adaptateur 12 V, ou la batterie de la tondeuse via J3 avec la carte mère éteinte).
2. Multimètre sur **OUT+ / OUT−**, tournez le potentiomètre jusqu'à lire **5,10 V** (beaucoup de modules sortent d'usine à 12 V ou plus — un Pi n'y survit pas).
3. Si le convertisseur a un réglage de limite de courant (modules XL4015 CC/CV), tournez-le à fond dans le sens horaire (maximum).
4. Notez la tension mesurée sur un bout de ruban collé sur le convertisseur.

> [!DANGER]
> Ne branchez jamais la carte de calcul avant d'avoir confirmé 5,1 V en sortie. Un tour de potentiomètre dans le mauvais sens est la façon la plus courante de griller un Pi.`,
      },
    },
    {
      id: "compute-mount",
      title: { en: "Mount the board in the mower", fr: "Monter la carte dans la tondeuse" },
      body: {
        en: `1. Fit the compute board and the DC-DC on the printed bracket (heat-set inserts or M2.5 screws). Add the cooler / heatsink — the shell is a closed box in the sun.
2. Place the bracket in the front cavity. Check that the shell still closes and that nothing touches the blade-motor housing or the wheel arms.
3. Route the DC-DC **input** wires to the mainboard: **J3** (battery after the power button — the recommended tap, so the Pi shuts down with the mower) or **J18 pin 8 (VBat) + pin 1 (GND)**, 2 A max. Use a crimped 2.0 mm housing on J18; on J3 piggy-back the existing terminals or use an inline tap.
4. Route the DC-DC **output** to the board: header **pin 4 (5 V)** and **pin 6 (GND)** on a Pi, twisted pair, as short as possible. Alternatively wire a USB-C plug — it keeps the board's input fuse but a Pi 5 will complain about an "unofficial" supply unless the converter can do 5 A.
5. Plug the **USB A-to-A** cable between a Pi USB port and mainboard **J14**. On a Pi 5, keep the blue USB 3 ports for the SSD if you use a USB SSD, and use the black USB 2 ports for the mainboard, GNSS and ST-Link.

> [!WARNING]
> Do not power the board from the mainboard's J14 USB. It is a data connector; the STM32 side is not designed to be back-fed.`,
        fr: `1. Fixez la carte de calcul et le DC-DC sur le support imprimé (inserts thermiques ou vis M2,5). Ajoutez le ventilateur / dissipateur — la coque est une boîte fermée au soleil.
2. Placez le support dans la cavité avant. Vérifiez que le capot ferme toujours et que rien ne touche le carter du moteur de lame ni les bras de roues.
3. Amenez les fils **d'entrée** du DC-DC à la carte mère : **J3** (batterie après le bouton — la prise recommandée, le Pi s'éteint avec la tondeuse) ou **J18 broche 8 (VBat) + broche 1 (GND)**, 2 A max. Boîtier serti au pas de 2,0 mm sur J18 ; sur J3, doublez les cosses existantes ou utilisez un raccord en ligne.
4. Amenez la **sortie** du DC-DC à la carte : **broche 4 (5 V)** et **broche 6 (GND)** du connecteur d'un Pi, paire torsadée, la plus courte possible. Sinon câblez une fiche USB-C — elle conserve le fusible d'entrée mais un Pi 5 se plaindra d'une alimentation « non officielle » si le convertisseur ne fournit pas 5 A.
5. Branchez le câble **USB A vers A** entre un port USB du Pi et **J14** de la carte mère. Sur un Pi 5, gardez les ports USB 3 bleus pour le SSD si vous utilisez un SSD USB, et les ports USB 2 noirs pour la carte mère, le GNSS et le ST-Link.

> [!WARNING]
> N'alimentez pas la carte depuis le J14 USB de la carte mère. C'est un connecteur de données ; le côté STM32 n'est pas prévu pour être alimenté à l'envers.`,
      },
      parts: [
        { qty: 1, name: { en: "Printed bracket, M2.5 screws / inserts", fr: "Support imprimé, vis / inserts M2,5" } },
        { qty: 1, name: { en: "USB A-to-A cable", fr: "Câble USB A vers A" } },
      ],
    },
    {
      id: "compute-check",
      title: { en: "Power-up check", fr: "Vérification à la mise sous tension" },
      body: {
        en: `Reconnect the battery, switch the mower on with its button, and check:

- [ ] the DC-DC output still reads 5.1 V under load (measure at the board's pins);
- [ ] the board boots (activity LED) and you can \`ssh\` into it over Wi-Fi from inside the shell;
- [ ] \`lsusb\` on the board lists \`0483:5740 STMicroelectronics Virtual COM Port\` — that is the mainboard on J14 (it enumerates even with the stock firmware);
- [ ] the mower's panel behaves as usual (stock firmware still installed).

\`\`\`
lsusb
dmesg | grep -i -A2 "STM\\|cdc_acm" | tail
\`\`\`

If the board reboots when you press the mower's START button or the wheels twitch, add the 470–1000 µF capacitor across the DC-DC output and shorten the 5 V wires.`,
        fr: `Rebranchez la batterie, allumez la tondeuse avec son bouton, et vérifiez :

- [ ] la sortie du DC-DC lit toujours 5,1 V en charge (mesurez sur les broches de la carte) ;
- [ ] la carte démarre (LED d'activité) et vous pouvez vous y connecter en \`ssh\` par Wi-Fi depuis l'intérieur de la coque ;
- [ ] \`lsusb\` sur la carte liste \`0483:5740 STMicroelectronics Virtual COM Port\` — c'est la carte mère sur J14 (elle s'énumère même avec le firmware d'origine) ;
- [ ] le panneau de la tondeuse se comporte normalement (firmware d'origine toujours en place).

\`\`\`
lsusb
dmesg | grep -i -A2 "STM\\|cdc_acm" | tail
\`\`\`

Si la carte redémarre quand vous appuyez sur START ou que les roues bougent, ajoutez le condensateur 470–1000 µF sur la sortie du DC-DC et raccourcissez les fils 5 V.`,
      },
    },
  ],
});
