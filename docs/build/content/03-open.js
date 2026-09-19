MOWGLI_MANUAL.chapters.push({
  id: "open",
  icon: "🔧",
  title: { en: "Open the mower", fr: "Ouvrir la tondeuse" },
  media: { type: "img", src: "img/mainboard.svg", alt: { en: "Mainboard connector map", fr: "Plan des connecteurs de la carte mère" } },
  steps: [
    {
      id: "open-prepare",
      title: { en: "Blades off, battery off", fr: "Lames retirées, batterie coupée" },
      body: {
        en: `1. Switch the mower off with its power button and take it off the dock.
2. Turn it over on a blanket and **remove the three blades** from the blade disc (Torx screws). Store them with the screws — they go back on only in the last chapter.
3. Open the battery compartment underneath and **unplug the battery connector**. From now on the board is unpowered until the manual says otherwise.

> [!TIP] Take photos
> Photograph every connector before you unplug anything. The YardForce harness is keyed but the photos save you a guess later, and they are what people will ask for on Telegram when something does not work.`,
        fr: `1. Éteignez la tondeuse avec son bouton et sortez-la de la station.
2. Retournez-la sur une couverture et **retirez les trois lames** du disque (vis Torx). Rangez-les avec leurs vis — elles ne reviennent qu'au dernier chapitre.
3. Ouvrez le compartiment batterie dessous et **débranchez le connecteur de la batterie**. À partir de maintenant, la carte est hors tension jusqu'à indication contraire.

> [!TIP] Prenez des photos
> Photographiez chaque connecteur avant de débrancher quoi que ce soit. Le faisceau YardForce est détrompé mais les photos évitent de deviner plus tard, et c'est ce qu'on vous demandera sur Telegram si quelque chose ne marche pas.`,
      },
    },
    {
      id: "open-shell",
      title: { en: "Remove the top shell", fr: "Retirer le capot" },
      body: {
        en: `The top shell is held by Torx screws around the rim (some hidden under rubber plugs and under the rear panel bezel). Lift it gently: the **panel board** (buttons and LEDs) is wired to the mainboard through a short ribbon — unplug it at the mainboard side (J6) only if you need the room, and plug it back before any power-up.

Under the shell you find:

- the **mainboard** in the centre, under a plastic cover;
- the front cavity where the original electronics leave room for the compute board and the GNSS receiver;
- the flat top surface at the front, the usual spot for the GNSS antenna (and the LiDAR on the highest point).

> [!WARNING] Panel required
> The stop buttons and lift sensors on the panel are hard-wired into the safety chain. The blade will not start without the panel plugged in — and it must not, so leave it that way.`,
        fr: `Le capot tient par des vis Torx sur le pourtour (certaines cachées sous des bouchons en caoutchouc et sous l'enjoliveur du panneau arrière). Soulevez-le doucement : la **carte panneau** (boutons et voyants) est reliée à la carte mère par une courte nappe — débranchez-la côté carte mère (J6) seulement si vous avez besoin de place, et rebranchez-la avant toute mise sous tension.

Sous le capot vous trouvez :

- la **carte mère** au centre, sous un cache plastique ;
- la cavité avant où l'électronique d'origine laisse la place pour la carte de calcul et le récepteur GNSS ;
- la surface plane à l'avant, l'emplacement habituel de l'antenne GNSS (et du LiDAR au point le plus haut).

> [!WARNING] Panneau obligatoire
> Les boutons STOP et les capteurs de levage du panneau sont câblés en dur dans la chaîne de sécurité. La lame ne démarre pas sans le panneau branché — et elle ne doit pas, alors laissez-le ainsi.`,
      },
    },
    {
      id: "open-connectors",
      title: { en: "Locate the mainboard connectors", fr: "Repérer les connecteurs de la carte mère" },
      body: {
        en: `Remove the plastic cover over the mainboard. Four connectors matter for this build — find them now and compare with the silkscreen (the diagram on the left is schematic, not to scale):

| Connector | What it is | Used for |
|---|---|---|
| **J14** | USB header / socket | Data link to the compute board (USB A-to-A) |
| **J18** | Red 9-pin header | IMU (I²C), 5 V, battery tap; optional firmware debug UART |
| **J9** | 4-pin header near the MCU: GND · SWCL · SWDA · 3V3 | ST-Link programming (SWD) |
| **J3** | Battery input (after the power button) | 29 V tap for the DC-DC converter |
| J6 | Panel ribbon | leave connected |

Also note the MCU marking next to J9: **STM32F103** = Classic 500 (and SA650 / 900 ECO), **STM32F401** = 500B. This decides the firmware build.`,
        fr: `Retirez le cache plastique de la carte mère. Quatre connecteurs comptent pour ce montage — repérez-les et comparez avec la sérigraphie (le schéma à gauche est indicatif, pas à l'échelle) :

| Connecteur | Nature | Usage |
|---|---|---|
| **J14** | Embase / prise USB | Liaison données vers la carte de calcul (USB A vers A) |
| **J18** | Connecteur rouge 9 broches | IMU (I²C), 5 V, prise batterie ; UART de debug firmware optionnel |
| **J9** | 4 broches près du MCU : GND · SWCL · SWDA · 3V3 | Programmation ST-Link (SWD) |
| **J3** | Entrée batterie (après le bouton) | Prise 29 V pour le convertisseur DC-DC |
| J6 | Nappe du panneau | laisser branchée |

Notez aussi le marquage du MCU à côté de J9 : **STM32F103** = Classic 500 (et SA650 / 900 ECO), **STM32F401** = 500B. C'est ce qui détermine le build firmware.`,
      },
    },
    {
      id: "open-backup",
      title: { en: "Back up the stock firmware (recommended)", fr: "Sauvegarder le firmware d'origine (recommandé)" },
      media: {
        type: "img",
        src: "img/stlink-mainboard.jpg",
        alt: { en: "ST-Link wires plugged on the mainboard J9 header", fr: "Fils du ST-Link branchés sur le connecteur J9 de la carte mère" },
        caption: { en: "ST-Link on J9, mainboard side.", fr: "ST-Link sur J9, côté carte mère." },
        credit: { en: "Juditech3D (mowgli-docs, GPLv3)", fr: "Juditech3D (mowgli-docs, GPLv3)" },
      },
      body: {
        en: `A backup lets you return the mower to stock (resale, warranty claim, or just curiosity). It takes five minutes once the ST-Link is wired, and you will wire it anyway in the firmware chapter.

The repository ships scripts for this: [\`firmware/stm32/mainboard_firmware/\`](https://github.com/mowglinext/mowglinext/tree/main/firmware/stm32/mainboard_firmware) (\`backup_firmware.sh\` / \`restore_firmware.sh\`, OpenOCD + ST-Link). Known-good SHA256 sums of stock images are listed there so you can confirm the dump is complete.

Simplest path: do it **from the Pi** once it is set up (chapter 8 wires the ST-Link and installs OpenOCD), before the GUI flashes Mowgli. Or do it now from a laptop with OpenOCD installed and the ST-Link on J9.

> [!NOTE] YardForce 500 / 500B
> Stock images for both are already archived by the community, so this step is optional for them. It stays recommended for any other chassis.`,
        fr: `Une sauvegarde permet de remettre la tondeuse d'origine (revente, garantie, ou simple curiosité). Cela prend cinq minutes une fois le ST-Link câblé, et vous le câblerez de toute façon au chapitre firmware.

Le dépôt fournit les scripts : [\`firmware/stm32/mainboard_firmware/\`](https://github.com/mowglinext/mowglinext/tree/main/firmware/stm32/mainboard_firmware) (\`backup_firmware.sh\` / \`restore_firmware.sh\`, OpenOCD + ST-Link). Les SHA256 des images d'origine connues y sont listés pour vérifier que le dump est complet.

Le plus simple : le faire **depuis le Pi** une fois configuré (le chapitre 8 câble le ST-Link et installe OpenOCD), avant que l'interface flashe Mowgli. Ou dès maintenant depuis un portable avec OpenOCD et le ST-Link sur J9.

> [!NOTE] YardForce 500 / 500B
> Les images d'origine des deux modèles sont déjà archivées par la communauté, l'étape est donc optionnelle pour elles. Elle reste recommandée pour tout autre châssis.`,
      },
    },
  ],
});
