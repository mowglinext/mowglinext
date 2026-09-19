MOWGLI_MANUAL.chapters.push({
  id: "intro",
  icon: "🌱",
  title: { en: "Read this first", fr: "À lire avant de commencer" },
  media: { type: "img", src: "img/overview.svg", alt: { en: "System overview", fr: "Vue d'ensemble du système" } },
  steps: [
    {
      id: "intro-what",
      title: { en: "What you are about to build", fr: "Ce que vous allez construire" },
      body: {
        en: `You will turn a **YardForce 500 / 500B** (or a sibling chassis) into an autonomous mower that knows where it is to the centimetre, drives planned stripes, avoids obstacles and docks by itself — with no boundary wire.

The stock mainboard stays. Everything happens **around** it:

- the stock STM32 mainboard gets the open-source **Mowgli firmware** (motors, blade, battery and every safety decision stay in firmware);
- a small **Linux compute board** (Raspberry Pi 5 or equivalent) runs the MowgliNext stack — ROS 2, Nav2, the localizer and the web interface — in Docker containers;
- an **RTK GNSS receiver** gives centimetre positioning, an **IMU** gives heading, and an optional **LiDAR** sees obstacles.

The diagram on the left is the whole build. Every chapter of this manual fills in one box or one arrow.

> [!TIP] How this manual works
> Pick your hardware under **Your build** in the sidebar: steps that do not apply to your mower, GNSS wiring or LiDAR choice are hidden and the step counter adapts. Use ← → on the keyboard to move between steps, and *Mark as done* to keep track. Your progress stays in this browser.`,
        fr: `Vous allez transformer une **YardForce 500 / 500B** (ou un châssis cousin) en tondeuse autonome qui sait où elle est au centimètre, tond en bandes planifiées, évite les obstacles et rentre seule à sa station — sans fil périphérique.

La carte mère d'origine reste en place. Tout se passe **autour** d'elle :

- la carte mère STM32 d'origine reçoit le **firmware Mowgli** open source (moteurs, lame, batterie et toutes les décisions de sécurité restent dans le firmware) ;
- une petite **carte Linux** (Raspberry Pi 5 ou équivalent) fait tourner la pile MowgliNext — ROS 2, Nav2, le localisateur et l'interface web — dans des conteneurs Docker ;
- un **récepteur GNSS RTK** donne la position au centimètre, une **IMU** donne le cap, et un **LiDAR** optionnel voit les obstacles.

Le schéma à gauche est tout le projet. Chaque chapitre de ce guide remplit une case ou une flèche.

> [!TIP] Comment utiliser ce guide
> Choisissez votre matériel sous **Votre montage** dans la barre latérale : les étapes qui ne concernent pas votre tondeuse, votre câblage GNSS ou votre choix de LiDAR sont masquées et le compteur s'adapte. Les flèches ← → du clavier changent d'étape, *Marquer comme faite* garde une trace. Votre progression reste dans ce navigateur.`,
      },
    },
    {
      id: "intro-safety",
      title: { en: "Safety and disclaimer", fr: "Sécurité et avertissement" },
      body: {
        en: `> [!DANGER] A mower has spinning blades
> Everything in this manual is done **at your own risk**. Opening the robot and flashing its firmware voids the manufacturer warranty. During tests the robot can move or start the blade unexpectedly. The authors and contributors accept no liability for damage or injury.

Rules that apply to every step of this manual:

1. **Remove the blades** before any bench work, and put them back only for the first mow. A firmware flash reboots the board with the motors powered.
2. **Disconnect the battery** before touching a connector on the mainboard.
3. The STM32 firmware is the **only** blade and emergency-stop authority. Nothing on the Pi can override the stop buttons, the lift sensors or the tilt sensor — do not try to.
4. Stay next to the robot during every calibration drive and the first mow. Know where the **STOP** button is.
5. Keep children and pets away from the lawn while testing.

> [!WARNING] Firmware
> Only the **mainboard** is ever flashed. Never flash the panel (button/LED board): it can become unrecoverable.`,
        fr: `> [!DANGER] Une tondeuse a des lames qui tournent
> Tout ce qui suit est réalisé **sous votre entière responsabilité**. Ouvrir le robot et flasher son firmware annule la garantie constructeur. Pendant les tests, le robot peut bouger ou démarrer la lame de façon inattendue. Les auteurs et contributeurs déclinent toute responsabilité en cas de dommage ou de blessure.

Règles valables pour chaque étape de ce guide :

1. **Retirez les lames** avant tout travail sur l'établi, et remettez-les seulement pour la première tonte. Un flash du firmware redémarre la carte avec les moteurs alimentés.
2. **Débranchez la batterie** avant de toucher un connecteur de la carte mère.
3. Le firmware STM32 est la **seule** autorité pour la lame et l'arrêt d'urgence. Rien sur le Pi ne peut contourner les boutons STOP, les capteurs de levage ou d'inclinaison — n'essayez pas.
4. Restez à côté du robot pendant chaque calibration en mouvement et la première tonte. Sachez où est le bouton **STOP**.
5. Éloignez enfants et animaux de la pelouse pendant les essais.

> [!WARNING] Firmware
> Seule la **carte mère** est flashée. Ne flashez jamais le panneau (carte boutons/voyants) : il peut devenir irrécupérable.`,
      },
    },
    {
      id: "intro-models",
      title: { en: "Supported mowers", fr: "Tondeuses compatibles" },
      body: {
        en: `The Mowgli firmware runs on the stock YardForce "GForce" mainboard family. The GUI ships a preset for each chassis (dimensions, wheel geometry, cutting width, encoder ticks, battery thresholds):

| Model | Mainboard MCU | Status |
|---|---|---|
| YardForce Classic 500 | STM32F103 | Primary target — everything in this manual was field-tested on it |
| YardForce 500B | STM32F401 | Supported — different blade-motor UART and panel, own firmware build |
| YardForce SA650 | STM32F103 | Supported |
| YardForce 900 ECO | STM32F103 | Supported |
| YardForce LUV1000RI | — | Preset exists, **no firmware build yet** (pinout unknown) |
| Sabo MOWiT 500F | — | Supported (YardForce rebrand) |
| Custom robot | — | Every chassis parameter entered by hand |

**How to tell a 500 from a 500B:** the 500B has the newer panel with a keypad and a display of LEDs; on the mainboard the MCU marking reads STM32F401 instead of STM32F103. When in doubt, read the chip.

Select your model in the sidebar now — the 500 and 500B differ in a few steps (firmware build, panel).`,
        fr: `Le firmware Mowgli tourne sur la famille de cartes mères YardForce « GForce » d'origine. L'interface fournit un préréglage par châssis (dimensions, géométrie des roues, largeur de coupe, ticks encodeur, seuils batterie) :

| Modèle | MCU carte mère | Statut |
|---|---|---|
| YardForce Classic 500 | STM32F103 | Cible principale — tout ce guide a été testé sur le terrain avec elle |
| YardForce 500B | STM32F401 | Compatible — UART moteur de lame et panneau différents, build firmware dédié |
| YardForce SA650 | STM32F103 | Compatible |
| YardForce 900 ECO | STM32F103 | Compatible |
| YardForce LUV1000RI | — | Préréglage présent, **pas encore de build firmware** (brochage inconnu) |
| Sabo MOWiT 500F | — | Compatible (YardForce rebadgée) |
| Robot custom | — | Tous les paramètres châssis saisis à la main |

**Distinguer une 500 d'une 500B :** la 500B a le panneau récent avec clavier et voyants ; sur la carte mère, le marquage du MCU indique STM32F401 au lieu de STM32F103. En cas de doute, lisez la puce.

Sélectionnez votre modèle dans la barre latérale dès maintenant — la 500 et la 500B diffèrent sur quelques étapes (build firmware, panneau).`,
      },
    },
    {
      id: "intro-time",
      title: { en: "Skills, time and budget", fr: "Compétences, temps et budget" },
      body: {
        en: `**Skills.** Basic soldering or crimping (a handful of wires), a screwdriver, and the confidence to paste a command into an SSH terminal. No programming and no ROS knowledge is needed: the installer and the GUI wizard do the software side.

**Time.** Plan two sessions:

| Phase | Typical time |
|---|---|
| Print brackets, order parts | while waiting for delivery |
| Open the mower, mount and wire everything | one afternoon |
| Flash the Pi, run the installer, flash the firmware | 1–2 h, mostly waiting for downloads |
| Onboarding wizard, calibrations, record the first area | 1–2 h outdoors, RTK-Fixed required |

**Budget** (2026 street prices, excluding the mower): about 250–400 € — the RTK receiver and antenna are the bulk of it, the LiDAR adds ~80 €.

**What you do NOT need** any more compared with the original Mowgli / OpenMower build:

- no PS3/Xbox controller — areas are recorded from the web GUI on your phone;
- no STM32CubeProgrammer or Windows tooling — the GUI flashes the firmware through the ST-Link plugged into the Pi;
- no hand-edited config files — the installer and the GUI write them.`,
        fr: `**Compétences.** Un peu de soudure ou de sertissage (une poignée de fils), un tournevis, et l'aisance pour coller une commande dans un terminal SSH. Aucune programmation ni connaissance de ROS : l'installateur et l'assistant de l'interface font la partie logicielle.

**Temps.** Prévoyez deux sessions :

| Phase | Durée typique |
|---|---|
| Imprimer les supports, commander les pièces | pendant la livraison |
| Ouvrir la tondeuse, monter et câbler | une après-midi |
| Flasher le Pi, lancer l'installateur, flasher le firmware | 1–2 h, surtout des téléchargements |
| Assistant de démarrage, calibrations, première zone | 1–2 h dehors, RTK Fixed obligatoire |

**Budget** (prix 2026, hors tondeuse) : environ 250–400 € — le récepteur RTK et l'antenne en sont l'essentiel, le LiDAR ajoute ~80 €.

**Ce dont vous n'avez PLUS besoin** par rapport au montage Mowgli / OpenMower d'origine :

- pas de manette PS3/Xbox — les zones s'enregistrent depuis l'interface web sur votre téléphone ;
- pas de STM32CubeProgrammer ni d'outillage Windows — l'interface flashe le firmware via le ST-Link branché au Pi ;
- pas de fichiers de configuration à éditer à la main — l'installateur et l'interface les écrivent.`,
      },
    },
    {
      id: "intro-help",
      title: { en: "Where to get help", fr: "Où trouver de l'aide" },
      body: {
        en: `- **Wiki** — reference documentation: [github.com/mowglinext/mowglinext/wiki](https://github.com/mowglinext/mowglinext/wiki)
- **Discussions** — questions and build logs: [GitHub Discussions](https://github.com/mowglinext/mowglinext/discussions)
- **Bugs** — [open an issue](https://github.com/mowglinext/mowglinext/issues/new/choose) with the GUI's diagnostics export
- **French-speaking community** — the Telegram group that wrote the original Mowgli guide: [Telegram Mowgli FR](https://t.me/+x6U3UwU5lB4yOWNk)
- **First-boot checklist** — the short post-install list this manual expands on: [docs/FIRST_BOOT.md](https://github.com/mowglinext/mowglinext/blob/main/docs/FIRST_BOOT.md)

This manual is part of the repository (\`docs/build/\`). Found a mistake? Edit the chapter file and open a pull request — the last chapter explains how.`,
        fr: `- **Wiki** — documentation de référence : [github.com/mowglinext/mowglinext/wiki](https://github.com/mowglinext/mowglinext/wiki)
- **Discussions** — questions et journaux de montage : [GitHub Discussions](https://github.com/mowglinext/mowglinext/discussions)
- **Bugs** — [ouvrir un ticket](https://github.com/mowglinext/mowglinext/issues/new/choose) avec l'export diagnostics de l'interface
- **Communauté francophone** — le groupe Telegram à l'origine du guide Mowgli : [Telegram Mowgli FR](https://t.me/+x6U3UwU5lB4yOWNk)
- **Checklist premier démarrage** — la liste courte post-installation que ce guide détaille : [docs/FIRST_BOOT.md](https://github.com/mowglinext/mowglinext/blob/main/docs/FIRST_BOOT.md)

Ce guide fait partie du dépôt (\`docs/build/\`). Une erreur ? Modifiez le fichier du chapitre et ouvrez une pull request — le dernier chapitre explique comment.`,
      },
    },
  ],
});
