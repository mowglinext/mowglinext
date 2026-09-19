MOWGLI_MANUAL.chapters.push({
  id: "imu",
  icon: "🧭",
  title: { en: "IMU", fr: "IMU" },
  media: { type: "img", src: "img/j18.svg", alt: { en: "J18 pinout", fr: "Brochage de J18" } },
  steps: [
    {
      id: "imu-wire",
      title: { en: "Wire the IMU to J18", fr: "Câbler l'IMU sur J18" },
      body: {
        en: `The IMU talks I²C to the STM32 on the red **J18** header (bit-banged bus on PB3 / PB4). Four wires:

| IMU pin | J18 pin |
|---|---|
| VCC | **4** (5 V) — WT901 and GY-521 boards have their own 3.3 V regulator |
| GND | **1** (GND) |
| SCL | **5** |
| SDA | **6** |

Crimp a 2.0 mm-pitch housing (or use a HY2.0 pre-wired lead) on the J18 side, and Dupont or soldered wires on the IMU side. Keep the lead under ~25 cm: it is a bit-banged bus with no pull-up tuning.

> [!WARNING] Bare 3.3 V chips
> A bare LSM6 / ICM breakout without a regulator must be fed 3.3 V, not J18's 5 V. Check the board before plugging it in.

Pin numbering follows the community wiring diagram (verified on 500 and 500B). Confirm pin 1 on your board's silkscreen — an inverted plug puts 29 V (pin 8) where you expect GND.`,
        fr: `L'IMU parle en I²C au STM32 sur le connecteur rouge **J18** (bus logiciel sur PB3 / PB4). Quatre fils :

| Broche IMU | Broche J18 |
|---|---|
| VCC | **4** (5 V) — les modules WT901 et GY-521 ont leur propre régulateur 3,3 V |
| GND | **1** (GND) |
| SCL | **5** |
| SDA | **6** |

Sertissez un boîtier au pas de 2,0 mm (ou un fil HY2.0 pré-câblé) côté J18, et des fils Dupont ou soudés côté IMU. Gardez la liaison sous ~25 cm : c'est un bus logiciel sans réglage de pull-up.

> [!WARNING] Puces 3,3 V nues
> Un module LSM6 / ICM sans régulateur doit être alimenté en 3,3 V, pas en 5 V depuis J18. Vérifiez le module avant de le brancher.

La numérotation suit le schéma de câblage communautaire (vérifié sur 500 et 500B). Confirmez la broche 1 sur la sérigraphie de votre carte — une prise inversée met 29 V (broche 8) là où vous attendez la masse.`,
      },
      parts: [
        { qty: 1, name: { en: "IMU module (WT901 I²C / MPU6050)", fr: "Module IMU (WT901 I²C / MPU6050)" } },
        { qty: 1, name: { en: "4-wire lead with a 2.0 mm housing", fr: "Fil 4 conducteurs avec boîtier au pas de 2,0 mm" } },
      ],
    },
    {
      id: "imu-mount",
      title: { en: "Mount the IMU", fr: "Fixer l'IMU" },
      media: { type: "img", src: "img/axes.svg", alt: { en: "Robot axes and sensor offsets", fr: "Axes du robot et positions des capteurs" } },
      body: {
        en: `Where and how you fix the IMU matters more than which one you bought.

- **Rigid.** Screw or hot-glue it to the chassis (not to the floating shell, not to a cable). A wobbling IMU is a noisy gyro.
- **Flat.** Its board parallel to the ground. The firmware removes small tilts (it reports the implied mounting pitch/roll in the logs), but keep it under a few degrees.
- **Away from the motors and the blade ESC.** Magnetometers hate motor cables; the stock spot on the right-hand side, forward of the axle, is what the YardForce 500 preset assumes (\`imu_x 0.187 m\`, \`imu_y −0.195 m\`).
- **Any yaw.** You do **not** need to align the IMU's X axis with the robot's: the mounting yaw is solved by a short calibration drive in chapter 11 and stored as \`imu_yaw\`.

Write down where you put it relative to the rear axle centre (forward = +X, left = +Y) if you deviate from the preset — you enter it in the wizard's *Sensors* step.`,
        fr: `L'emplacement et la fixation de l'IMU comptent plus que le modèle acheté.

- **Rigide.** Vissez-la ou collez-la à la colle chaude sur le châssis (pas sur la coque flottante, pas sur un câble). Une IMU qui vibre, c'est un gyro bruité.
- **À plat.** Sa carte parallèle au sol. Le firmware compense les petites inclinaisons (il indique le tangage/roulis de montage dans les logs), mais restez sous quelques degrés.
- **Loin des moteurs et de l'ESC de lame.** Les magnétomètres détestent les câbles moteurs ; l'emplacement habituel côté droit, en avant de l'essieu, est celui que suppose le préréglage YardForce 500 (\`imu_x 0.187 m\`, \`imu_y −0.195 m\`).
- **Orientation libre.** Vous n'avez **pas** besoin d'aligner l'axe X de l'IMU avec celui du robot : le cap de montage est résolu par une courte calibration en mouvement au chapitre 11 et stocké dans \`imu_yaw\`.

Notez sa position par rapport au centre de l'essieu arrière (avant = +X, gauche = +Y) si vous vous écartez du préréglage — vous la saisirez à l'étape *Capteurs* de l'assistant.`,
      },
    },
    {
      id: "imu-verify",
      title: { en: "How you will know it works", fr: "Comment savoir qu'elle fonctionne" },
      body: {
        en: `You cannot test the IMU until the Mowgli firmware is flashed (chapter 8). At that point:

- the firmware's boot log lists \`Testing supported IMUs:\` followed by the detected chip;
- the GUI's **Diagnostics → Sensors** card shows live gyro rates and accelerations — rotate the robot by hand and watch yaw rate move;
- the IMU bias calibration runs automatically whenever the robot sits still on the dock (about 2 s of samples) and is stored on the Pi.

If nothing is detected: swap SCL/SDA (the most common mistake), check the 5 V on pin 4 with the multimeter, and make sure the lead is short.`,
        fr: `Vous ne pourrez tester l'IMU qu'une fois le firmware Mowgli flashé (chapitre 8). À ce moment :

- le journal de démarrage du firmware affiche \`Testing supported IMUs:\` suivi de la puce détectée ;
- la carte **Diagnostics → Capteurs** de l'interface montre les vitesses gyro et accélérations en direct — tournez le robot à la main et regardez la vitesse de lacet bouger ;
- la calibration du biais IMU se lance automatiquement quand le robot est immobile sur la station (environ 2 s d'échantillons) et est stockée sur le Pi.

Si rien n'est détecté : inversez SCL/SDA (l'erreur la plus fréquente), vérifiez le 5 V sur la broche 4 au multimètre, et assurez-vous que le fil est court.`,
      },
    },
  ],
});
