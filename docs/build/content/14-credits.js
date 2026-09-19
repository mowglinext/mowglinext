MOWGLI_MANUAL.chapters.push({
  id: "credits",
  icon: "💚",
  title: { en: "Credits and contributing", fr: "Crédits et contribution" },
  steps: [
    {
      id: "credits-people",
      title: { en: "Who made this possible", fr: "Ceux qui ont rendu cela possible" },
      body: {
        en: `This manual adapts the **Mowgli Docs** guide written by [Judicaël (Juditech3D)](https://github.com/juditech3D) for the original Mowgli + OpenMower build — its structure, its wiring diagram, its photos and its hard-won troubleshooting list. Reused under the GPLv3, like everything here. The original lives at [mowglifrenchtouch.github.io/mowgli-docs](https://mowglifrenchtouch.github.io/mowgli-docs/).

- **[cloudn1ne](https://github.com/cloudn1ne)** — reverse-engineered the YardForce mainboard and wrote the first Mowgli firmware.
- **[cedbossneo](https://github.com/cedbossneo)** — the Mowgli firmware fork, mowgli-docker, and the MowgliNext stack.
- **nekraus** — the 500B panel support and countless late nights.
- **[Pepeuch](https://makerworld.com/fr/@user_3228887730)** — Universal GNSS / Unicore integration, the 500B printed block, field testing.
- **Etienne** — the community Mowgli PCB.
- **[tetiti20](https://github.com/tetiti20)** and every tester filing bugs.
- **[OpenMower](https://openmower.de/)** — for proving it could be done.
- The **French Telegram community** that keeps the knowledge flowing.

Full contributor list: [github.com/mowglinext/mowglinext/graphs/contributors](https://github.com/mowglinext/mowglinext/graphs/contributors).`,
        fr: `Ce guide adapte le guide **Mowgli Docs** écrit par [Judicaël (Juditech3D)](https://github.com/juditech3D) pour le montage Mowgli + OpenMower d'origine — sa structure, son schéma de câblage, ses photos et sa liste de dépannage durement acquise. Réutilisé sous GPLv3, comme tout ici. L'original est sur [mowglifrenchtouch.github.io/mowgli-docs](https://mowglifrenchtouch.github.io/mowgli-docs/).

- **[cloudn1ne](https://github.com/cloudn1ne)** — a rétro-conçu la carte mère YardForce et écrit le premier firmware Mowgli.
- **[cedbossneo](https://github.com/cedbossneo)** — le fork du firmware Mowgli, mowgli-docker, et la pile MowgliNext.
- **nekraus** — le support du panneau 500B et d'innombrables nuits blanches.
- **[Pepeuch](https://makerworld.com/fr/@user_3228887730)** — intégration Universal GNSS / Unicore, le bloc imprimé 500B, les tests terrain.
- **Etienne** — le PCB Mowgli communautaire.
- **[tetiti20](https://github.com/tetiti20)** et tous les testeurs qui signalent les bugs.
- **[OpenMower](https://openmower.de/)** — pour avoir prouvé que c'était possible.
- La **communauté Telegram francophone** qui fait circuler le savoir.

Liste complète des contributeurs : [github.com/mowglinext/mowglinext/graphs/contributors](https://github.com/mowglinext/mowglinext/graphs/contributors).`,
      },
    },
    {
      id: "credits-edit",
      title: { en: "Improve this manual", fr: "Améliorer ce guide" },
      body: {
        en: `The manual is plain files in the repository, no build step:

\`\`\`
docs/build/
├── index.html          the interactive shell
├── print.html          the printable version
├── manual.js           the step engine
├── manual-data.js      title, profile groups (mower / GNSS / LiDAR)
├── content/NN-*.js     one file per chapter — edit these
└── img/                diagrams (SVG) and photos
\`\`\`

Each step is an object with a \`title\`, a \`body\` in a small markdown subset, an optional \`media\` image, optional \`parts\`, and an optional \`when\` filter. Both languages sit side by side in the same object so they never drift apart. The folder's \`README.md\` documents the format.

To contribute:

1. Edit a file under \`docs/build/content/\` directly on GitHub (pencil icon) or in a clone.
2. Open the page locally to check it: \`cd docs && python3 -m http.server\`, then \`http://localhost:8000/build/\`.
3. Open a pull request against the \`dev\` branch.

Photos of your build, a chassis we have not documented, a fix that took you an evening: those are the most valuable contributions. Thank you.`,
        fr: `Le guide est constitué de fichiers simples dans le dépôt, sans étape de build :

\`\`\`
docs/build/
├── index.html          la coquille interactive
├── print.html          la version imprimable
├── manual.js           le moteur d'étapes
├── manual-data.js      titre, groupes de profil (tondeuse / GNSS / LiDAR)
├── content/NN-*.js     un fichier par chapitre — c'est ici qu'on édite
└── img/                schémas (SVG) et photos
\`\`\`

Chaque étape est un objet avec un \`title\`, un \`body\` dans un petit sous-ensemble de markdown, une image \`media\` optionnelle, des \`parts\` optionnelles et un filtre \`when\` optionnel. Les deux langues sont côte à côte dans le même objet pour ne jamais diverger. Le \`README.md\` du dossier documente le format.

Pour contribuer :

1. Modifiez un fichier sous \`docs/build/content/\` directement sur GitHub (icône crayon) ou dans un clone.
2. Ouvrez la page en local pour vérifier : \`cd docs && python3 -m http.server\`, puis \`http://localhost:8000/build/\`.
3. Ouvrez une pull request vers la branche \`dev\`.

Des photos de votre montage, un châssis non documenté, une correction qui vous a coûté une soirée : ce sont les contributions les plus précieuses. Merci.`,
      },
    },
  ],
});
