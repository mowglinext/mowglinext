# GUI Lot 21 bis — Mesures des champs numériques en modale

## Préflight et périmètre

Branche `fix/gui-warnings-errors`, HEAD
`5ddf1d1f5a7da31fe48486348f253a68d6a1213b` : lot 21 committé avant ce correctif.
Index vide ; seul le sous-module MAVROS avait une modification préexistante.

**Cas A confirmé : correction du test seul.** `NumberWithAddon.tsx` et la fixture
`number-addons.tsx` restent inchangés. Aucun changement de CSS, de dépendance,
de valeur, de callback, de précision, de sauvegarde ou de paramètre robot.

Fichiers modifiés :

- `tests/e2e/inputnumber-addons.spec.ts`
- `docs/gui-lot21b-number-addon-modal-layout.md`

## Cause et instrumentation

Le test attendait seulement que la valeur initiale soit affichée. Cette condition
peut être satisfaite avant la fin du zoom d'entrée Ant Design. Or
getBoundingClientRect mesure la géométrie **transformée**, contrairement à
clientWidth/scrollWidth, qui décrivent les dimensions de mise en page.

Le premier passage sans modification à ce HEAD a réussi aux deux largeurs
(2/2). Cela ne contredit pas les 4/4 échecs du lot 21 : le résultat dépend de
l'instant auquel le navigateur prend la mesure. Aucune réussite initiale n'a
été utilisée pour écarter le problème.

Une instrumentation temporaire a relevé les dimensions des groupes, champs,
addons et corps de modale, leurs hauteurs, les débordements et la transformation
CSS. Elle a d'abord observé la phase appear-start, où la transformation peut
encore être none. Une seconde capture a attendu explicitement la phase active,
puis la disparition des classes d'entrée et de la transformation. Elle a établi
les mesures suivantes, sans désactiver l'animation ni ajouter de temporisation.
Les journaux de mesure sont conservés dans `/tmp/gui-lot21b-*-measure.log` ;
l'instrumentation et ses logs console sont retirés du test final.

| Mesure, en pixels | 390 px pendant zoom actif | 390 px stabilisé | 1440 px pendant zoom actif | 1440 px stabilisé |
| --- | ---: | ---: | ---: | ---: |
| Facteur de transformation | 0,2 | aucune transformation | 0,700321 | aucune transformation |
| Modale | 74,80 | 374 | 504,23 | 720 |
| Groupe fixe | 56 | **280** | 196,09 | **280** |
| Champ du groupe fixe | **25,94** | 129,70 | 90,83 | 129,70 |
| Addon avant du groupe fixe | 30,06 | 150,30 | 105,26 | 150,30 |
| Groupe fluide | 65,20 | 326 | 470,62 | 672 |
| Champ du groupe fluide | 32,46 | **162,28** | 355,96 | **508,28** |
| Addon long après le champ fluide | 32,94 | 164,72 | 115,36 | 164,72 |

Les captures actives portent les classes ant-zoom-appear / appear-active et les
matrices `matrix(0.2, 0, 0, 0.2, 0, 0)` et
`matrix(0.700321, 0, 0, 0.700321, 0, 0)`. Après stabilisation, transform vaut none.
Les 249–257 px observés au lot 21 correspondent à d'autres instants du zoom,
pas à un changement de la largeur CSS fixe de 280 px.

Le relevé détaillé attribue ici exactement les 25,94 px au **champ fixe**
(129,703125 × 0,2). L'ancienne assertion commune ne précisait pas quel champ
échouait ; une largeur finale insuffisante du champ fluide n'est pas reproduite.

Après animation :

- Le corps intérieur disponible mesure 326 px à 390 px, et 672 px à 1440 px.
  Le contenu a 24 px de padding à gauche et à droite.
- Les hauteurs champ/addon sont identiques : 32/32 px pour le groupe fixe,
  24/24 px pour le groupe fluide, avec les mêmes positions verticales.
- Les addons gardent leur texte complet et leur largeur intrinsèque ; leur
  scrollWidth n'excède pas leur clientWidth.
- Les groupes restent dans le corps de la modale et dans le viewport. La largeur
  défilante du document reste égale au viewport, 390 ou 1440 px.

Ces dimensions finales justifient de ne pas changer le composant partagé.

## Correction et assertions conservées

Le test attend que les classes ant-zoom-appear/enter disparaissent et que transform
soit none. Il vérifie à nouveau ces deux conditions **dans chaque relevé de
mesure**, afin de garantir que les assertions portent sur l'état final.
Il n'impose pas d'observer une animation active pour réussir : une ouverture déjà
terminée reste valide. Le relevé couvre l'ouverture réelle à chaque viewport.

Toutes les assertions précédentes restent présentes, notamment :

- groupe fixe proche de 280 px avec la même tolérance technique ;
- largeur numérique strictement supérieure à 30 px ;
- différence de hauteur inférieure à 2 px ;
- limites du viewport et de l'addon ;
- zéro initial affiché avec trois décimales, édition en 2,5, blur, puis sauvegarde
  Form.Item du résultat `{fixed: 2.5, fluid: 1}` ;
- absence de dépréciation InputNumber addonBefore/addonAfter.

Les contrôles supplémentaires portent sur la largeur du véritable élément input,
la visibilité et l'absence de rognage des unités, l'alignement vertical, les limites
du corps de modale et les débordements du groupe/document. Les assertions ne sont
ni affaiblies ni remplacées par des valeurs de largeur de style non rendues.

Aucun waitForTimeout, force, timeout global accru, retry ou désactivation
d'animation n'est ajouté. Les dimensions sont toujours lues avec
getBoundingClientRect après la fin observable de la transition.

## Validation

Exécution avec le Playwright installé, équivalent aux commandes npx demandées ;
E2E_PORT=5185 et les sorties `/tmp` isolent le serveur et les traces.

- Avant correction : deux scénarios de modale inchangés, **2/2 passent** sur ce
  passage ; l'instrumentation active démontre ensuite le mécanisme d'échec.
- Fichier Playwright complet, Chromium, retries=0, workers=1 : **4/4 passent**,
  6,9 s. Comprend les champs de capteurs à 390/1440 px et les deux modales.
- `--grep "fixed/fluid long addons" --repeat-each=10 --retries=0 --workers=1` :
  **20/20 passent**, dix ouvertures à chaque largeur, 29,5 s.
- Tests du composant NumberWithAddon : **11/11 passent**, incluant Form.Item,
  valeurs contrôlées, précision, événements, ref, validation et désactivation.
- `yarn tsc --noEmit` : passe. Le tsconfig principal couvre src ; les tests E2E
  sont transformés/exécutés par Playwright.
- `yarn lint` : passe, **105 warnings, zéro erreur**, inchangé.
- `yarn build --outDir /tmp/gui-lot21b-dist` : passe.
- `git diff --check` : passe, y compris le nouveau rapport ; index vide.

La suite Vitest complète et les autres tests Playwright Settings ne sont pas
relancés : leur exécution supplémentaire était conditionnée à une modification
réelle de NumberWithAddon, absente ici. Le lot 21 avait validé 2 073 tests Vitest ;
ce résultat précédent n'est pas présenté comme un nouveau passage du lot 21 bis.

## Limites

Mesures Chromium aux deux viewports demandés, sur ce commit et ces dépendances ;
pas de preuve universelle pour toute unité, langue, taille d'écran ou navigateur.
Le correctif garde justement les assertions géométriques pour détecter une future
régression de responsive réelle. Les notices de build concernant les gros chunks
restent hors périmètre. Les autres anomalies du lot 21 ne sont pas modifiées.

Tous les échanges robot des tests sont simulés ; la fixture sauvegarde dans son
état React local. Aucune commande ni écriture de configuration réelle.
Aucun staging, commit, push ou changement MAVROS.
