# Diagnostics — présentation calme et délai de réception de 60 secondes

## Référence et périmètre

Branche `fix/gui-warnings-errors`, HEAD de départ
`86132aeca1b3975ca1f4f596d8d54cb75d88cfc9` : hotfix RTK committé.
Référence : 2 396 tests Vitest, 105 warnings ESLint, zéro erreur.
L’index était vide ; seule la modification MAVROS indépendante préexistait.
Aucun sous-module, backend, producteur ROS, protocole ou commande n’est modifié.
Aucune connexion robot, écriture de configuration ou action matérielle n’a été
exécutée pour cette correction. Aucun staging, commit ou push.

## Défaut reproduit et décision de présentation

Le test rouge du composant réel montre qu’un rapport projeté comme périmé produit
immédiatement un groupe d’alerte « État non vérifié », même pour une interruption
de dix secondes. Log : `/tmp/gui-quiet-diagnostics-red.log`.

La rétention visuelle du lot 27 est supprimée : ni épisode résolu/non vérifié,
ni minimum de trente secondes, ni bouton d’historique ou compteur. Ses anciens
tests d’acceptation sont remplacés par ceux de la nouvelle règle ; leur retrait
n’est pas présenté comme une augmentation de couverture. Le composant de suivi reste monté, mais sa carte est absente quand aucune alerte
n’est visible, comme dans la page d’origine ; ses lignes contiennent uniquement des défauts
courants et des pertes de réception ayant atteint le délai.

- Avant 60 secondes continues sans données admissibles : aucune alerte de
  réception, même si le contrôle interne déclare déjà la télémétrie indisponible.
- À 60 secondes : une ligne par source requise, avec un texte FR/EN constant.
- Après 60 secondes : même clé et même ligne ; aucun compteur d’âge visible.
- Une publication admissible rétablie retire la ligne immédiatement, sans OK,
  message « Résolue » ou conservation de son contenu en historique.
- Les WARN/ERROR courants qui décrivent un défaut réel restent immédiats.
  Une urgence ou autre conclusion critique reste immédiatement visible dans le
  résumé. Une perte de réception ne bénéficie d’aucun pouvoir d’acquittement.

Le résumé retrouve les six badges essentiels d’origine (conteneurs, GNSS, batterie,
urgence, température et utilisation CPU), sans décliner tous les checks en
badges OK/inconnu/périmé. Les
conclusions internes OK et inconnue ont la même présentation neutre « — », sans
verdict de bonne santé inventé. Les conclusions warning/critical restent visibles.
Le fix GNSS, ses valeurs et ses catégories sont conservés. Les mentions génériques
de fraîcheur sont retirées du résumé, de MowerStatus et de l’accueil. Les badges
techniques de fraîcheur Fusion/COG/magnétomètre utilisent « — » quand indisponibles.
Les interruptions temporaires des lectures snapshot/calibration restent silencieuses :
le snapshot requis suit le compteur système, tandis qu’une ressource de calibration
facultative absente n’établit pas un défaut matériel. Les véritables erreurs de
lecture ou de traitement restent indiquées localement dans leur section.
Les erreurs d’actions explicitement demandées et les résultats de calibration
ne sont pas modifiés.
Les vrais messages de sécurité (notamment une urgence non acquittée) restent ;
ils ne sont pas remplacés par un message rassurant.
La section Diagnostics ROS détaillée conserve niveaux, rapports et données utiles.

## Compteurs indépendants et admission

`diagnosticAlertPresentation.ts` ne contient que l’état de réception visuel.
Chaque source garde son dernier jeton de progression et l’instant monotone
correspondant. Le délai part de cette dernière donnée, **pas** de l’expiration
interne à trente secondes. Sans première donnée, il part de l’entrée de la source
requise dans ce panneau. La minuterie déjà présente dans Diagnostics suffit ;
aucune minuterie ou temporisation n’est attachée aux publications ou au rendu.

Les jetons réutilisent exclusivement les métadonnées des frontières existantes :
publication/progression admise des hooks, horodatage de réception du snapshot
validé, et progression de chaque couple nom/hardware_id des diagnostics. Un
replay identifiable, une régression, une trame rejetée ou une simple reconnexion
ne renouvellent pas le compteur. Une autre source ne peut pas le renouveler.
Les âges ROS diagnostics, enregistrés en horloge navigateur murale, sont convertis
en durée avant d’ancrer le délai monotone ; aucun timestamp producteur ROS n’est
comparé directement à `performance.now()`.

Sources surveillées par cette présentation :

| Source | Condition d’exigence | Admission réutilisée |
| --- | --- | --- |
| État du robot | Toujours | HighLevelStatus décodé, politique de réception du contrat existant |
| État d’urgence | Toujours | Emergency validé et non bloqué |
| Supervision système | Toujours | Snapshot validé avec progression, sans erreur de requête |
| Liaison matérielle | Backend confirmé mowgli, mavros ou openmower : rapport commun Hardware Bridge / mowgli/hardware_bridge | Rapport admissible qui ne signale pas seulement une absence |
| Odométrie | mowgli/openmower ; MAVROS uniquement après déclaration explicite required=true | Même règle, compteur distinct |
| LiDAR | Uniquement lidar_enabled=true | Rapport LiDAR / mowgli/lidar admissible |
| GNSS | gnss_stack=universal ou backend matériel MAVROS confirmé | Métadonnées du hook GNSS existant, affichage inchangé |
| Localisation Fusion | Mode autonome explicitement établi | Progression Fusion non bloquée |

Une perte du contexte de mode ne supprime pas une exigence Fusion déjà établie :
seul un autre mode opérationnel explicite la retire. Cela empêche un compteur de
repartir à zéro sur trame invalide ou déconnexion. Les sources réellement retirées
par configuration/mode sont supprimées du modèle ; celui-ci ne collecte aucun
historique de rapports et reste borné par les exigences du panneau.

`unavailableReport()` réutilise la distinction existante entre absence et défaut.
Il reconnaît aussi le WARN exact `missing_or_stale` du rapport MAVROS
`mowgli_mavros_bridge/wheel_odometry_source`, hardware_id vide, avec un champ
`required` booléen textuel valide. Ce message décrit une disponibilité, pas un
défaut mesuré. Un ERROR ou un autre message conserve son traitement immédiat.
Les différents champs d’un même capteur ne créent pas de compteurs supplémentaires.

### Réserves de revue : contrats des backends et déconnexion globale

Le nœud natif est réservé à mowgli (`mowgli_bringup/launch/mowgli.launch.py`).
Cependant, `diagnostics_node` est lancé pour les trois backends dans
`full_system.launch.py` et surveille le contrat commun `/hardware_bridge/status`.
Son rapport `Hardware Bridge / mowgli/hardware_bridge` n’est donc pas une preuve
de présence du nœud natif et reste requis pour les trois backends confirmés.
L’état backend inconnu ne crée pas d’exigence matérielle supposée.

`diagnostics_node.cpp` surveille aussi `/wheel_odom`. Le producteur natif et
le bridge OpenMower fournissent ce flux ; le bridge OpenMower le remappe dans
`launch/openmower_bridge.launch.py`. MAVROS ne le garantit pas systématiquement.
Le consommateur lit la déclaration `required=true/false` du rapport exact ci-dessus.
Une déclaration admise est conservée pendant une perte de contexte ; une nouvelle
déclaration explicite peut la changer. Sans déclaration MAVROS, la GUI ne suppose
pas l’odométrie obligatoire. Un passage à true crée son compteur de 60 secondes.

Cette intégration a été comparée aux contrats du dépôt principal et, en lecture
seule, au checkout MAVROS local `5328dd70d1fce34ee9e0c81ff3883b0260a712de`.
Le gitlink MAVROS épinglé est différent ; la présence de cette déclaration n’est
pas attribuée à une image déployée ou à ce gitlink sans preuve. Les producteurs
anciens sans déclaration ne permettent pas d’établir cette exigence optionnelle.
Aucun fichier de producteur ou sous-module n’est modifié.

Pendant une déconnexion globale, les compteurs individuels continuent d’avancer,
mais leurs alertes de réception sont regroupées en une indication de communication
après 60 secondes. Ni la réouverture du socket ni un polling HTTP réussi ne
prouvent la reprise ROS : une progression admissible d’une source ROS est nécessaire.
À cette reprise, les sources toujours manquantes retrouvent leur alerte selon leur
compteur conservé. Aucun message OK/Résolue n’est ajouté. Les défauts courants et
les urgences restent indépendants du regroupement et immédiatement visibles.

## Invariants et limites

Le calcul `deriveDiagnosticHealth`, les seuils internes, l’admission GNSS et ses
barrières par source, le décodage, le contrôle de fraîcheur, les bits d’urgence et
les autorisations de commande restent inchangés. Les délais visuels ne sont jamais
réinjectés dans ces fonctions. Le module de santé ne change que la classification de présentation du WARN
MAVROS d’absence exactement identifié ci-dessus et l’export des prédicats réutilisés.

HighLevelStatus ne possède ni timestamp ni séquence dans son contrat ROS. Sa
politique existante admet la réception d’une publication structurellement valide ;
une valeur identique peut être une publication légitime. Aucun timestamp producteur,
identité d’acquisition ou hash d’état n’est fabriqué. Un replay dépourvu d’identité
n’est donc pas distinguable d’une telle publication par ce contrat. Les garanties
anti-rejeu identifiables restent celles des frontières existantes, sans prétendre
résoudre cette limite par le délai de 60 secondes. Une reconnexion seule ne suffit
jamais à renouveler le compteur.

Les compteurs vivent pendant le montage de la page, sans stockage intersession.
Les compteurs Hardware Bridge/odométrie sont liés au contexte backend ; les autres
sources ne sont pas réinitialisées par son changement.
Ils ne constituent ni un moniteur ROS exhaustif ni une nouvelle preuve matérielle.
Le panneau disparaît seulement par navigation ; un défaut résolu retire sa ligne,
même si son groupe était ouvert, conformément à la nouvelle demande sans historique.
Un changement réel de défaut peut donc encore changer la hauteur de son groupe.
L’objectif testé concerne les changements incessants liés à la réception normale.

## Tests et validations

Les résultats ci-dessous précèdent les deux dernières réserves ; la validation
de leur correction est consignée dans la section suivante.

- Tests ciblés finaux : **151 réussis dans huit fichiers**, 4,20 secondes ;
  composant d’alertes, résumé, modèle pur, santé interne, MowerStatus, locales,
  hooks GNSS et télémétrie d’état. Log `/tmp/gui-quiet-focused-final.log`.
- Chromium final après retour à la disposition d’origine : **16 réussis sans
  retry**, 33,1 secondes, quatre parcours de réception, huit scénarios de santé
  et quatre du hotfix GNSS. Log `/tmp/gui-quiet-original-browser-final.log`.
- TypeScript `yarn tsc --noEmit` et build `yarn build --outDir /tmp/gui-quiet-dist` :
  réussis, build 10,53 secondes. Logs `/tmp/gui-quiet-tsc-final.log` et
  `/tmp/gui-quiet-original-build.log`.
- `yarn lint` et inventaire JSON complet `eslint src -f json` : réussis,
  **105 → 105 warnings, zéro erreur**, aucun warning dans les fichiers modifiés.
- Suite complète finale `yarn test --pool=threads --maxWorkers=2` : **2 382 tests
  réussis dans 197 fichiers**, 238,65 secondes, plus le test du registre
  d’apparences. Log `/tmp/gui-quiet-original-full.log`. Le total net baisse de
  14 par rapport aux 2 396 du hotfix : les tests du contrat d’historique abandonné
  ont été retirés/remplacés, aucun test n’est masqué ou ignoré pour faire passer
  la suite.
- `git diff --check`, contrôles des nouveaux fichiers et JSON : réussis.
  Aucun fichier de formatage/configuration/dépendance modifié.
- Aucun test Go/ROS/Python : aucune modification de ces composants.
- La première suite complète a chevauché l’ajout de la régression sur l’exigence
  autonome et a chargé deux versions différentes du helper et de son test :
  une erreur d’export, 2 381 autres tests réussis. Ce passage intermédiaire n’est
  pas une validation du diff final. La suite d’acceptation est relancée après
  les dernières suppressions d’alertes de polling.

Les tests purs et du composant vérifient 10/30/59 secondes sans alerte, une seule
ligne stable à 60/61/70/90 secondes, la reprise, les replays/régressions/données
invalides, l’indépendance des sources, les coupures de 5 à 20 secondes et le retrait
des sources optionnelles. Ils couvrent aussi la conservation de l’exigence autonome
pendant la perte du contexte. Les défauts réels et urgences ne passent pas par le
modèle de délai.

Chromium utilise la vraie page et les vrais composants avec toutes les API et
WebSockets simulés, en FR/EN et 390/1440 px. Le scénario de réception laisse une
source requise silencieuse tout en renouvelant les autres ; il contrôle le nombre,
le texte et la clé de la ligne, puis la reprise et plusieurs minutes de coupures
courtes. Il mesure la stabilité de la position des sections avant le délai, vérifie un défaut moteur
et une urgence immédiats, l’absence de mots d’historique/fraîcheur et de toute
écriture implicite. Les scénarios de santé et du hotfix RTK sont relancés aussi.

Le premier harnais figeait l’horloge avant le démarrage de React, empêchant le
montage ; il la fige maintenant après montage. Une deuxième tentative rejouait
chaque image d’animation pendant plusieurs minutes virtuelles et dépassait les
30 secondes du test. Les longs intervalles utilisent désormais les sauts d’horloge
Playwright ; les timers de rendu/sécurité et les timeouts de test sont inchangés.
Après retour à la carte conditionnelle d’origine, le sélecteur du défaut moteur
a été limité à la carte Alertes : la section ROS détaillée possède légitimement
un second groupe du même nom sur desktop.
Aucun retry, sommeil arbitraire, console masquée ou modification applicative n’a
servi à contourner ces deux problèmes de harnais.

Validation matérielle de l’ergonomie : **HARDWARE_REQUIRED**, distincte des tests
simulés. Après déploiement séparément autorisé, enregistrer commit/digests GUI/ROS,
robot, image firmware, gitlinks MAVROS/Universal GNSS et versions des producteurs ; robot immobile, lames désactivées et mouvement
empêché. Observer plusieurs minutes de flux normal : pas de bruit de fraîcheur,
pas de groupe qui apparaît/disparaît sur une courte interruption ; vérifier les
interruptions naturelles et leur reprise sans provoquer de défaut matériel. Aucun
déploiement ou redémarrage n’est inclus dans ce lot.

Diff complet : `/tmp/gui-diagnostics-quiet-final.diff`, nouveaux fichiers inclus,
MAVROS exclu. Index vide ; attendre la revue indépendante et le GO COMMIT.

## Fichiers de revue

- `src/components/diagnostics/DiagnosticAlertsPanel.tsx` et son test : panneau courant sans historique.
- `src/components/diagnostics/DiagnosticHealthSummary.tsx` et son test : résumé sans bruit OK/inconnu.
- `src/utils/diagnosticAlertPresentation.ts` et son test (nouveaux) : compteurs indépendants de réception.
- `src/utils/diagnosticAlertHistory.ts` et son test (supprimés) : ancien modèle de rétention abandonné.
- `src/utils/diagnosticHealth.ts` et son test : distinction absence/défaut du rapport MAVROS et lecture de son exigence.
- `src/pages/DiagnosticsPage.tsx` : admission des sources et nettoyage de la présentation.
- `src/components/MowerStatus.tsx` et son test : suppression de la mention générique.
- `src/pages/MowgliNextPage.tsx` : retrait des mêmes mentions d’accueil.
- `src/i18n/locales/en.json`, `fr.json` : texte simple des alertes de réception.
- `tests/e2e/diagnostics-alert-history.spec.ts` : scénarios remplacés par la nouvelle règle.
- `tests/e2e/diagnostics-health.spec.ts` : attentes de présentation adaptées, contrôles critiques conservés.
- `tests/e2e/diagnostics-backend-reception.spec.ts` : trois backends, exigence MAVROS et déconnexion/reprise sur la page réelle.
- Ce rapport.

Ces chemins sont relatifs à `gui/web`. Les rapports antérieurs restent des
traces historiques, pas une description de la nouvelle politique de présentation.

## Comparaison avec la page antérieure aux lots 25–27

Référence Git inspectée : `00122e4f` (parent du lot 25 `ffcb1ba2`).
Les changements du lot 26 `9105cb61` et du lot 27 `87003b0b` ont été comparés
séparément ; le hotfix courant `86132aec` ne changeait que la classification GNSS.

| Élément | Avant le lot 25 | Lots 26/27 | Patch révisé |
| --- | --- | --- | --- |
| Grand résumé | Titre, verdict, sous-titre | Cause détaillée et mentions de fraîcheur | Même place, taille de titre/valeur d’origine ; défaut courant uniquement, aucune mention de fraîcheur |
| Bandeau | Six informations essentielles | Liste élargie de checks et niveaux techniques | Six informations d’origine, sans suffixe de niveau technique |
| Alertes | Carte conditionnelle, groupes repliables, nom/message | Carte permanente, épisodes, états et historique | Carte conditionnelle et groupes simples comme à l’origine ; délai uniquement pour absence de réception |
| Sections | Onglets desktop, sections repliables mobile | Même organisation | Noms, ordre, paramètres d’URL, ouverture et contenu essentiel conservés |
| Diagnostics ROS | Tous les rapports techniques dans une section dédiée | Conservés | Conservés ; aucun transfert vers une liste complète dans les alertes |
| GNSS | Décodage/classification antérieurs | Admission par provenance et identités | Admission conservée et hotfix 3D + Fixed/Float intact |

Différences fonctionnelles restantes, intentionnelles : délai de réception de
60 secondes, retrait des messages génériques et de l’historique, disparition
d’une alerte à la reprise. Les valeurs manquantes ne redeviennent pas zéro et
une liste de conteneurs absente ne redevient pas une preuve de bonne santé. Le
verdict sans défaut visible est neutre « — » pour ne pas alterner entre succès
et absence de preuve ; ce n’est pas une conclusion interne OK. Les contrôles
critiques, données admises, messages d’urgence non acquittée et états de sécurité
restent ceux du lot 26. Les couleurs neutres remplacent les anciens badges verts
non justifiés. Aucun seuil ni action ne revient à sa version historique.

L’état de présentation se limite au dernier instant admissible par source,
à l’instant de déconnexion globale et aux dernières exigences explicites
(autonomie et odométrie MAVROS) conservées pendant une perte de contexte. Il n’existe aucun automate d’épisodes, journal de réception ou
nouvelle architecture de page. Le helper de présentation du lot 26 est réutilisé
pour rendre l’ancien duo résumé/bandeau, avec les mêmes sections de page.

Inventaire ESLint avant la dernière correction ciblée : `no-explicit-any` 35 ; `no-unsafe-assignment` 23 ;
`no-unsafe-member-access` 12 ; `no-unsafe-argument` 8 ; `no-unsafe-call` 3 ;
`no-unsafe-return` 3 ; `react-refresh/only-export-components` 11 ;
`react-hooks/set-state-in-effect` 8 ; `react-hooks/purity` 2.
Les avertissements existants de build (gros chunks), Ant Design/React 19 et
Descriptions, ainsi que les notices jsdom de pseudo-éléments, ne sont pas masqués
et restent distincts des résultats ESLint.

État final : HEAD inchangé `86132aeca1b3975ca1f4f596d8d54cb75d88cfc9`, index
vide. MAVROS reste à la révision locale préexistante
`5328dd70d1fce34ee9e0c81ff3883b0260a712de` ; le gitlink du dépôt est inchangé.
Universal GNSS reste à `9a329cc216ed137d0e375a4cfe1f5c26e1dd52d0`.
Aucun staging, commit, push, reset, stash, clean ou déploiement.


## Validation des deux dernières réserves

Les tests Chromium ont réellement été exécutés après les corrections de revue :

- Nouveaux parcours de la page réelle : **10 réussis**, sans retry, 31,6 secondes
  (`/tmp/gui-quiet-review-browser-final.log`). Les trois backends sont couverts
  en FR/EN ; les déconnexions/reconnexions MAVROS en 390/1440 px vérifient une
  seule indication, l’urgence conservée et le compteur odométrie non réinitialisé.
- Régressions réception/santé/GNSS Fixed : **16 réussies**, sans retry,
  31,5 secondes (`/tmp/gui-quiet-review-regression.log`). Total : **26**.
- Suite complète : **2 388 tests réussis dans 197 fichiers**, 230,93 secondes,
  plus le test du registre d’apparences (`/tmp/gui-quiet-review-full.log`).
  Aucun test ignoré pour ce lot. Le total est supérieur de six au passage
  précédent (2 382) et inférieur de huit au baseline RTK (2 396), du fait du
  retrait/remplacement intentionnel des tests d’historique.
- Tests ciblés de revue : **145 réussis dans sept fichiers**, 3,16 secondes
  (`/tmp/gui-quiet-review-focused.log`), avant correction du seul harnais E2E.
- TypeScript : réussi (`/tmp/gui-quiet-review-tsc-final.log`).
- Build : réussi, 10,22 secondes (`/tmp/gui-quiet-review-build.log`).
- ESLint : **105 warnings, zéro erreur**, aucun diagnostic dans les sources
  modifiées (`/tmp/gui-quiet-review-lint.log`, inventaire
  `/tmp/gui-quiet-review-eslint.json`). Les règles et nombres restent identiques.

Le test rouge de regroupement présentait deux lignes au lieu d’une
(`/tmp/gui-quiet-review-red.log`). Le premier passage navigateur des nouveaux
parcours a donné six réussites et quatre échecs à la reprise : après les sauts
virtuels, le watchdog existant pouvait avoir retiré le socket simulé. Le harnais
attend maintenant le timer de reconnexion existant et la souscription effective
avant d’envoyer la reprise. Le deuxième passage ci-dessus est vert, sans modifier
le watchdog, les hooks, les timeouts applicatifs ni les producteurs.

`git diff --check`, espaces des nouveaux fichiers et parsing JSON : réussis.
Le hotfix et les frontières de télémétrie GNSS ne présentent aucun diff par
rapport au HEAD de départ. Index vide, HEAD et gitlinks inchangés ; la modification
locale MAVROS indépendante est conservée et exclue du diff de revue.
La validation physique reste **HARDWARE_REQUIRED**, selon la procédure ci-dessus.


## Dernière correction ciblée avant gel de DiagnosticsPage

### Changement de backend sans remontage

Le hook `useHardwareBackend` actuel effectue une lecture au montage ; il ne possède
pas de mécanisme de changement à chaud. Cependant, la valeur MAVROS mémorisée dans
la page n’était pas explicitement limitée à son contexte. La mémoire est maintenant
associée au backend : tout changement réel réinitialise cette exigence et uniquement
les compteurs Hardware Bridge/odométrie concernés. Les autres sources continuent.

À l’entrée d’un nouveau contexte, la page conserve les jetons de progression déjà
connus des trois rapports concernés. Seul un jeton ultérieur peut apporter une
nouvelle donnée ou déclaration dans ce contexte ; un rapport accumulé en cache
ne suffit pas. Ce contrôle compare les métadonnées navigateur existantes, sans
fabriquer d’identité producteur ni dépendre d’un délai arbitraire.

Le test Chromium de la vraie page injecte uniquement une mise à jour du résultat
du hook backend (la production ne la propose pas actuellement), sans remontage.
Il traverse MAVROS → OpenMower → MAVROS → mowgli → MAVROS, vérifie l’abandon
d’une déclaration précédente/cachée et l’admission d’une nouvelle déclaration.
Un témoin DOM prouve le maintien du montage ; le compteur état du robot arrive
à échéance pendant ces changements, ce qui vérifie son absence de remise à zéro.

### Lectures : indisponibilité ou erreur utile

- Rejet réseau, HTTP 408/429/502/503/504 : pas d’alerte technique immédiate.
  Le snapshot requis conserve son compteur de réception à 60 secondes.
- HTTP 404/405 : ressource absente/non prise en charge, pas de fausse alerte
  de calibration facultative. Le snapshot système reste une source requise.
- Échec explicite de lecture serveur (notamment HTTP 500), accès refusé,
  JSON illisible, enveloppe invalide ou snapshot régressif : une indication
  locale persistante dans la section concernée, sans notification répétée.
- Les erreurs propres aux artefacts `dock/imu/mag.error` étaient déjà visibles
  dans leurs cartes ; ce comportement est démontré et conservé.
- Une lecture valide retire l’indication, sans OK/Résolue ni historique.

Les hooks exposent un champ `readError` destiné à cette distinction. Le snapshot
conserve son décodeur, sa progression, sa protection contre les réponses anciennes
et son absence de polling chevauché. Le hook calibration vérifie l’enveloppe
`dock/imu/mag` avant son utilisation. Les titres FR/EN déjà présents sont réutilisés.
Ni les producteurs, ni les protocoles, ni les commandes ou sécurités ne changent.
Aucune nouvelle disposition de page n’est introduite.

Fichiers supplémentaires concernés : `hooks/useDiagnosticsSnapshot.ts` et son
test, `hooks/useCalibrationStatus.ts` et son nouveau test, `utils/errors.ts`.
Les tests de page sont dans les deux specs backend/réception et santé existantes.


### Validation finale de cette correction

- Tests ciblés sur le code figé : **140 réussis dans sept fichiers**,
  4,41 secondes (`/tmp/gui-diagnostics-freeze-focused.log`).
- Chromium : **29 réussis sans retry**, 86,77 secondes au total de commande
  (`/tmp/gui-diagnostics-freeze-browser.log`). Les trois nouveaux parcours
  couvrent le changement de backend sans remontage et les erreurs de lecture
  FR/EN, en plus des 26 régressions précédentes.
- TypeScript : réussi (`/tmp/gui-diagnostics-freeze-tsc.log`).
- Build : réussi, 11,08 secondes (`/tmp/gui-quiet-freeze-build.log`), sur les
  sources finales, sorties dans `/tmp/gui-quiet-freeze-dist`.

Un passage intermédiaire de Chromium a chevauché la dernière correction de la
barrière de contexte et a donné 28 réussites/1 échec MAVROS ; il ne valide pas
la version figée. La relance complète ci-dessus passe sans retry.
Un contrôle lint intermédiaire a aussi rejeté le lancement direct d’un objet
HTTP comme exception : les deux hooks construisent désormais un Error portant
le statut utile à la classification. Aucune règle n’est désactivée.

- ESLint final : **104 warnings, zéro erreur**, aucun diagnostic dans les sources
  modifiées (`/tmp/gui-diagnostics-freeze-lint.log`, inventaire
  `/tmp/gui-diagnostics-freeze-eslint.json`). Le warning
  `react-hooks/set-state-in-effect` de `useCalibrationStatus.ts` ne figure plus
  après la correction de lecture ; les règles, seuils et suppressions restent
  inchangés. Ce passage n’est pas un nouveau lot de nettoyage ESLint.

Le gel de DiagnosticsPage est proposé après cette revue. Aucun nettoyage des
104 avertissements restants n’a été entrepris dans cette correction. Les lots
suivants devront conserver le comportement fonctionnel et ne pas rouvrir la
présentation de cette page. Aucun commit, push ou déploiement n’est effectué.


Suite complète finale, exécutée après gel des sources et tests : **2 392 tests
réussis dans 198 fichiers**, 228,85 secondes, plus le test du registre
d’apparences (`/tmp/gui-diagnostics-freeze-full.log`). Le premier passage complet
avait aussi réussi (2 392), mais chevauchait les derniers ajustements ; seule la
relance sur la version figée constitue la validation finale. Aucun test ignoré
pour cette correction. Le total augmente de quatre par rapport aux 2 388 de la
revue précédente : isolation des compteurs, lecture calibration et deux cas de
lecture snapshot. Aucun seuil de sécurité ou de fraîcheur GNSS n’est modifié.

Contrôles finaux : `git diff --check`, espaces des cinq nouveaux fichiers,
parsing JSON, HEAD inchangé et index vide. Le diff complet
`/tmp/gui-diagnostics-quiet-final.diff` inclut les nouveaux fichiers et exclut le
sous-module MAVROS indépendant. La réserve matérielle reste celle explicitement
décrite plus haut ; aucune observation ou action robot n’a été réalisée.
