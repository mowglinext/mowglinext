# Unicore Runtime Profiles

Ces profils pilotent à la fois :

- les `LOG` activés sur le récepteur via `sensors/unicore/configure_receiver.sh`
- les diagnostics ROS 2 activés par `sensors/unicore/start_gps.sh`
- les backends ASCII / hybride / binaire exposés par le driver Unicore GNSS (`UM982Driver`)

Le profil par défaut est `normal`.

## Profils

| Profil | Usage recommandé | Sortie recommandée | Logs principaux | Logs avancés |
| --- | --- | --- | --- | --- |
| `normal` | runtime Nav2 / production légère | `ascii` | `PVTSLNA`, `GPGGA`, `BESTNAVA`, `GPHPR` (sortie souvent `$GNHPR`), `RTKSTATUSA`, `RTCMSTATUSA` | aucun |
| `debug` | essais terrain / Foxglove | `ascii` par défaut, `hybrid` sur demande | profil `normal` | `BESTSATA`, `SATSINFOA`, `AGCA`, `HWSTATUSA`, `JAMSTATUSA`, `FREQJAMSTATUSA`, `GSV` |
| `survey` | analyse GNSS avancée | `hybrid` | profil `debug`, mais plus lent | `OBSVMCMPB` optionnel, comparaisons ASCII/binaire |
| `high_precision` | précision max / tuning RTK | `hybrid` ou `binary` expérimental | profil `debug` | `CONFIG PVTALG MULTI`, `CONFIG RTCMDECAUTO ENABLE`, `CONFIG RTCMPHASERATE POSITIVE`, `CONFIG RTCMCLOCKOFFSET ENABLE`, `OBSVMCMPB` optionnel |

## Périodes par défaut

| Profil | `main` | `bestnav` | `diagnostic` | `satellite` | `rf` | `raw` |
| --- | --- | --- | --- | --- | --- | --- |
| `normal` | `0.2` | `0.2` | `1` | `1` | `1` | `5` |
| `debug` | `0.2` | `0.2` | `1` | `1` | `1` | `5` |
| `survey` | `1` | `1` | `1` | `2` | `2` | `5` |
| `high_precision` | `0.1` | `0.1` | `1` | `1` | `1` | `5` |

Notes :

- `normal` reste léger par défaut.
- `debug` reste en ASCII tant qu’on ne demande pas explicitement `UNICORE_OUTPUT_FORMAT=hybrid` ou `binary`.
- `OBSVMCMPB` reste désactivé par défaut. Il est destiné au `survey/debug` avancé, pas au runtime Nav2.
- en `hybrid`, les logs Unicore qui ont une variante binaire sortent en `A+B`; en `binary`, ils sortent en `B` uniquement.
- la génération des commandes suit maintenant une table N4 par message: `LOG ... ONTIME` pour `GPGGA/PVTSLN`, période directe pour `BESTNAV/RTKSTATUS/GPHPR`, et `ONCHANGED` pour `RTCMSTATUS/GPHPR2`.
- en `binary`, `start_gps.sh` force les consommateurs ROS binaires nécessaires pour éviter des diagnostics stale ou un `NavSatFix` vide.
- Les résumés de diagnostics `GPS: raw observations` n’existent que si `UNICORE_ENABLE_RAW_OBSERVATIONS=true` et `UNICORE_OUTPUT_FORMAT=hybrid` ou `binary`.
- `high_precision` ajoute des commandes dépendantes du firmware N4 ; elles sont volontairement limitées à ce profil.

## Mapping `.env` vers ROS

`start_gps.sh` mappe directement ces variables vers les paramètres du nœud `unicore_node` :

- `UNICORE_ENABLE_UNICORE_BINARY` -> `enable_unicore_binary`
- `UNICORE_USE_BINARY_NAV` -> `use_binary_nav`
- `UNICORE_USE_BINARY_SATELLITE_DIAG` -> `use_binary_satellite_diag`
- `UNICORE_USE_BINARY_RTCM_DIAG` -> `use_binary_rtcm_diag`
- `UNICORE_USE_BINARY_RTK_DIAG` -> `use_binary_rtk_diag`
- `UNICORE_USE_BINARY_RF_DIAG` -> `use_binary_rf_diag`
- `UNICORE_USE_BINARY_HW_DIAG` -> `use_binary_hw_diag`
- `UNICORE_USE_BINARY_JAMMING_DIAG` -> `use_binary_jamming_diag`
- `UNICORE_ENABLE_RAW_OBSERVATION_DIAG` -> `enable_raw_observation_diag`
- `UNICORE_USE_BINARY_RAW_OBSERVATIONS` -> `use_binary_raw_observations`

Par défaut, ces variables sont résolues depuis le couple `UNICORE_PROFILE` + `UNICORE_OUTPUT_FORMAT`, puis peuvent être surchargées explicitement dans le `.env`.

## Variables `.env`

Exemple recommandé pour MowgliNext en runtime standard :

```env
UNICORE_PROFILE=normal
UNICORE_TARGET_BAUD=921600
UNICORE_MAIN_LOG_PERIOD=0.2
UNICORE_BESTNAV_LOG_PERIOD=0.2
UNICORE_DIAGNOSTIC_LOG_PERIOD=1
UNICORE_SATELLITE_LOG_PERIOD=1
UNICORE_RF_LOG_PERIOD=1
UNICORE_RAW_LOG_PERIOD=5
UNICORE_OUTPUT_FORMAT=ascii
UNICORE_ENABLE_SATELLITES=false
UNICORE_ENABLE_RF=false
UNICORE_ENABLE_JAMMING=false
UNICORE_ENABLE_RAW_OBSERVATIONS=false
```

Exemple terrain / debug Foxglove :

```env
UNICORE_PROFILE=debug
UNICORE_OUTPUT_FORMAT=ascii
UNICORE_ENABLE_SATELLITES=true
UNICORE_ENABLE_RF=true
UNICORE_ENABLE_JAMMING=true
```

Exemple précision max :

```env
UNICORE_PROFILE=high_precision
UNICORE_OUTPUT_FORMAT=hybrid
UNICORE_MAIN_LOG_PERIOD=0.1
UNICORE_BESTNAV_LOG_PERIOD=0.1
UNICORE_ENABLE_SATELLITES=true
UNICORE_ENABLE_RF=true
UNICORE_ENABLE_JAMMING=true
```

Exemple `survey` avec observations compressées binaires :

```env
UNICORE_PROFILE=survey
UNICORE_OUTPUT_FORMAT=hybrid
UNICORE_ENABLE_RAW_OBSERVATIONS=true
UNICORE_RAW_LOG_PERIOD=5
```

Exemple `binary` expérimental :

```env
UNICORE_PROFILE=high_precision
UNICORE_OUTPUT_FORMAT=binary
UNICORE_USE_BINARY_NAV=true
UNICORE_USE_BINARY_SATELLITE_DIAG=true
UNICORE_USE_BINARY_RTCM_DIAG=true
UNICORE_USE_BINARY_RTK_DIAG=true
UNICORE_USE_BINARY_RF_DIAG=true
UNICORE_USE_BINARY_HW_DIAG=true
UNICORE_USE_BINARY_JAMMING_DIAG=true
```

## Recommandations

- `normal` : recommandé pour le runtime Nav2 quotidien
- `debug` : recommandé pour les essais terrain et l’analyse Foxglove en gardant un backend lisible
- `survey` : recommandé pour l’analyse GNSS avancée, les captures ponctuelles et `OBSVMCMPB`
- `high_precision` : recommandé pour les campagnes RTK exigeantes après validation firmware, idéalement d’abord en `hybrid`
