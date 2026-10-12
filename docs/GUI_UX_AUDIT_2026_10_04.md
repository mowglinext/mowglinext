# GUI UX audit remediation — 4 October 2026

This is a historical implementation record for the live GUI audit. The audit
covered nine main pages, twenty settings categories, and selected phone layouts.
Implementation started from repository commit
`bced049e55aea8ccf19f8f38ada97d7ec1c2140a`. The audited GUI was a different build,
`22e375a38eeb4ac7e3207278660d1c58c69afaf4`; observations are not evidence about
other firmware, receiver, or robot baselines.

The robot configuration and map were backed up before the live review. No mowing,
blade, calibration, or movement command was executed. These fixes were developed
and checked with a mocked backend, without deploying to or restarting the robot.
Private configuration, map backups, and live screenshots are not included here.

## Findings and resulting behavior

| ID | Priority | Finding | Result |
|---|---|---|---|
| F01 | High | Emergency action appeared to describe an already-active state. On phones, the same STOP control could release the latch. | STOP always invokes `emergency` with `Emergency: 1`. Release is a separate, explicitly named action. No confirmation delay was added to STOP. |
| F02 | High | Ambiguous motion labels and mixed map menus. | Start/return controls have visible labels; blade actions name the operation and direction. Map display, movement, blades, and files have separate menu groups. The redundant legacy S2/start alias was removed from the menu. |
| F03 | High | Phone STOP had insufficient contrast over satellite imagery. | An opaque danger surface and dark foreground apply in normal, hover, focus, and pressed states. STOP remains pinned outside the scrolling toolbar. |
| F04 | High | Planning advertised a hardcoded 20% return threshold. | Rules show the saved battery threshold and rain mode, or unknown when unavailable. Copy distinguishes saved settings from runtime application. |
| F05 | High | Hardware and sensor controls lacked accessible names. | Legacy fields associate labels and IDs; numeric fields expose units/help. Sensor controls and reset buttons have scoped names. Model cards support keyboard selection. |
| F06 | High | Completed runs used total sessions, and missing distance obscured activity. | Completed runs use `completed`. Unrecorded distance is explicit. Annual activity counts recorded sessions; weekly activity uses runtime, which exists for older sessions. Streaks exclude future calendar cells. |
| F07 | Medium | Unexpanded area interpolation, inconsistent numbering, misleading counts, and 0 m² obstacles. | Map list/canvas use localized one-based fallback labels; current area names resolve through live map indices. Names are displayed without conflicting suffixes. Mowing/navigation/obstacle counts are distinguished; small areas show `< 1 m²`. Commands still resolve stable IDs. |
| F08 | Medium | Search required technical parameter names. | Localized labels and tooltips are indexed alongside keys, with accent/case/separator normalization. Results can focus and highlight a field. |
| F09 | Medium | English application text appeared in French screens. | Dock calibration, session outcomes, charging, and standard Ant controls use the selected locale. Raw producer messages and logs retain their original text. |
| F10 | Medium | Model names and badges broke into fragments. | Badges have their own row and cards use fewer columns, including one column on phones. |
| F11 | Medium | Schedule creation immediately saved a disabled run without a durable state cue. | Creation explicitly says disabled, cards/grid distinguish inactive schedules, switches have names, and edits explain autosave. The safe disabled creation default is preserved. |
| F12 | Medium | Wizard/section changes retained the previous scroll position. | Context changes reset the actual content scroller and transfer focus to the new region/heading. The desktop settings rail scrolls independently. |
| F13 | Medium | Faint helper text and an invisible off-switch track. | Shared caption/description colors are stronger, forms use solid surfaces, unchecked switches have a visible track, and keyboard focus has an outline. |
| F14 | Medium | Negative diagnostic age and an initial “accepted” LiDAR verdict. | Browser delivery uses a monotonic clock sampled after receipt. Producer identity and ROS publication stamp are displayed separately; repeated stamps do not renew distinct-publication age. Missing identity remains unknown. No verdict/score/spread is claimed before a candidate exists. |
| F15 | Medium | Every settings section suggested a ROS restart and most preview rails were empty. | Browser-only appearance has no robot save/restart footer. Confirmed live drive writes do not create a restart reminder; failed live application does. Existing reminders survive navigation and clear after successful restart completion. Only sections with real previews reserve a preview column. |
| F16 | Low | Rail labels and build track text clipped or split mid-word. | Short rail labels, full-label tooltips, and ellipsis replace word fragments. |
| F17 | Low | UTF-8 log text displayed as `Âµs`. | Dedicated text streams decode base64 bytes with a per-connection streaming UTF-8 decoder, including split multibyte characters. |
| F18 | Medium | Help led with implementation names and obsolete navigation/config references. | Localization and drive tuning lead with operator consequences. Technical detail is disclosed separately, links use real localized destinations, and obsolete dock-yaw guidance is removed. |

## Verification and limits

`gui/web/tests/e2e/ux-audit.spec.ts` exercises French desktop (1440×900) and phone
(390×844) views, all twenty settings categories, accessible numeric/switch names,
search/focus, inactive schedules, the saved 15% threshold, a 55-session/18-completed
history with missing odometry, wizard scroll restoration, LiDAR before its first
candidate, and successful/failed live parameter application. STOP's request payload
is checked in idle, latched, and offline scenarios. Every API and socket is mocked;
the test dev-server fallback points at loopback port 9.

Unit regressions cover publication identity versus delivery, independent robot and
browser clocks, UTF-8 frame boundaries/reconnection, localized search, area labels,
LiDAR verdict initialization, and calendar/streak alignment for all seven weekdays.
Existing page, map, log, mobile-clearance, and
motion-effect suites provide broader regression coverage. Screenshots are emitted
to the ignored Playwright artifact directory (or `UX_SCREENSHOT_DIR`).

**Safety-critical UI change:** the map STOP/release affordance changed. ROS command
values, firmware interlocks, and motion planning were not altered. Mocked request
checks establish what the GUI sends; they do not establish physical stopping time,
blade safety, localization accuracy, or behavior of this patch on installed robot
software. No physical validation or deployment is claimed.

Distance provenance for older sessions remains limited by the stored API data.
The UI reports unrecorded/possibly incomplete odometry instead of inventing travel.
Historical sessions contain area indices rather than stable area IDs, so their
fallback labels do not claim to recover historical user-assigned names.
