# GNSS GUI Design

## Goal

MowgliNext should expose a stable GNSS configuration layer that stays usable
across Unicore, u-blox, and future receivers without teaching normal users
vendor command syntax.

The GUI is responsible for:

- collecting vendor-neutral intent
- persisting a stable GNSS configuration contract
- surfacing expert-only vendor overrides when needed

The backend is responsible for:

- translating saved GUI intent into receiver-family-specific apply plans
- calling `gnss_config_plan` / `gnss_config_apply`
- handling confirm, reboot, baud migration, and factory reset safety

## Design Rule

The normal user flow must not model raw Unicore commands such as
`CONFIG SIGNALGROUP 3 6`.

Why:

- `SIGNALGROUP` is a Unicore detail, not a GNSS concept
- the main UI must stay portable across receiver families
- future receivers will need different knobs, but the same user goals
- backend translation belongs in Universal GNSS, not in the web page

Universal GNSS profile ids such as `runtime_only` or
`rover_high_precision` are acceptable in the normal UI because they are
backend-level profile abstractions, not vendor commands.

## Modes

### User mode

Shown by default on the GPS page and onboarding GPS step.

Fields:

- Receiver Profile
  - `runtime_only`
  - `rover_high_precision`
  - `rover_high_precision_debug`
  - `factory_reset`
- Signal Profile
  - `balanced` -> Balanced
  - `high_precision` -> Maximum Accuracy
  - `all_signals` -> Maximum Compatibility
  - `minimal` -> Low Bandwidth
  - `custom`
- Position Rate
  - `1`
  - `5`
  - `7`
  - `10`
- Runtime Baud
  - `115200`
  - `230400`
  - `460800`
  - `921600`
- Configured Receiver Baud
  - `115200`
  - `230400`
  - `460800`
  - `921600`

Help text:

- Normal settings are vendor-neutral.
- Signal Profile descriptions must be visible directly in the UI.
- Changing baud requires the receiver and runtime baud to match.
- `460800` is recommended for unstable USB serial links.
- `921600` may work on direct UART or robust USB adapters but must be validated.
- `Custom` is reserved for a future workflow that will accept user-provided
  profile files or raw receiver command sets in Expert/Developer mode.

### Expert mode

Hidden behind an explicit toggle.

Fields:

- Receiver Family
- Serial Device
- Vendor settings card, selected by receiver family

Current Unicore expert fields:

- Signal group preset
- Raw signal group
- PVT algorithm
- RTK reliability
- RTK timeout
- DGPS timeout

Help text:

- Expert settings are receiver-family specific.
- For UM982, `UM982 recommended` maps to `CONFIG SIGNALGROUP 3 6`.

### Developer mode

Not wired in the first GUI layer.

Planned capabilities:

- raw receiver command textarea
- dry-run profile plan preview
- guarded reset/apply tooling

These require a backend API and must not be faked in the UI.

## Persistence Contract

The GUI persists the vendor-neutral runtime contract keys:

- `GNSS_PROFILE`
- `GNSS_SIGNAL_PROFILE`
- `GNSS_PROFILE_RATE_HZ`
- `GNSS_SERIAL_BAUD`
- `GNSS_CONFIG_BAUD`
- `GNSS_RECEIVER_FAMILY`
- `GNSS_SERIAL_DEVICE`

Vendor-specific expert values may also be stored in the YAML config for
future translation, but they should not redefine the main user-facing model.

## Translation Boundary

The web UI saves intent.
Universal GNSS translates intent.

Examples:

- `GNSS_SIGNAL_PROFILE=high_precision` may later map to different command
  sets for Unicore, u-blox, or Septentrio.
- `GNSS_PROFILE=rover_high_precision` may later expand into different
  receiver command plans per family.
- Unicore expert overrides such as signal group stay optional inputs to the
  translator, not first-class requirements for all users.

## Actions

The first GUI layer exposes the action surface but does not fake backend work.

Current state:

- Save settings: implemented
- Save + restart GPS: implemented for runtime serial/NTRIP changes
- Apply profile to receiver: disabled until backend API exists
- Factory reset + apply profile: disabled until backend API exists

## Backend Gap

The missing backend task is an API that:

- reads the saved GNSS contract
- maps it to receiver-family-specific Universal GNSS plan/apply arguments
- runs `gnss_config_plan` and `gnss_config_apply`
- handles factory reset confirmation and recovery
- handles baud migration and reconnect sequencing

Until that exists, the GUI must clearly say that profile apply and factory
reset are not wired yet.
