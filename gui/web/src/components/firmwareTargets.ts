/**
 * The flashable hardware targets: controller boards and front panels, as the
 * firmware names them (board.h.template). One place for the option lists so
 * the derived-target summary and the manual override pickers can never
 * disagree on a label.
 */
export type FirmwareTargetOption = Readonly<{ label: string; value: string }>;

export const BOARD_OPTIONS: readonly FirmwareTargetOption[] = [
    {label: "Mowgli - YardForce 500 Classic", value: "BOARD_YARDFORCE500"},
    {label: "Mowgli - YardForce 500 B Variant", value: "BOARD_YARDFORCE500B"},
    {label: "Mowgli - LUV1000RI", value: "BOARD_LUV1000RI"},
    // OpenMower's RP2040 board. Its host side lives on a separate branch, so on
    // dev it is only reachable through the manual target override — never a
    // model-derived default.
    {label: "Vermut - YardForce 500 Classic", value: "BOARD_VERMUT_YARDFORCE500"},
];

export const PANEL_OPTIONS: readonly FirmwareTargetOption[] = [
    {label: "YardForce 500 Classic", value: "PANEL_TYPE_YARDFORCE_500_CLASSIC"},
    {label: "YardForce 500B Classic", value: "PANEL_TYPE_YARDFORCE_500B_CLASSIC"},
    {label: "YardForce LUV1000RI", value: "PANEL_TYPE_YARDFORCE_LUV1000RI"},
    {label: "YardForce 900 ECO", value: "PANEL_TYPE_YARDFORCE_900_ECO"},
];

// Boards that flash WITHOUT compiling: Vermut has its own release-zip path, and
// the Mowgli STM32 boards have prebuilt binaries in the release manifest (mirrors
// firmware/scripts/package_release.py's PERMUTATIONS). A board absent here (e.g.
// LUV1000RI) has no prebuilt yet and must use the Expert compile path — the UI
// steers the user there instead of letting the flash fail at runtime.
export const PREBUILT_BOARDS: ReadonlySet<string> = new Set<string>([
    "BOARD_VERMUT_YARDFORCE500",
    "BOARD_YARDFORCE500",
    "BOARD_YARDFORCE500B",
]);

export const VERMUT_BOARD = "BOARD_VERMUT_YARDFORCE500";

// The Biltema RM1000 ships the 500B mainboard; with that model selected the
// board is named after the mower the operator actually owns.
const RM1000_MODEL = "BiltemaRM1000";
const RM1000_BOARD = "BOARD_YARDFORCE500B";

/** Board options as shown for a given mower model (same values, model-aware labels). */
export const boardOptionsForModel = (
    mowerModel: string | undefined,
    t: (key: string) => string,
): FirmwareTargetOption[] =>
    BOARD_OPTIONS.map((o) =>
        mowerModel === RM1000_MODEL && o.value === RM1000_BOARD
            ? {...o, label: t("flashBoard.boardBiltemaRM1000")}
            : o,
    );

const labelOf = (options: readonly FirmwareTargetOption[], value: string | undefined): string | undefined =>
    options.find((o) => o.value === value)?.label;

export const boardLabel = (
    value: string | undefined,
    mowerModel: string | undefined,
    t: (key: string) => string,
): string | undefined => labelOf(boardOptionsForModel(mowerModel, t), value);
export const panelLabel = (value: string | undefined): string | undefined => labelOf(PANEL_OPTIONS, value);

export const hasPrebuiltFirmware = (board: string | undefined): boolean =>
    !!board && PREBUILT_BOARDS.has(board);
