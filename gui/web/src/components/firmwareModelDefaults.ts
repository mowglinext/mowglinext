/**
 * Firmware targets inferred from the mower model the operator picked during
 * onboarding — the flash flow must not ask for a board and panel a second
 * time when the model already identifies them.
 *
 * Board + panel come from the firmware's own names (board.h.template,
 * panel.h) and, where one exists, the published prebuilt permutations in
 * firmware/scripts/package_release.py. A model whose controller board the
 * firmware does not know (SA650, Sabo, CUSTOM) maps to nothing, and the UI
 * falls back to the manual target pickers; a partial mapping (900 ECO: the
 * panel is known, the mainboard is not) fills what it can and asks for the
 * rest. Nothing here is guessed.
 */
export type FirmwareSelection = {
    boardType?: string;
    panelType?: string;
    /** An exact PlatformIO/release-manifest target when the board is ambiguous. */
    firmwareTarget?: string;
    /** Persisted provenance for each independently editable field. */
    boardTypeOrigin?: FirmwareFieldOrigin;
    panelTypeOrigin?: FirmwareFieldOrigin;
    firmwareTargetOrigin?: FirmwareFieldOrigin;
    /** Mower model whose automatic defaults were last applied. */
    firmwareSelectionModel?: string;
};

export type FirmwareFieldOrigin = "auto" | "manual" | "legacy";

export type FirmwareModelDefaults = Readonly<Pick<FirmwareSelection, "boardType" | "panelType" | "firmwareTarget">>;

export const FIRMWARE_MODEL_DEFAULTS: Readonly<Record<string, FirmwareModelDefaults>> = {
    YardForce500: {
        // The stock YardForce 500 mainboard running Mowgli firmware. The
        // OpenMower "Vermut" replacement board for the same chassis is a
        // different host stack (its own branch), so it is never derived —
        // an operator who fitted one overrides the target by hand.
        boardType: "BOARD_YARDFORCE500",
        panelType: "PANEL_TYPE_YARDFORCE_500_CLASSIC",
    },
    YardForce500B: {
        boardType: "BOARD_YARDFORCE500B",
        panelType: "PANEL_TYPE_YARDFORCE_500B_CLASSIC",
    },
    BiltemaRM1000: {
        boardType: "BOARD_YARDFORCE500B",
        panelType: "PANEL_TYPE_YARDFORCE_900_ECO",
        firmwareTarget: "BiltemaRM1000",
    },
    LUV1000RI: {
        // The firmware names both the board and the panel after this model.
        // There is no prebuilt (and no PlatformIO env yet), so the flash
        // flow steers to the custom build path for it.
        boardType: "BOARD_LUV1000RI",
        panelType: "PANEL_TYPE_YARDFORCE_LUV1000RI",
    },
    YardForce900ECO: {
        // The 900 ECO panel is known; which mainboard it drives is not
        // recorded in the firmware, so the board stays a manual choice.
        panelType: "PANEL_TYPE_YARDFORCE_900_ECO",
    },
};

export const firmwareDefaultsForModel = (
    mowerModel: unknown,
): FirmwareModelDefaults | undefined => {
    if (typeof mowerModel !== "string") return undefined;
    return FIRMWARE_MODEL_DEFAULTS[mowerModel];
};

/** True when the model alone identifies both the board and the panel. */
export const modelIdentifiesFirmwareTarget = (mowerModel: unknown): boolean => {
    const defaults = firmwareDefaultsForModel(mowerModel);
    return !!defaults?.boardType && !!defaults?.panelType;
};

/**
 * Apply only inferred board/panel fields. Other firmware settings are returned
 * untouched, and a field marked as manually overridden is left untouched even
 * when the mower model changes.
 */
export const applyFirmwareModelDefaults = <T extends FirmwareSelection>(
    mowerModel: unknown,
    selection: T,
    manualOverrides: Partial<Record<keyof FirmwareSelection, boolean>> = {},
): T => {
    const defaults = firmwareDefaultsForModel(mowerModel);
    return {
        ...selection,
        // An empty string is deliberate: it overrides Formily's static
        // defaults and leaves an unsupported model visibly unselected.
        ...(manualOverrides.boardType ? {} : {boardType: defaults?.boardType ?? ""}),
        ...(manualOverrides.panelType ? {} : {panelType: defaults?.panelType ?? ""}),
        ...(manualOverrides.firmwareTarget ? {} : {firmwareTarget: defaults?.firmwareTarget ?? ""}),
    } as T;
};

/**
 * Convert persisted field provenance to the override policy used by the form.
 * Missing/unknown provenance is treated as legacy, except that selecting the
 * newly supported RM1000 migrates targetless legacy configs to its known-safe
 * board, panel, and firmware target defaults.
 */
export const manualOverridesFromProvenance = (
    selection: FirmwareSelection,
    mowerModel?: unknown,
): Partial<Record<"boardType" | "panelType" | "firmwareTarget", boolean>> => ({
    // A targetless config predates the RM1000 target. Once RM1000 has been
    // selected, its known board and SA900ECO panel defaults replace legacy
    // values unless a modern provenance marker says the user chose them.
    boardType: mowerModel === "BiltemaRM1000" && !selection.firmwareTarget
        ? selection.boardTypeOrigin === "manual"
        : selection.boardTypeOrigin !== "auto",
    panelType: mowerModel === "BiltemaRM1000" && !selection.firmwareTarget
        ? selection.panelTypeOrigin === "manual"
        : selection.panelTypeOrigin !== "auto",
    // The target field is new. A targetless legacy config cannot contain an
    // explicit target override, so infer the target from the selected model.
    firmwareTarget: selection.firmwareTargetOrigin !== undefined
        ? selection.firmwareTargetOrigin !== "auto"
        : selection.firmwareTarget !== undefined,
});
