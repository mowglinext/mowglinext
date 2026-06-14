export const GNSS_RECEIVER_FAMILY_OPTIONS = [
    { value: "auto", label: "Auto" },
    { value: "ublox", label: "u-blox" },
    { value: "unicore", label: "Unicore" },
    { value: "nmea", label: "NMEA" },
] as const;

export const GNSS_BAUD_OPTIONS = [
    { value: 115200, label: "115200" },
    { value: 230400, label: "230400" },
    { value: 460800, label: "460800" },
    { value: 921600, label: "921600" },
] as const;

export const GNSS_PROFILE_OPTIONS = [
    { value: "runtime_only", label: "Runtime only" },
    { value: "rover_high_precision", label: "Rover high precision" },
    { value: "rover_high_precision_debug", label: "Rover high precision + debug" },
    { value: "factory_reset", label: "Factory reset" },
] as const;

export const GNSS_SIGNAL_PROFILE_OPTIONS = [
    {
        value: "balanced",
        label: "Balanced",
        description:
            "Recommended default profile. Uses a balanced mix of available GNSS constellations and signal bands.",
    },
    {
        value: "high_precision",
        label: "Maximum Accuracy",
        description:
            "Prioritizes multi-band and high-precision signals for the best positioning performance.",
    },
    {
        value: "all_signals",
        label: "Maximum Compatibility",
        description:
            "Uses the widest set of commonly supported signals for maximum receiver and regional compatibility.",
    },
    {
        value: "minimal",
        label: "Low Bandwidth",
        description:
            "Reduces signal and observation load when bandwidth, processing power or correction transport capacity is limited.",
    },
    {
        value: "custom",
        label: "Custom",
        description:
            "Reserved for future user-provided profile files or raw receiver command sets when backend support is available.",
    },
] as const;

export const GNSS_SIGNAL_PROFILE_HELP_TEXT =
    "Signal profiles are vendor-neutral. Different receiver families can map the same profile to different signal and constellation choices.";

export const GNSS_SIGNAL_PROFILE_CUSTOM_HELP_TEXT =
    "Custom will later support user-provided profile files or raw receiver command sets from Expert/Developer mode. Upload and raw-command backend support are not wired yet.";

export const GNSS_PROFILE_RATE_OPTIONS = [
    { value: 1, label: "1 Hz" },
    { value: 5, label: "5 Hz" },
    { value: 7, label: "7 Hz" },
    { value: 10, label: "10 Hz" },
] as const;

export const GNSS_CUSTOM_OPTION_VALUE = "__custom__";

export type GnssPresetTextFieldOption = {
    value: string;
    label: string;
    description?: string;
};

export type GnssSelectFieldOption = {
    value: string;
    label: string;
    description?: string;
};

export type GnssPresetTextFieldDefinition = {
    kind: "presetText";
    key: string;
    label: string;
    rawLabel?: string;
    tooltip: string;
    options: readonly GnssPresetTextFieldOption[];
    customOptionLabel: string;
    customPlaceholder: string;
    helpText?: string;
};

export type GnssTextFieldDefinition = {
    kind: "text";
    key: string;
    label: string;
    tooltip: string;
    placeholder?: string;
    helpText?: string;
};

export type GnssSelectFieldDefinition = {
    kind: "select";
    key: string;
    label: string;
    tooltip: string;
    options: readonly GnssSelectFieldOption[];
    helpText?: string;
};

export type GnssNumberFieldDefinition = {
    kind: "number";
    key: string;
    label: string;
    tooltip: string;
    placeholder?: string;
    min?: number;
    step?: number;
    addonAfter?: string;
    helpText?: string;
};

export type GnssAdvancedFieldDefinition =
    | GnssPresetTextFieldDefinition
    | GnssTextFieldDefinition
    | GnssSelectFieldDefinition
    | GnssNumberFieldDefinition;

export type GnssReceiverAdvancedDefinition = {
    title: string;
    description: string;
    fields: readonly GnssAdvancedFieldDefinition[];
};

export const GNSS_ADVANCED_SETTINGS_BY_FAMILY: Record<string, GnssReceiverAdvancedDefinition | undefined> = {
    unicore: {
        title: "Vendor-Specific Settings",
        description:
            "Use these only when the generic GNSS abstractions are not enough. " +
            "UM982 signal-group tuning stays here instead of leaking into the main user workflow.",
        fields: [
            {
                kind: "presetText",
                key: "gnss_signal_group",
                label: "Signal Group Preset",
                rawLabel: "Raw Signal Group",
                tooltip:
                    "Choose a Unicore signal-group preset, then fine-tune the raw value only if needed.",
                options: [
                    { value: "", label: "Default / Current" },
                    {
                        value: "3 6",
                        label: "UM982 recommended",
                        description: "Maps to CONFIG SIGNALGROUP 3 6.",
                    },
                    {
                        value: "2",
                        label: "All bands (UM980 / UM981)",
                        description: "Maps to CONFIG SIGNALGROUP 2.",
                    },
                    {
                        value: "3",
                        label: "PPP optimized",
                        description: "Maps to CONFIG SIGNALGROUP 3.",
                    },
                ],
                customOptionLabel: "Custom",
                customPlaceholder: "e.g. 3 6",
                helpText:
                    "Normal settings are vendor-neutral. Expert settings are receiver-family specific. " +
                    "For UM982, \"UM982 recommended\" maps to CONFIG SIGNALGROUP 3 6.",
            },
            {
                kind: "text",
                key: "gnss_unicore_pvt_algorithm",
                label: "PVT Algorithm",
                tooltip:
                    "Optional Unicore-specific PVT algorithm override that the future backend translator can map to vendor commands.",
                placeholder: "e.g. MULTI",
                helpText:
                    "Leave blank to keep the receiver default/current value. This field is saved now, " +
                    "but backend apply translation is still TODO.",
            },
            {
                kind: "text",
                key: "gnss_unicore_rtk_reliability",
                label: "RTK Reliability",
                tooltip:
                    "Optional Unicore-specific RTK reliability tuning passed through only in Expert mode.",
                placeholder: "e.g. HIGH",
            },
            {
                kind: "number",
                key: "gnss_unicore_rtk_timeout_s",
                label: "RTK Timeout",
                tooltip: "Optional receiver-family-specific RTK timeout override.",
                min: 0,
                step: 1,
                addonAfter: "s",
            },
            {
                kind: "number",
                key: "gnss_unicore_dgps_timeout_s",
                label: "DGPS Timeout",
                tooltip: "Optional receiver-family-specific DGPS timeout override.",
                min: 0,
                step: 1,
                addonAfter: "s",
            },
        ],
    },
};

export const normalizeGnssString = (value: unknown): string => {
    if (typeof value === "string") {
        return value.trim();
    }
    if (typeof value === "number" && Number.isFinite(value)) {
        return String(value);
    }
    return "";
};

export const normalizeGnssSignalGroup = (value: unknown): string =>
    normalizeGnssString(value)
        .split(/\s+/)
        .filter(Boolean)
        .join(" ");

const normalizeOptionValue = (value: unknown): string =>
    normalizeGnssString(value).toLowerCase().replace(/-/g, "_");

export const normalizeGnssProfile = (value: unknown): string => {
    switch (normalizeOptionValue(value)) {
        case "":
        case "runtime_only":
        case "balanced":
        case "power_saving":
            return "runtime_only";
        case "high_precision":
        case "survey":
        case "rover_high_precision":
            return "rover_high_precision";
        case "debug":
        case "rover_high_precision_debug":
            return "rover_high_precision_debug";
        case "factory_reset":
            return "factory_reset";
        default:
            return "runtime_only";
    }
};

export const normalizeGnssSignalProfile = (value: unknown): string => {
    switch (normalizeOptionValue(value)) {
        case "":
        case "balanced":
            return "balanced";
        case "minimal":
            return "minimal";
        case "ppp_optimized":
        case "high_precision":
            return "high_precision";
        case "all_signals":
            return "all_signals";
        case "custom":
            return "custom";
        default:
            return "custom";
    }
};

export const inferPresetTextSelection = (
    field: GnssPresetTextFieldDefinition,
    rawValue: unknown,
): string => {
    const normalized = field.key === "gnss_signal_group"
        ? normalizeGnssSignalGroup(rawValue)
        : normalizeGnssString(rawValue);
    if (!normalized) {
        return "";
    }
    return field.options.some((option) => option.value === normalized)
        ? normalized
        : GNSS_CUSTOM_OPTION_VALUE;
};

const findOptionLabel = (
    value: unknown,
    options: readonly { value: string; label: string }[],
    fallback: string,
): string => {
    const normalized = normalizeOptionValue(value);
    const option = options.find((candidate) => candidate.value === normalized);
    return option?.label ?? (normalized || fallback);
};

export const gnssProfileLabel = (value: unknown): string =>
    findOptionLabel(normalizeGnssProfile(value), GNSS_PROFILE_OPTIONS, "Runtime only");

export const gnssSignalProfileLabel = (value: unknown): string =>
    findOptionLabel(normalizeGnssSignalProfile(value), GNSS_SIGNAL_PROFILE_OPTIONS, "Balanced");

export const gnssSignalProfileDescription = (value: unknown): string => {
    const normalized = normalizeGnssSignalProfile(value);
    const option = GNSS_SIGNAL_PROFILE_OPTIONS.find((candidate) => candidate.value === normalized);
    return option?.description ?? GNSS_SIGNAL_PROFILE_HELP_TEXT;
};
