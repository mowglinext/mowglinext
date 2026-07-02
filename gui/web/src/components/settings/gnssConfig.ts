export const GNSS_RECEIVER_FAMILY_OPTIONS = [
    { value: "auto", label: "gnssConfig.receiverFamily.auto.label" },
    { value: "ublox", label: "gnssConfig.receiverFamily.ublox.label" },
    { value: "unicore", label: "gnssConfig.receiverFamily.unicore.label" },
    { value: "nmea", label: "gnssConfig.receiverFamily.nmea.label" },
] as const;

export const GNSS_BAUD_OPTIONS = [
    { value: 115200, label: "115200" },
    { value: 230400, label: "230400" },
    { value: 460800, label: "460800" },
    { value: 921600, label: "921600" },
] as const;

export const GNSS_PROFILE_OPTIONS = [
    { value: "runtime_only", label: "gnssConfig.profile.runtime_only.label" },
    { value: "rover_high_precision", label: "gnssConfig.profile.rover_high_precision.label" },
    { value: "rover_high_precision_debug", label: "gnssConfig.profile.rover_high_precision_debug.label" },
    { value: "factory_reset", label: "gnssConfig.profile.factory_reset.label" },
] as const;

export const GNSS_SIGNAL_PROFILE_OPTIONS = [
    {
        value: "balanced",
        label: "gnssConfig.signalProfile.balanced.label",
        description: "gnssConfig.signalProfile.balanced.description",
    },
    {
        value: "high_precision",
        label: "gnssConfig.signalProfile.high_precision.label",
        description: "gnssConfig.signalProfile.high_precision.description",
    },
    {
        value: "all_signals",
        label: "gnssConfig.signalProfile.all_signals.label",
        description: "gnssConfig.signalProfile.all_signals.description",
    },
    {
        value: "minimal",
        label: "gnssConfig.signalProfile.minimal.label",
        description: "gnssConfig.signalProfile.minimal.description",
    },
    {
        value: "custom",
        label: "gnssConfig.signalProfile.custom.label",
        description: "gnssConfig.signalProfile.custom.description",
    },
] as const;

export const GNSS_SIGNAL_PROFILE_HELP_TEXT =
    "gnssConfig.signalProfileHelpText";

export const GNSS_SIGNAL_PROFILE_CUSTOM_HELP_TEXT =
    "gnssConfig.signalProfileCustomHelpText";

export const GNSS_PROFILE_RATE_OPTIONS = [
    { value: 1, label: "1 Hz" },
    { value: 5, label: "5 Hz" },
    { value: 7, label: "7 Hz" },
    { value: 10, label: "10 Hz" },
] as const;

export const GNSS_ACTION_SETTINGS_KEYS = [
    "gnss_receiver_family",
    "gnss_receiver_model",
    "gnss_serial_device",
    "gnss_serial_baud",
    "gnss_config_baud",
    "gnss_profile",
    "gnss_signal_profile",
    "gnss_profile_rate_hz",
    "gnss_signal_group",
    "gnss_unicore_pvt_algorithm",
    "gnss_unicore_rtk_reliability",
    "gnss_unicore_rtk_timeout_s",
    "gnss_unicore_dgps_timeout_s",
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

export const pickGnssActionSettings = (values: Record<string, any>): Record<string, any> => {
    const partial: Record<string, any> = {};
    for (const key of GNSS_ACTION_SETTINGS_KEYS) {
        if (Object.prototype.hasOwnProperty.call(values, key)) {
            partial[key] = values[key];
        }
    }
    return partial;
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
        title: "gnssConfig.unicore.title",
        description: "gnssConfig.unicore.description",
        fields: [
            {
                kind: "select",
                key: "gnss_receiver_model",
                label: "gnssConfig.unicore.receiverModel.label",
                tooltip: "gnssConfig.unicore.receiverModel.tooltip",
                options: [
                    { value: "", label: "gnssConfig.unicore.receiverModel.option.auto.label" },
                    { value: "UM982", label: "gnssConfig.unicore.receiverModel.option.um982.label" },
                    { value: "UM980", label: "gnssConfig.unicore.receiverModel.option.um980.label" },
                    { value: "UB9A0", label: "gnssConfig.unicore.receiverModel.option.ub9a0.label" },
                    { value: "UM981", label: "gnssConfig.unicore.receiverModel.option.um981.label" },
                    { value: "UM952", label: "gnssConfig.unicore.receiverModel.option.um952.label" },
                ],
                helpText: "gnssConfig.unicore.receiverModel.helpText",
            },
            {
                kind: "presetText",
                key: "gnss_signal_group",
                label: "gnssConfig.unicore.signalGroup.label",
                rawLabel: "gnssConfig.unicore.signalGroup.rawLabel",
                tooltip: "gnssConfig.unicore.signalGroup.tooltip",
                options: [
                    { value: "", label: "gnssConfig.unicore.signalGroup.option.default.label" },
                    {
                        value: "3 6",
                        label: "gnssConfig.unicore.signalGroup.option.um982.label",
                        description: "gnssConfig.unicore.signalGroup.option.um982.description",
                    },
                    {
                        value: "2",
                        label: "gnssConfig.unicore.signalGroup.option.allBands.label",
                        description: "gnssConfig.unicore.signalGroup.option.allBands.description",
                    },
                    {
                        value: "3",
                        label: "gnssConfig.unicore.signalGroup.option.ppp.label",
                        description: "gnssConfig.unicore.signalGroup.option.ppp.description",
                    },
                ],
                customOptionLabel: "gnssConfig.unicore.signalGroup.customOptionLabel",
                customPlaceholder: "gnssConfig.unicore.signalGroup.customPlaceholder",
                helpText: "gnssConfig.unicore.signalGroup.helpText",
            },
            {
                kind: "text",
                key: "gnss_unicore_pvt_algorithm",
                label: "gnssConfig.unicore.pvtAlgorithm.label",
                tooltip: "gnssConfig.unicore.pvtAlgorithm.tooltip",
                placeholder: "gnssConfig.unicore.pvtAlgorithm.placeholder",
                helpText: "gnssConfig.unicore.pvtAlgorithm.helpText",
            },
            {
                kind: "text",
                key: "gnss_unicore_rtk_reliability",
                label: "gnssConfig.unicore.rtkReliability.label",
                tooltip: "gnssConfig.unicore.rtkReliability.tooltip",
                placeholder: "gnssConfig.unicore.rtkReliability.placeholder",
            },
            {
                kind: "number",
                key: "gnss_unicore_rtk_timeout_s",
                label: "gnssConfig.unicore.rtkTimeout.label",
                tooltip: "gnssConfig.unicore.rtkTimeout.tooltip",
                min: 0,
                step: 1,
                addonAfter: "s",
            },
            {
                kind: "number",
                key: "gnss_unicore_dgps_timeout_s",
                label: "gnssConfig.unicore.dgpsTimeout.label",
                tooltip: "gnssConfig.unicore.dgpsTimeout.tooltip",
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

export const normalizeGnssReceiverModel = (value: unknown): string => {
    const normalized = normalizeGnssString(value).toUpperCase();
    switch (normalized) {
        case "":
        case "AUTO":
        case "UNKNOWN":
        case "UNKNOWN/AUTO":
        case "AUTO/UNKNOWN":
            return "";
        default:
            return normalized;
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
    findOptionLabel(normalizeGnssProfile(value), GNSS_PROFILE_OPTIONS, "gnssConfig.profile.runtime_only.label");

export const gnssSignalProfileLabel = (value: unknown): string =>
    findOptionLabel(normalizeGnssSignalProfile(value), GNSS_SIGNAL_PROFILE_OPTIONS, "gnssConfig.signalProfile.balanced.label");

export const gnssSignalProfileDescription = (value: unknown): string => {
    const normalized = normalizeGnssSignalProfile(value);
    const option = GNSS_SIGNAL_PROFILE_OPTIONS.find((candidate) => candidate.value === normalized);
    return option?.description ?? GNSS_SIGNAL_PROFILE_HELP_TEXT;
};
