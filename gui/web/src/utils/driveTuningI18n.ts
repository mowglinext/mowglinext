import type { TFunction } from "i18next";

type RegexTranslation = {
    pattern: RegExp;
    render: (t: TFunction, match: RegExpMatchArray) => string;
};

const EXACT_TRANSLATIONS: Record<string, string> = {
    "Failed to fetch drive tuning status": "settingsDriveMotor.backend.statusFetchFailed",
    "rollback requires confirm=true": "settingsDriveMotor.backend.errors.rollbackRequiresConfirm",
    "a drive tuning job is already running": "settingsDriveMotor.backend.errors.jobAlreadyRunning",
    "backup file did not contain drive parameters": "settingsDriveMotor.backend.errors.backupDidNotContainParameters",
    "Drive tuning rollback applied live and persisted to mowgli_robot.yaml": "settingsDriveMotor.backend.rollback.successMessage",
    "drive tuning command failed": "settingsDriveMotor.backend.errors.commandFailed",
    "drive tuning applied live, but mowgli_robot.yaml persistence failed": "settingsDriveMotor.backend.errors.persistenceAfterApply",
    "distance_m must be between 2 and 10": "settingsDriveMotor.backend.validation.distanceRange",
    "test_speed_mps must be > 0 and <= 0.5": "settingsDriveMotor.backend.validation.testSpeedRange",
    "odom_timeout_s must be > 0": "settingsDriveMotor.backend.validation.odomTimeoutPositive",
    "passes must be >= 1": "settingsDriveMotor.backend.validation.passesMin",
    "turn_direction must be 'left' or 'right'": "settingsDriveMotor.backend.validation.turnDirection",
    "max_speed_mps must be > 0 and <= 0.5": "settingsDriveMotor.backend.validation.maxSpeedRange",
    "segment_duration_s must be >= 2.0": "settingsDriveMotor.backend.validation.segmentDurationMin",
    "No feed-forward trials recorded yet.": "settingsDriveMotor.backend.summary.feedForward.noTrials",
    "A feed-forward pass stalled or produced no wheel motion. Re-run after checking mechanics, traction power, and odometry.": "settingsDriveMotor.backend.summary.feedForward.stall",
    "Live oscillation was observed during feed-forward calibration. Review mechanics and feed-forward before accepting the result.": "settingsDriveMotor.backend.summary.feedForward.liveOscillation",
    "Feed-forward report exists, but no RTK/GPS-backed odometry validation was accepted.": "settingsDriveMotor.backend.summary.feedForward.noAcceptedRtk",
    "Stop behavior warning: residual motion detected after zero-speed command. Stop trials were excluded from PID gain selection, but braking/control should be reviewed before accepting this tune.": "settingsDriveMotor.backend.summary.pid.stopBehaviorReviewBraking",
    "Stop behavior warning: residual motion detected after zero-speed command. Stop trials were excluded from PID gain selection.": "settingsDriveMotor.backend.summary.pid.stopBehaviorExcluded",
    "No PID step-response report recorded yet.": "settingsDriveMotor.backend.summary.pid.noTrials",
    "PID report shows a stall or no-motion step. Re-run after checking mechanics and feed-forward.": "settingsDriveMotor.backend.summary.pid.stall",
    "Live oscillation was observed during PID tuning. Re-run after reducing aggression or reviewing feed-forward.": "settingsDriveMotor.backend.summary.pid.liveOscillation",
    "PID report suggests sustained integral saturation under load. Review KI and integral limit before accepting the result.": "settingsDriveMotor.backend.summary.pid.integralSaturation",
    "Zero-speed stop trials were excluded from gain selection.": "settingsDriveMotor.backend.summary.pid.zeroSpeedExcludedSuffix",
};

const REGEX_TRANSLATIONS: RegexTranslation[] = [
    {
        pattern: /^drive tuning finished, but the report could not be read: (.+)$/,
        render: (t, match) => t("settingsDriveMotor.backend.errors.reportReadAfterRun", { error: match[1] }),
    },
    {
        pattern: /^(.+) container not found$/,
        render: (t, match) => t("settingsDriveMotor.backend.errors.containerNotFound", { container: match[1] }),
    },
    {
        pattern: /^failed to read report (.+): (.+)$/,
        render: (t, match) => t("settingsDriveMotor.backend.errors.readReport", { path: match[1], details: match[2] }),
    },
    {
        pattern: /^failed to read backup (.+): (.+)$/,
        render: (t, match) => t("settingsDriveMotor.backend.errors.readBackup", { path: match[1], details: match[2] }),
    },
    {
        pattern: /^failed to list drive tuning reports: (.+)$/,
        render: (t, match) => t("settingsDriveMotor.backend.errors.listReports", { details: match[1] }),
    },
    {
        pattern: /^failed to marshal YAML: (.+)$/,
        render: (t, match) => t("settingsDriveMotor.backend.errors.marshalYaml", { error: match[1] }),
    },
    {
        pattern: /^Odom error ([0-9.]+)%, speed error ([0-9.]+)%\.$/,
        render: (t, match) => t("settingsDriveMotor.backend.summary.feedForward.errorSummary", {
            odomPct: match[1],
            speedPct: match[2],
        }),
    },
    {
        pattern: /^Validated with excellent odometry \(([0-9.]+)%\) and acceptable feed-forward speed error \(([0-9.]+)%\)\.$/,
        render: (t, match) => t("settingsDriveMotor.backend.summary.feedForward.validatedExcellent", {
            odomPct: match[1],
            speedPct: match[2],
        }),
    },
    {
        pattern: /^Validated with acceptable odometry \(([0-9.]+)%\) and feed-forward speed error \(([0-9.]+)%\)\.$/,
        render: (t, match) => t("settingsDriveMotor.backend.summary.feedForward.validatedAcceptable", {
            odomPct: match[1],
            speedPct: match[2],
        }),
    },
    {
        pattern: /^Feed-forward calibration completed with a post-analysis oscillation warning \(odom ([0-9.]+)%, speed ([0-9.]+)%\)\. Review the report before accepting the result\.$/,
        render: (t, match) => t("settingsDriveMotor.backend.summary.feedForward.postOscillationWarning", {
            odomPct: match[1],
            speedPct: match[2],
        }),
    },
    {
        pattern: /^Calibration completed with warnings \(odom ([0-9.]+)%, speed ([0-9.]+)%\)\. Review trial notes before accepting the result\.$/,
        render: (t, match) => t("settingsDriveMotor.backend.summary.feedForward.completedWithWarnings", {
            odomPct: match[1],
            speedPct: match[2],
        }),
    },
    {
        pattern: /^PID overshoot reached ([0-9.]+)% on at least one step\.$/,
        render: (t, match) => t("settingsDriveMotor.backend.summary.pid.overshootReached", {
            overshootPct: match[1],
        }),
    },
    {
        pattern: /^Post-analysis oscillation detected but no live oscillation was observed; max overshoot ([0-9.]+)% remains usable for conservative tuning\.$/,
        render: (t, match) => t("settingsDriveMotor.backend.summary.pid.postOscillationUsable", {
            overshootPct: match[1],
        }),
    },
    {
        pattern: /^PID response completed with warnings; max overshoot ([0-9.]+)%\. Review trial notes before accepting the result\.( Zero-speed stop trials were excluded from gain selection\.)?$/,
        render: (t, match) => {
            const suffix = match[2] ? ` ${t("settingsDriveMotor.backend.summary.pid.zeroSpeedExcludedSuffix")}` : "";
            return `${t("settingsDriveMotor.backend.summary.pid.completedWithWarnings", {
                overshootPct: match[1],
            })}${suffix}`;
        },
    },
    {
        pattern: /^Validated PID step response with max overshoot ([0-9.]+)%\.( Zero-speed stop trials were excluded from gain selection\.)?$/,
        render: (t, match) => {
            const suffix = match[2] ? ` ${t("settingsDriveMotor.backend.summary.pid.zeroSpeedExcludedSuffix")}` : "";
            return `${t("settingsDriveMotor.backend.summary.pid.validated", {
                overshootPct: match[1],
            })}${suffix}`;
        },
    },
];

const INTERNAL_TIER_TRANSLATIONS: Record<string, string> = {
    medium: "settingsDriveMotor.report.tiers.medium",
    lightweight: "settingsDriveMotor.report.tiers.lightweight",
    heavy: "settingsDriveMotor.report.tiers.heavy",
    "extra-heavy": "settingsDriveMotor.report.tiers.extraHeavy",
};

const TRIAL_PHASE_TRANSLATIONS: Record<string, string> = {
    feedforward: "settingsDriveMotor.report.trials.phases.feedForward",
    pid: "settingsDriveMotor.report.trials.phases.pid",
    response: "settingsDriveMotor.report.trials.phases.response",
};

const TRIAL_QUALITY_TRANSLATIONS: Record<string, string> = {
    ok: "settingsDriveMotor.report.trials.qualities.ok",
    warning: "settingsDriveMotor.report.trials.qualities.warning",
    poor: "settingsDriveMotor.report.trials.qualities.poor",
};

export const translateDriveTuningBackendMessage = (
    t: TFunction,
    message?: string | null,
): string | undefined => {
    if (!message) {
        return undefined;
    }

    const exactKey = EXACT_TRANSLATIONS[message];
    if (exactKey) {
        return t(exactKey);
    }

    for (const entry of REGEX_TRANSLATIONS) {
        const match = message.match(entry.pattern);
        if (match) {
            return entry.render(t, match);
        }
    }

    return undefined;
};

export const translateDriveTuningInternalTier = (
    t: TFunction,
    tier?: string | null,
): string | undefined => {
    if (!tier) {
        return undefined;
    }
    const key = INTERNAL_TIER_TRANSLATIONS[tier];
    return key ? t(key) : tier;
};

export const translateDriveTuningTrialPhase = (
    t: TFunction,
    phase?: string | null,
): string => {
    if (!phase) {
        return t("settingsDriveMotor.common.unknown");
    }
    const key = TRIAL_PHASE_TRANSLATIONS[phase];
    return key ? t(key) : phase;
};

export const translateDriveTuningTrialQuality = (
    t: TFunction,
    quality?: string | null,
): string => {
    if (!quality) {
        return t("settingsDriveMotor.report.trials.qualities.ok");
    }
    const key = TRIAL_QUALITY_TRANSLATIONS[quality];
    return key ? t(key) : quality;
};

export const formatDriveTuningBoolean = (
    t: TFunction,
    value?: boolean | null,
): string => {
    if (value == null) {
        return t("settingsDriveMotor.common.unknown");
    }
    return value ? t("settingsDriveMotor.common.yes") : t("settingsDriveMotor.common.no");
};
