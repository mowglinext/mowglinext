import {useCallback, useState} from "react";
import {App} from "antd";
import {useTranslation} from "react-i18next";

// Shared state machine for the IMU yaw + dock pose auto-calibration.
// Used by both:
//   - RobotComponentEditor (Settings → Sensors): inline compass-icon button
//     next to the IMU Yaw input.
//   - OnboardingPage's ImuYawStep: a dedicated wizard step that walks the
//     operator through the preflight + start + apply flow.
//
// The hook owns the modal-open + running + result state so consumers can
// render any UI shell they want and just plug `startCalibration` /
// `applyCalibration` / `resetCalibration` into buttons.
//
// `applyCalibration` writes back the calibrated `imu_yaw` (always),
// `imu_pitch`/`imu_roll` (only when stationary samples ≥ 150 — node
// threshold), and surfaces dock-pose results from the server's pre-phase
// (which already persisted them to `mowgli_robot.yaml`). Settings save is
// the caller's responsibility.

export interface ImuYawCalibrationResult {
    success: boolean;
    message: string;
    imu_yaw_rad: number;
    imu_yaw_deg: number;
    samples_used: number;
    std_dev_deg: number;
    imu_pitch_rad?: number;
    imu_pitch_deg?: number;
    imu_roll_rad?: number;
    imu_roll_deg?: number;
    stationary_samples_used?: number;
    gravity_mag_mps2?: number;
    dock_valid?: boolean;
    dock_pose_x?: number;
    dock_pose_y?: number;
    dock_pose_yaw_rad?: number;
    dock_pose_yaw_deg?: number;
    dock_yaw_sigma_deg?: number;
    dock_undock_displacement_m?: number;
}

const CALIB_DURATION_SEC = 30;
// Backend budget is 150 s (dock pre-phase up to 25 s + drives ~20 s +
// optional mag rotation). Give the fetch a slightly larger client timeout
// so the AbortController doesn't fire while the server is still working.
const CALIB_CLIENT_TIMEOUT_MS = 155_000;

// Round to N decimal places — keep the YAML round-tripping stable.
const roundTo = (value: number, decimals: number): number => {
    const factor = Math.pow(10, decimals);
    return Math.round(value * factor) / factor;
};

const radToDeg = (rad: number) => (rad * 180) / Math.PI;

interface UseImuYawCalibrationOptions {
    /// Called after a successful "Apply" with each (key, value) pair the
    /// operator wants to commit. Signature matches the rest of the
    /// onboarding/settings forms — caller is responsible for persisting.
    onApplyValue: (key: string, value: number) => void;
    /// Read by `applyCalibration` to log the previous imu_yaw alongside
    /// the new one so operators can sanity-check the delta.
    currentImuYawRad?: number;
}

export const useImuYawCalibration = ({onApplyValue, currentImuYawRad}: UseImuYawCalibrationOptions) => {
    const {t} = useTranslation();
    const {notification} = App.useApp();
    const [calibOpen, setCalibOpen] = useState(false);
    const [calibRunning, setCalibRunning] = useState(false);
    const [calibResult, setCalibResult] = useState<ImuYawCalibrationResult | null>(null);

    const resetCalibration = useCallback(() => {
        setCalibRunning(false);
        setCalibResult(null);
    }, []);

    const closeCalibration = useCallback(() => {
        if (calibRunning) return;
        setCalibOpen(false);
        resetCalibration();
    }, [calibRunning, resetCalibration]);

    const openCalibration = useCallback(() => {
        resetCalibration();
        setCalibOpen(true);
    }, [resetCalibration]);

    const startCalibration = useCallback(async () => {
        setCalibRunning(true);
        setCalibResult(null);
        const controller = new AbortController();
        const timeoutId = setTimeout(() => controller.abort(), CALIB_CLIENT_TIMEOUT_MS);
        try {
            const res = await fetch("/api/calibration/imu-yaw", {
                method: "POST",
                headers: {"Content-Type": "application/json"},
                body: JSON.stringify({duration_sec: CALIB_DURATION_SEC}),
                signal: controller.signal,
            });
            if (!res.ok) {
                const errBody = await res.text();
                throw new Error(`HTTP ${res.status}: ${errBody}`);
            }
            const data = (await res.json()) as ImuYawCalibrationResult;
            setCalibResult(data);
        } catch (e: any) {
            const msg = e?.name === "AbortError"
                ? t("imuYawCalibration.timedOut")
                : (e?.message || String(e));
            notification.error({
                message: t("imuYawCalibration.failed"),
                description: msg,
            });
        } finally {
            clearTimeout(timeoutId);
            setCalibRunning(false);
        }
    }, [notification, t]);

    const applyCalibration = useCallback(() => {
        if (!calibResult || !calibResult.success) return;

        const appliedBits: string[] = [];

        onApplyValue("imu_yaw", roundTo(calibResult.imu_yaw_rad, 4));
        appliedBits.push(`imu_yaw = ${calibResult.imu_yaw_deg.toFixed(2)}°`);

        // Pitch/roll: node returns these only when the stationary baseline
        // had ≥ 150 samples, otherwise the values come back as 0 and we
        // would overwrite a good prior. Match the threshold.
        const stationaryOk =
            (calibResult.stationary_samples_used ?? 0) >= 150
            && Number.isFinite(calibResult.imu_pitch_rad)
            && Number.isFinite(calibResult.imu_roll_rad);
        if (stationaryOk) {
            onApplyValue("imu_pitch", roundTo(calibResult.imu_pitch_rad!, 4));
            onApplyValue("imu_roll", roundTo(calibResult.imu_roll_rad!, 4));
            appliedBits.push(
                `imu_pitch = ${calibResult.imu_pitch_deg!.toFixed(2)}°`,
                `imu_roll = ${calibResult.imu_roll_deg!.toFixed(2)}°`,
            );
        }

        // Dock pose: server pre-phase already wrote it to mowgli_robot.yaml.
        // Surface the result for verification — there is no form value to
        // change here.
        if (calibResult.dock_valid && Number.isFinite(calibResult.dock_pose_yaw_rad)) {
            appliedBits.push(
                `dock_pose_yaw = ${calibResult.dock_pose_yaw_deg!.toFixed(2)}° `
                + `(σ${calibResult.dock_yaw_sigma_deg!.toFixed(2)}°, `
                + `displacement ${calibResult.dock_undock_displacement_m?.toFixed(2) ?? "?"} m, `
                + "persisted to mowgli_robot.yaml)",
            );
        }

        const previousDeg = currentImuYawRad != null ? roundTo(radToDeg(currentImuYawRad), 2) : null;
        notification.success({
            message: t("imuYawCalibration.applied"),
            description:
                appliedBits.join(" · ")
                + (previousDeg != null ? ` ${t("imuYawCalibration.previously", {value: previousDeg})}` : "")
                + t("imuYawCalibration.rememberToSave"),
            duration: 6,
        });
        setCalibOpen(false);
        resetCalibration();
    }, [calibResult, onApplyValue, currentImuYawRad, notification, resetCalibration, t]);

    return {
        calibOpen,
        calibRunning,
        calibResult,
        openCalibration,
        closeCalibration,
        resetCalibration,
        startCalibration,
        applyCalibration,
    };
};
