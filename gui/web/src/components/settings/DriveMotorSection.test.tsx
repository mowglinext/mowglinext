import { beforeEach, describe, expect, it, vi } from "vitest";
import { fireEvent, render, screen, waitFor } from "@testing-library/react";
import { App } from "antd";
import i18n from "../../i18n";
import { DriveMotorSection } from "./DriveMotorSection";

const tuning = vi.hoisted(() => ({
    status: undefined as any,
    startFeedForward: vi.fn().mockResolvedValue({}),
    loadLatestReport: vi.fn(),
}));
vi.mock("../../hooks/useDriveTuning.ts", () => ({ useDriveTuning: () => ({
    ...tuning, startPID: vi.fn(), rollback: vi.fn(), loadingLatestReport: false,
}) }));
vi.mock("../../hooks/useEmergency.ts", () => ({ useEmergency: () => ({}) }));
vi.mock("../../hooks/useStatus.ts", () => ({ useStatus: () => ({}) }));
vi.mock("../../hooks/useDockingSensor.ts", () => ({ useDockingSensor: () => ({ dock_present: true }) }));

describe("shared motor calibration", () => {
    beforeEach(async () => {
        tuning.status = undefined;
        tuning.startFeedForward.mockClear();
        tuning.loadLatestReport.mockReset();
        await i18n.changeLanguage("en");
    });

    it.each(["en", "fr"])("shows only odometry and retains the modal controls for MAVROS (%s)", async language => {
        await i18n.changeLanguage(language);
        const onChange = vi.fn();
        render(<App><DriveMotorSection hardwareBackend="mavros" values={{ ticks_per_meter: 343.596123 }} onChange={onChange} /></App>);
        expect(document.body.textContent).not.toMatch(/feed.?forward|PWM|MANUAL\/m\/s|wheel_pid|PID/i);
        const ticks = screen.getByRole("spinbutton", { name: i18n.t("settingsDriveMotor.odometry.ticksPerMeter") });
        expect(ticks).toHaveValue("343.596123");
        fireEvent.change(ticks, { target: { value: "343.596789" } });
        fireEvent.blur(ticks);
        expect(onChange).toHaveBeenCalledWith("ticks_per_meter", 343.596789);
        fireEvent.click(screen.getByRole("button", { name: i18n.t("settingsDriveMotor.odometry.start") }));
        expect(await screen.findByText(i18n.t("settingsDriveMotor.odometry.modalTitle"))).toBeInTheDocument();
        for (const key of ["distance", "testSpeed", "odomTimeout"]) {
            const unit = key === "distance" ? "m" : key === "testSpeed" ? "m/s" : "s";
            expect(screen.getByRole("spinbutton", { name: `${i18n.t(`settingsDriveMotor.ffModal.fields.${key}`)}, ${unit}` })).toBeInTheDocument();
        }
        for (const key of ["passes", "allowUndockIfDocked", "undockDistance", "applyPersistIfSuccessful"]) {
            expect(screen.getByText(i18n.t(`settingsDriveMotor.common.${key}`))).toBeInTheDocument();
        }
        expect(screen.getByText(i18n.t("settingsDriveMotor.ffModal.fields.autoUTurn"))).toBeInTheDocument();
        expect(screen.getAllByText(i18n.t("settingsDriveMotor.common.robotOnDockTitle")).length).toBeGreaterThan(0);
        expect(screen.getByText(i18n.t("settingsDriveMotor.odometry.reportOnlyDescription"))).toBeInTheDocument();
        expect(document.body.textContent).not.toMatch(/feed.?forward|PWM|wheel_pid|PID/i);
    });

    it("keeps the historical STM32 controls", () => {
        render(<App><DriveMotorSection hardwareBackend="mowgli" values={{}} onChange={vi.fn()} /></App>);
        expect(screen.getByRole("button", { name: "Start odometry/feed-forward calibration" })).toBeInTheDocument();
        expect(screen.getByRole("spinbutton", { name: "PWM per m/s, PWM" })).toBeInTheDocument();
        expect(screen.getByRole("button", { name: "Start PID auto-tune" })).toBeInTheDocument();
    });

    it.each(["en", "fr"])("shows the actual translated MAVROS warning cause (%s)", async language => {
        await i18n.changeLanguage(language);
        tuning.status = { feed_forward: {
            status: "warning", message: "No valid RTK/GNSS-backed odometry distance was accepted.",
        } };
        render(<App><DriveMotorSection hardwareBackend="mavros" values={{}} onChange={vi.fn()} /></App>);
        expect(screen.getByText(i18n.t("settingsDriveMotor.odometry.validation.noReference"))).toBeInTheDocument();
        expect(document.body.textContent).not.toMatch(/feed.?forward|PWM|speed error/i);
    });

    it.each(["en", "fr"])("shows the odometry error percentage in the actual summary (%s)", async language => {
        await i18n.changeLanguage(language);
        tuning.status = { feed_forward: {
            status: "validated", message: "Validated odometry with distance error 0.5%.",
        } };
        render(<App><DriveMotorSection hardwareBackend="mavros" values={{}} onChange={vi.fn()} /></App>);
        expect(screen.getByText(i18n.t("settingsDriveMotor.odometry.validation.validated", { errorPct: "0.5" }))).toBeInTheDocument();
    });

    it("syncs only float ticks from an applied MAVROS report, including legacy reports", async () => {
        tuning.status = { job: { id: "one", apply: true, state: "succeeded", report_path: "/report.yaml" } };
        tuning.loadLatestReport.mockResolvedValue({
            latest_report: { report_path: "/report.yaml" },
            parsed: { proposed_params: { ticks_per_meter: 343.596123, wheel_pid_pwm_per_mps: 999, wheel_pid_kp: 99 } },
        });
        const accept = vi.fn();
        render(<App><DriveMotorSection hardwareBackend="mavros" values={{}} onChange={vi.fn()} acceptPersistedValues={accept} /></App>);
        await waitFor(() => expect(accept).toHaveBeenCalledWith({ ticks_per_meter: 343.596123 }));
    });
});
