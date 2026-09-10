import { describe, expect, it, vi } from "vitest";
import { render, screen } from "@testing-library/react";
import userEvent from "@testing-library/user-event";
import { ThemeProvider } from "../../theme/ThemeContext.tsx";
import { LedsSection } from "./LedsSection.tsx";

describe("LedsSection", () => {
    function renderSection(
        values: Record<string, any> = {},
        overrides: Partial<{
            onChange: (key: string, value: any) => void;
            isOverridden: (key: string) => boolean;
            hasDefault: (key: string) => boolean;
            onReset: (key: string) => void;
        }> = {},
    ) {
        const onChange = overrides.onChange ?? vi.fn();
        render(
            <ThemeProvider>
                <LedsSection
                    values={values}
                    onChange={onChange}
                    isOverridden={overrides.isOverridden}
                    hasDefault={overrides.hasDefault}
                    onReset={overrides.onReset}
                />
            </ThemeProvider>,
        );
        return { onChange };
    }

    const enabledValues = {
        led_enabled: true,
        led_count: 16,
        led_spi_device: "/dev/spidev4.1",
        led_spi_speed_hz: 2400000,
        led_brightness: 0.6,
        led_idle_scale: 0.1,
        led_refresh_hz: 20,
        led_low_battery_percent: 20,
        led_charge_full_percent: 99,
        led_status_timeout_s: 5,
        led_keepalive_s: 2,
        led_device_retry_s: 30,
    };

    // An absent led_enabled key must read as OFF, matching the schema and ROS2
    // template defaults. If this ever disagreed with them the switch would show
    // ON for a robot whose config says nothing and whose ring is dark.
    it("renders the ring as off when the key is absent", () => {
        renderSection({});
        expect(screen.getByRole("switch")).not.toBeChecked();
    });

    it("hides every control until the ring is enabled", () => {
        renderSection({});
        expect(screen.queryByRole("spinbutton")).not.toBeInTheDocument();
        expect(screen.queryByRole("textbox")).not.toBeInTheDocument();
        expect(screen.queryByText(/LED count/i)).not.toBeInTheDocument();
    });

    // The persistence-critical path: the backend prunes any value equal to the
    // schema default (false), so switching ON must write `true` — that is the
    // only value that survives the prune and reaches the installed yaml. This
    // is the shape of the lidar_enabled bug fixed in #508; the Go-side guard is
    // TestLedEnabledSchemaDefaultIsFalseSoTurningItOnPersists.
    it("writes true when the operator switches the ring on", async () => {
        const onChange = vi.fn();
        renderSection({}, { onChange });

        await userEvent.click(screen.getByRole("switch"));

        expect(onChange).toHaveBeenCalledWith("led_enabled", true);
    });

    it("writes false when the operator switches the ring off", async () => {
        const onChange = vi.fn();
        renderSection(enabledValues, { onChange });

        await userEvent.click(screen.getByRole("switch"));

        expect(onChange).toHaveBeenCalledWith("led_enabled", false);
    });

    // Both of these will bite the operator before anything else does: the
    // device does not exist until the overlay is enabled and the robot
    // rebooted, and the spidevB.C numbering is kernel-assigned so the default
    // path is only a guess.
    it("names the device-tree overlay and how to find the real device", () => {
        renderSection(enabledValues);
        expect(screen.getByText("rk3588-spi4-m0-cs1-spidev.dtbo")).toBeInTheDocument();
        expect(screen.getByText("ls /dev/spidev*")).toBeInTheDocument();
        expect(screen.getByText(/reboot/i)).toBeInTheDocument();
    });

    // The 3.3 V data line into a 5 V ring is below WS2812B's VIH and is the
    // first thing to suspect once the software is correct. Saying so here is
    // cheaper than the support question.
    it("warns that wrong colours are a 3.3 V level problem, not a software one", () => {
        renderSection(enabledValues);
        expect(screen.getByText(/3\.3 V/)).toBeInTheDocument();
        expect(screen.getByText(/level shifter/i)).toBeInTheDocument();
    });

    it("exposes the hardware fields once enabled", () => {
        renderSection(enabledValues);
        expect(screen.getByText("LED count")).toBeInTheDocument();
        expect(screen.getByText("SPI device")).toBeInTheDocument();
        expect(screen.getByText("SPI clock")).toBeInTheDocument();
        expect(screen.getByDisplayValue("/dev/spidev4.1")).toBeInTheDocument();
    });

    it("exposes the appearance and threshold fields once enabled", () => {
        renderSection(enabledValues);
        for (const label of [
            "Brightness",
            "Idle brightness",
            "Refresh rate",
            "Low battery threshold",
            "Charge complete",
            "Status timeout",
            "Keepalive",
            "Device retry",
        ]) {
            expect(screen.getByText(label)).toBeInTheDocument();
        }
    });

    it("edits the LED count, which is a placeholder rather than a measurement", async () => {
        const onChange = vi.fn();
        renderSection(enabledValues, { onChange });

        const countInput = screen.getAllByRole("spinbutton")[0];
        await userEvent.clear(countInput);
        await userEvent.type(countInput, "24");

        expect(onChange).toHaveBeenCalledWith("led_count", 24);
    });

    it("edits the SPI device path", async () => {
        const onChange = vi.fn();
        renderSection(enabledValues, { onChange });

        const deviceInput = screen.getByDisplayValue("/dev/spidev4.1");
        await userEvent.type(deviceInput, "0");

        expect(onChange).toHaveBeenCalledWith("led_spi_device", "/dev/spidev4.10");
    });

    // The ring is not self-explanatory: without the legend the operator has to
    // read the C++ to know what amber means.
    it("explains every pattern the ring can show", () => {
        renderSection(enabledValues);
        for (const mode of [
            "Emergency",
            "Charging",
            "No status",
            "Low battery",
            "Mowing",
            "Mowing without RTK fix",
            "Recording a boundary",
            "Manual mowing",
            "Idle",
        ]) {
            expect(screen.getByText(mode)).toBeInTheDocument();
        }
    });

    it("distinguishes the two red patterns by motion, not colour", () => {
        renderSection(enabledValues);
        expect(screen.getByText(/solid red, not moving/i)).toBeInTheDocument();
        expect(screen.getByText(/blinking is what tells them apart/i)).toBeInTheDocument();
    });

    it("offers reset-to-default on an overridden field", async () => {
        const onReset = vi.fn();
        renderSection(enabledValues, {
            isOverridden: (key) => key === "led_count",
            hasDefault: () => true,
            onReset,
        });

        const resetButtons = screen.getAllByRole("button", { name: /reset to default/i });
        expect(resetButtons).toHaveLength(1);

        await userEvent.click(resetButtons[0]);
        expect(onReset).toHaveBeenCalledWith("led_count");
    });

    it("renders plainly when the settings manager knows no defaults", () => {
        renderSection(enabledValues);
        expect(screen.queryByRole("button", { name: /reset to default/i })).not.toBeInTheDocument();
    });
});
