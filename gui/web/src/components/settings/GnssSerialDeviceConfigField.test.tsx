import { App } from "antd";
import { render, screen } from "@testing-library/react";
import userEvent from "@testing-library/user-event";
import { beforeEach, describe, expect, it, vi } from "vitest";
import en from "../../i18n/locales/en.json";
import { GnssSerialDeviceConfigField } from "./GnssSerialDeviceConfigField.tsx";

const useGnssRuntimeConfigMock = vi.fn();

vi.mock("../../hooks/useGnssRuntimeConfig.ts", () => ({
    useGnssRuntimeConfig: () => useGnssRuntimeConfigMock(),
}));

const renderField = (props?: Partial<React.ComponentProps<typeof GnssSerialDeviceConfigField>>) => render(
    <App>
        <GnssSerialDeviceConfigField
            value={undefined}
            onChange={vi.fn()}
            selectedReceiverFamily="auto"
            selectedReceiverModel=""
            gnssStatus={undefined}
            {...props}
        />
    </App>,
);

describe("GnssSerialDeviceConfigField", () => {
    beforeEach(() => {
        useGnssRuntimeConfigMock.mockReset();
        Object.defineProperty(window, "matchMedia", {
            writable: true,
            value: vi.fn().mockImplementation((query: string) => ({
                matches: false,
                media: query,
                onchange: null,
                addListener: vi.fn(),
                removeListener: vi.fn(),
                addEventListener: vi.fn(),
                removeEventListener: vi.fn(),
                dispatchEvent: vi.fn(),
            })),
        });
    });

    it("surfaces .env fallback usage and lets the operator pick a detected device", async () => {
        useGnssRuntimeConfigMock.mockReturnValue({
            runtimeConfig: {
                receiver_family: { active_value: "unicore", source: "env" },
                serial_device: {
                    active_value: "/dev/serial/by-id/usb-fallback",
                    source: "env",
                },
                serial_baud: { active_value: "460800", source: "env" },
                serial_devices: [
                    {
                        path: "/dev/serial/by-id/usb-fallback",
                        display_label: "usb-fallback -> /dev/ttyUSB0",
                        resolved_path: "/dev/ttyUSB0",
                        exists: true,
                    },
                    {
                        path: "/dev/serial/by-id/usb-next",
                        display_label: "usb-next -> /dev/ttyUSB1",
                        resolved_path: "/dev/ttyUSB1",
                        exists: true,
                    },
                ],
                active_serial_device_exists: true,
                active_serial_resolved_path: "/dev/ttyUSB0",
            },
            error: null,
        });

        const onChange = vi.fn();
        const user = userEvent.setup();
        renderField({ onChange });

        expect(screen.getAllByText(/\.env fallback/i).length).toBeGreaterThan(0);
        expect(screen.getAllByText(/usb-fallback/i).length).toBeGreaterThan(0);
        expect(screen.getByText("460800")).toBeInTheDocument();

        await user.click(screen.getByRole("combobox"));
        await user.click(await screen.findByText("usb-next -> /dev/ttyUSB1"));

        expect(onChange).toHaveBeenCalledWith(
            "/dev/serial/by-id/usb-next",
            expect.objectContaining({ value: "/dev/serial/by-id/usb-next" }),
        );
    });

    it("shows clear missing-device and receiver mismatch warnings", () => {
        useGnssRuntimeConfigMock.mockReturnValue({
            runtimeConfig: {
                receiver_family: { active_value: "unicore", source: "config" },
                serial_device: {
                    configured_value: "/dev/serial/by-id/usb-missing",
                    active_value: "/dev/serial/by-id/usb-missing",
                    source: "config",
                },
                serial_baud: { active_value: "921600", source: "config" },
                serial_devices: [],
                configured_serial_device_exists: false,
                active_serial_device_exists: false,
            },
            error: null,
        });

        renderField({
            value: "/dev/serial/by-id/usb-missing",
            selectedReceiverFamily: "unicore",
            selectedReceiverModel: "UM982",
            gnssStatus: {
                receiver_vendor: "u-blox",
                receiver_model: "F9P",
            },
        });

        expect(screen.getByText(en.gnssRuntimeConfig.configuredDeviceMissing.replace("{{device}}", "/dev/serial/by-id/usb-missing"))).toBeInTheDocument();
        expect(screen.getByText(/selected family unicore/i)).toBeInTheDocument();
        expect(screen.getByText(/selected model UM982/i)).toBeInTheDocument();
    });
});
