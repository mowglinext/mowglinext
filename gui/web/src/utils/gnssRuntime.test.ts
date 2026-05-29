import { describe, expect, it } from "vitest";
import {
    isRuntimeUnicore,
    resolveDisplayedGpsProtocol,
    sanitizeYamlGnssValues,
} from "./gnssRuntime.ts";

describe("gnssRuntime helpers", () => {
    const unicoreRuntime = {
        backend: "unicore",
        hardware_backend: "mowgli",
        protocol: "UNICORE",
        connection: "usb",
        port: "/dev/serial/by-id/usb-Unicore_UM980",
        by_id: "/dev/serial/by-id/usb-Unicore_UM980",
        baud: "921600",
        frame_id: "gps_link",
        has_config: true,
        source: "compose env",
    };

    it("recognizes unicore from the runtime contract", () => {
        expect(isRuntimeUnicore(unicoreRuntime)).toBe(true);
        expect(resolveDisplayedGpsProtocol("UBX", unicoreRuntime)).toBe("UNICORE");
    });

    it("sanitizes a GUI save so runtime unicore is not degraded to UBX or NMEA", () => {
        const payload = sanitizeYamlGnssValues(
            {
                gps_protocol: "UBX",
                gps_port: "/dev/gps",
                gps_baudrate: 460800,
                ntrip_enabled: true,
            },
            unicoreRuntime,
        );

        expect(payload.gps_protocol).toBe("UNICORE");
        expect(payload.gps_port).toBe("/dev/serial/by-id/usb-Unicore_UM980");
        expect(payload.gps_baudrate).toBe(921600);
        expect(payload.ntrip_enabled).toBe(true);
    });

    it("injects the universal unicore contract into partial GUI saves", () => {
        const payload = sanitizeYamlGnssValues(
            {
                ntrip_host: "caster.example.test",
            },
            unicoreRuntime,
        );

        expect(payload.gps_protocol).toBe("UNICORE");
        expect(payload.gps_port).toBe("/dev/serial/by-id/usb-Unicore_UM980");
        expect(payload.gps_baudrate).toBe(921600);
        expect(payload.ntrip_host).toBe("caster.example.test");
    });

    it("leaves generic payloads unchanged when runtime backend is not unicore", () => {
        const payload = { gps_protocol: "NMEA", gps_port: "/dev/gps", gps_baudrate: 115200 };
        expect(sanitizeYamlGnssValues(payload, {
            backend: "gps",
            hardware_backend: "mowgli",
            protocol: "NMEA",
            connection: "uart",
            port: "/dev/gps",
            by_id: "",
            baud: "115200",
            frame_id: "gps_link",
            has_config: true,
            source: "compose env",
        })).toEqual(payload);
    });
});
