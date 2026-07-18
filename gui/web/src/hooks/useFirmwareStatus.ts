import {useStatus} from "./useStatus.ts";

export interface FirmwareStatus {
    /**
     * Firmware <-> image compatibility from the hardware_bridge handshake.
     * `null` until the first Status arrives, so callers can stay quiet rather
     * than flashing a false "incompatible" on load.
     */
    firmwareCompatible: boolean | null;
    firmwareVersion: string;
}

/**
 * Shared selector for the firmware handshake fields on /status. Both the
 * onboarding Firmware step and the readiness gate need the same signal, so
 * the small projection that used to live inline in MowgliNextPage is factored
 * out here to avoid duplicating the subscription logic.
 */
export const useFirmwareStatus = (): FirmwareStatus => {
    const status = useStatus();
    return {
        firmwareCompatible: status.firmware_compatible ?? null,
        firmwareVersion: status.firmware_version ?? "",
    };
};
