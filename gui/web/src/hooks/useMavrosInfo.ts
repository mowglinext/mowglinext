import {useEffect, useState} from "react";
import {getMultiplexedSocket} from "./multiplexedSocket.ts";

export type MavrosState = {connected?: boolean; mode?: string; system_status?: number};

export type MavrosVehicleInfo = {
    available_info?: number;
    sysid?: number;
    compid?: number;
    autopilot?: number;
    type?: number;
    flight_sw_version?: number;
    flight_custom_version?: string;
    board_version?: number;
    vendor_id?: number;
    product_id?: number;
};

type VehicleInfoResponse = {success?: boolean; vehicles?: MavrosVehicleInfo[]};
export type MavrosInfo = {state: MavrosState; vehicle: MavrosVehicleInfo | null};

const AUTOPILOTS: Record<number, string> = {0: "Generic", 3: "ArduPilot", 12: "PX4"};
const VEHICLE_TYPES: Record<number, string> = {1: "Fixed wing", 2: "Quadrotor", 10: "Ground rover"};

export const mavrosAutopilotName = (value?: number): string | null =>
    value == null ? null : AUTOPILOTS[value] ?? `MAV_AUTOPILOT ${value}`;

export const mavrosVehicleTypeName = (value?: number): string | null =>
    value == null ? null : VEHICLE_TYPES[value] ?? `MAV_TYPE ${value}`;

export const formatMavrosFirmwareVersion = (vehicle: MavrosVehicleInfo | null): string | null => {
    if (!vehicle || ((vehicle.available_info ?? 0) & 2) === 0) return null;
    const raw = Number(vehicle.flight_sw_version ?? 0) >>> 0;
    const version = raw === 0 ? null : `${(raw >>> 24) & 0xff}.${(raw >>> 16) & 0xff}.${(raw >>> 8) & 0xff}`;
    const custom = vehicle.flight_custom_version?.replace(/\0/g, "").trim();
    return [version, custom].filter(Boolean).join(" · ") || null;
};

const hex = (value: number, width: number): string => `0x${(value >>> 0).toString(16).padStart(width, "0")}`;

export const formatMavrosBoardIdentity = (vehicle: MavrosVehicleInfo | null): string | null => {
    if (!vehicle || ((vehicle.available_info ?? 0) & 2) === 0) return null;
    const parts: string[] = [];
    if (vehicle.board_version) parts.push(`HW ${hex(vehicle.board_version, 8)}`);
    if (vehicle.vendor_id || vehicle.product_id) {
        parts.push(`VID ${hex(vehicle.vendor_id ?? 0, 4)} / PID ${hex(vehicle.product_id ?? 0, 4)}`);
    }
    return parts.join(" · ") || null;
};

export function useMavrosInfo(enabled = true): MavrosInfo {
    const [state, setState] = useState<MavrosState>({});
    const [vehicle, setVehicle] = useState<MavrosVehicleInfo | null>(null);
    useEffect(() => {
        if (!enabled) return;
        const socket = getMultiplexedSocket();
        const unsubscribeState = socket.subscribe("mavrosState", raw => {
            if (raw && typeof raw === "object") setState(raw as MavrosState);
        });
        const unsubscribeVehicle = socket.subscribe("mavrosVehicleInfo", raw => {
            if (!raw || typeof raw !== "object") return;
            const response = raw as VehicleInfoResponse;
            if (response.success !== false) setVehicle(response.vehicles?.[0] ?? null);
        });
        return () => {unsubscribeState(); unsubscribeVehicle();};
    }, [enabled]);
    return {state, vehicle};
}
