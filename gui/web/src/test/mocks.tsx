import React from 'react';
import {vi} from 'vitest';
import type {GnssStatus, HighLevelStatus} from '../types/ros.ts';
import {GnssStatusConstants} from '../types/ros.ts';

// Mock high level status
export const defaultHighLevelStatus: HighLevelStatus = {
    state_name: 'IDLE',
    gps_quality_percent: 0.95,
    battery_percent: 0.75,
    is_charging: false,
    emergency: false,
};

export const gnssStatusSampleNames = [
    'dual_antenna_um982_solved',
    'baseline_unavailable_unknown',
    'ntrip_startup_waiting',
    'correction_stream_active',
    'correction_stream_unavailable',
    'correction_stream_error',
    'msm_summary_present',
    'msm_malformed_not_decoded',
    'nmea_gga_fix_quality_float',
] as const;

export type GnssStatusSampleName = typeof gnssStatusSampleNames[number];

const RTK_MODE_CAPABILITY = GnssStatusConstants.CAP_RTK_MODE;
const BASELINE_CAPABILITIES =
    GnssStatusConstants.CAP_DUAL_ANTENNA_BASELINE |
    GnssStatusConstants.CAP_BASELINE_AZIMUTH |
    GnssStatusConstants.CAP_BASELINE_PITCH |
    GnssStatusConstants.CAP_BASELINE_LENGTH |
    GnssStatusConstants.CAP_BASELINE_SOLUTION_STATUS;
const CORRECTION_STREAM_CAPABILITY = GnssStatusConstants.CAP_CORRECTION_STREAM;
const MSM_SUMMARY_CAPABILITY = GnssStatusConstants.CAP_MSM_SUMMARY;

export const gnssStatusSamples: Record<GnssStatusSampleName, GnssStatus> = {
    dual_antenna_um982_solved: {
        backend: 'universal',
        receiver_vendor: 'Unicore',
        receiver_model: 'UM982',
        receiver_firmware: 'R4.10.11833',
        fix_type: GnssStatusConstants.FIX_TYPE_RTK_FIXED,
        fix_valid: true,
        rtk_mode: GnssStatusConstants.RTK_MODE_FIXED,
        dual_antenna_baseline: true,
        baseline_azimuth_deg: 184.32,
        baseline_pitch_deg: -0.85,
        baseline_length_m: 1.247,
        baseline_solution_status: GnssStatusConstants.BASELINE_STATUS_COMPUTED,
        correction_stream_status: GnssStatusConstants.CORRECTION_STREAM_STATUS_ACTIVE,
        msm_summary_seen: true,
        msm_summary_decoded: true,
        msm_summary_valid: true,
        msm_summary_message_type: 7,
        msm_summary_station_id: 4095,
        msm_summary_constellations_seen: 'GPS+GLO+GAL',
        msm_summary_satellite_count: 18,
        msm_summary_signal_count: 28,
        msm_summary_cell_count: 52,
        msm_summary_age_s: 0.4,
        quality_percent: 100,
        capability_flags: RTK_MODE_CAPABILITY | BASELINE_CAPABILITIES | CORRECTION_STREAM_CAPABILITY | MSM_SUMMARY_CAPABILITY,
        value_flags: RTK_MODE_CAPABILITY | BASELINE_CAPABILITIES | CORRECTION_STREAM_CAPABILITY | MSM_SUMMARY_CAPABILITY,
    },
    baseline_unavailable_unknown: {
        backend: 'universal',
        receiver_vendor: 'Unicore',
        receiver_model: 'UM982',
        fix_type: GnssStatusConstants.FIX_TYPE_GPS_FIX,
        fix_valid: true,
        rtk_mode: GnssStatusConstants.RTK_MODE_NONE,
        quality_percent: 25,
        capability_flags: RTK_MODE_CAPABILITY | BASELINE_CAPABILITIES,
        value_flags: RTK_MODE_CAPABILITY,
    },
    ntrip_startup_waiting: {
        backend: 'universal',
        receiver_vendor: 'u-blox',
        receiver_model: 'F9P',
        fix_type: GnssStatusConstants.FIX_TYPE_GPS_FIX,
        fix_valid: true,
        rtk_mode: GnssStatusConstants.RTK_MODE_NONE,
        correction_stream_status: GnssStatusConstants.CORRECTION_STREAM_STATUS_WAITING,
        quality_percent: 25,
        capability_flags: RTK_MODE_CAPABILITY | CORRECTION_STREAM_CAPABILITY,
        value_flags: RTK_MODE_CAPABILITY | CORRECTION_STREAM_CAPABILITY,
    },
    correction_stream_active: {
        backend: 'universal',
        receiver_vendor: 'u-blox',
        receiver_model: 'F9P',
        fix_type: GnssStatusConstants.FIX_TYPE_RTK_FLOAT,
        fix_valid: true,
        rtk_mode: GnssStatusConstants.RTK_MODE_FLOAT,
        correction_stream_status: GnssStatusConstants.CORRECTION_STREAM_STATUS_ACTIVE,
        quality_percent: 50,
        capability_flags: RTK_MODE_CAPABILITY | CORRECTION_STREAM_CAPABILITY,
        value_flags: RTK_MODE_CAPABILITY | CORRECTION_STREAM_CAPABILITY,
    },
    correction_stream_unavailable: {
        backend: 'universal',
        receiver_vendor: 'u-blox',
        receiver_model: 'F9P',
        fix_type: GnssStatusConstants.FIX_TYPE_GPS_FIX,
        fix_valid: true,
        rtk_mode: GnssStatusConstants.RTK_MODE_NONE,
        correction_stream_status: GnssStatusConstants.CORRECTION_STREAM_STATUS_UNAVAILABLE,
        quality_percent: 25,
        capability_flags: RTK_MODE_CAPABILITY | CORRECTION_STREAM_CAPABILITY,
        value_flags: RTK_MODE_CAPABILITY | CORRECTION_STREAM_CAPABILITY,
    },
    correction_stream_error: {
        backend: 'universal',
        receiver_vendor: 'u-blox',
        receiver_model: 'F9P',
        fix_type: GnssStatusConstants.FIX_TYPE_GPS_FIX,
        fix_valid: true,
        rtk_mode: GnssStatusConstants.RTK_MODE_NONE,
        correction_stream_status: GnssStatusConstants.CORRECTION_STREAM_STATUS_ERROR,
        quality_percent: 10,
        capability_flags: RTK_MODE_CAPABILITY | CORRECTION_STREAM_CAPABILITY,
        value_flags: RTK_MODE_CAPABILITY | CORRECTION_STREAM_CAPABILITY,
    },
    msm_summary_present: {
        backend: 'universal',
        receiver_vendor: 'u-blox',
        receiver_model: 'F9P',
        fix_type: GnssStatusConstants.FIX_TYPE_RTK_FLOAT,
        fix_valid: true,
        rtk_mode: GnssStatusConstants.RTK_MODE_FLOAT,
        correction_stream_status: GnssStatusConstants.CORRECTION_STREAM_STATUS_ACTIVE,
        msm_summary_seen: true,
        msm_summary_decoded: true,
        msm_summary_valid: true,
        msm_summary_message_type: 7,
        msm_summary_station_id: 1024,
        msm_summary_constellations_seen: 'GPS+GAL',
        msm_summary_satellite_count: 14,
        msm_summary_signal_count: 22,
        msm_summary_cell_count: 31,
        msm_summary_age_s: 0.6,
        capability_flags: RTK_MODE_CAPABILITY | CORRECTION_STREAM_CAPABILITY | MSM_SUMMARY_CAPABILITY,
        value_flags: RTK_MODE_CAPABILITY | CORRECTION_STREAM_CAPABILITY | MSM_SUMMARY_CAPABILITY,
    },
    msm_malformed_not_decoded: {
        backend: 'universal',
        receiver_vendor: 'u-blox',
        receiver_model: 'F9P',
        fix_type: GnssStatusConstants.FIX_TYPE_GPS_FIX,
        fix_valid: true,
        rtk_mode: GnssStatusConstants.RTK_MODE_NONE,
        correction_stream_status: GnssStatusConstants.CORRECTION_STREAM_STATUS_ACTIVE,
        msm_summary_seen: true,
        msm_summary_decoded: false,
        msm_summary_valid: false,
        msm_summary_message_type: 0,
        msm_summary_station_id: 0,
        msm_summary_constellations_seen: 'GPS',
        msm_summary_satellite_count: 0,
        msm_summary_signal_count: 0,
        msm_summary_cell_count: 0,
        msm_summary_age_s: 1.8,
        capability_flags: RTK_MODE_CAPABILITY | CORRECTION_STREAM_CAPABILITY | MSM_SUMMARY_CAPABILITY,
        value_flags: RTK_MODE_CAPABILITY | CORRECTION_STREAM_CAPABILITY | MSM_SUMMARY_CAPABILITY,
    },
    nmea_gga_fix_quality_float: {
        backend: 'universal',
        receiver_vendor: 'Generic',
        receiver_model: 'NMEA',
        fix_type: GnssStatusConstants.FIX_TYPE_GPS_FIX,
        fix_valid: true,
        rtk_mode: GnssStatusConstants.RTK_MODE_FLOAT,
        quality_percent: 50,
        capability_flags: RTK_MODE_CAPABILITY,
        value_flags: RTK_MODE_CAPABILITY,
    },
};

// Mock useHighLevelStatus hook
export const mockHighLevelStatus = vi.fn(() => ({
    highLevelStatus: defaultHighLevelStatus,
}));

// Mock useApi hook
export const mockGuiApi = {
    mowglinext: {
        callCreate: vi.fn().mockResolvedValue({}),
    },
    maps: {
        mapCreate: vi.fn().mockResolvedValue({}),
    },
    config: {
        configCreate: vi.fn().mockResolvedValue({}),
        configList: vi.fn().mockResolvedValue({data: {}}),
    },
};

export const mockUseApi = vi.fn(() => mockGuiApi);

// Mock useWS hook
export const createMockWS = () => ({
    start: vi.fn(),
    stop: vi.fn(),
    sendJsonMessage: vi.fn(),
});

export const mockUseWS = vi.fn(() => createMockWS());

// Mock notification
export const mockNotification = {
    success: vi.fn(),
    error: vi.fn(),
    info: vi.fn(),
    warning: vi.fn(),
};

// Mock App.useApp
export const mockUseApp = vi.fn(() => ({
    notification: mockNotification,
    message: {success: vi.fn(), error: vi.fn()},
    modal: {confirm: vi.fn()},
}));

// Mock react-router-dom
export const mockNavigate = vi.fn();
export const mockUseMatches = vi.fn(() => [
    {pathname: '/'},
    {pathname: '/mowglinext'},
]);

// Wrapper for rendering with providers
export function TestWrapper({children}: {children: React.ReactNode}) {
    return <>{children}</>;
}
