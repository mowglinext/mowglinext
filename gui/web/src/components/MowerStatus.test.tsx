import {describe, it, expect, vi, beforeEach} from 'vitest';
import {fireEvent, render, screen} from '@testing-library/react';
import {App} from 'antd';
import {MowerStatus} from './MowerStatus.tsx';
// Assert against the active locale (pinned to English in test/setup.ts) rather
// than hardcoded copy, so reworded translations don't break these tests.
import en from '../i18n/locales/en.json';
import {GnssStatusConstants, type Status, type HighLevelStatus, type GnssStatus} from '../types/ros.ts';

// MowerStatus reads snake_case rosbridge fields and pulls GPS from
// useGnssStatus / battery from usePower+useSettings — NOT from highLevelStatus.
// All hooks are mocked so the test exercises the real state-label + derivation
// logic in isolation.
const mockHighLevelStatus = vi.fn<() => {highLevelStatus: HighLevelStatus}>();
vi.mock('../hooks/useHighLevelStatus.ts', () => ({
    useHighLevelStatus: () => mockHighLevelStatus(),
}));
const mockStatus = vi.fn<() => Status>();
vi.mock('../hooks/useStatus.ts', () => ({useStatus: () => mockStatus()}));
const mockEmergency = vi.fn<() => import('../utils/statusTelemetry').EmergencyDisplay>();
vi.mock('../hooks/useEmergency.ts', () => ({useEmergency: () => mockEmergency()}));
vi.mock('../hooks/usePower.ts', () => ({usePower: () => ({v_battery: 0})}));
const mockGnss = vi.fn<() => GnssStatus>();
vi.mock('../hooks/useGnssStatus.ts', () => ({useGnssStatus: () => mockGnss()}));
vi.mock('../hooks/useSettings.ts', () => ({useSettings: () => ({settings: {}})}));
vi.mock('../hooks/useApi.ts', () => ({useApi: () => ({request: vi.fn()})}));
const rearm = vi.hoisted(() => vi.fn());
vi.mock('./MowerActions.tsx', () => ({useMowerAction: () => () => rearm}));
vi.mock('../theme/ThemeContext.tsx', () => ({
    useThemeMode: () => ({
        colors: {primary: '#52c41a', warning: '#faad14', danger: '#ff4d4f', text: '#000', muted: '#888'},
    }),
}));

const renderStatus = () => render(<App><MowerStatus/></App>);

describe('MowerStatus', () => {
    it('does not invent idle from a hardware initialization or OK report', () => {
        mockHighLevelStatus.mockReturnValue({highLevelStatus: {}});
        mockStatus.mockReturnValue({mower_status: 255});
        renderStatus();
        expect(screen.getAllByText(en.telemetry.statusUnknown).length).toBeGreaterThan(0);
        expect(screen.queryByText(en.utils.stateAtBase)).not.toBeInTheDocument();
    });

    it('does not let a false high-level bit mask an active hardware emergency', () => {
        mockHighLevelStatus.mockReturnValue({highLevelStatus: {state: 1, state_name: 'IDLE', emergency: false}});
        mockEmergency.mockReturnValue({active_emergency: true});
        renderStatus();
        expect(screen.getByText(en.utils.stateEmergencyStop)).toBeInTheDocument();
        expect(screen.getByText(en.mowerStatus.rearm)).toBeInTheDocument();
        expect(rearm).not.toHaveBeenCalled();
    });

    it('shows the latch when the active bit is missing without sending a reset', () => {
        mockHighLevelStatus.mockReturnValue({highLevelStatus: {}});
        mockEmergency.mockReturnValue({latched_emergency: true});
        renderStatus();
        expect(screen.getByText(en.utils.stateEmergencyStop)).toBeInTheDocument();
        expect(rearm).not.toHaveBeenCalled();
    });

    it('labels replayed clearance as unverified rather than a currently cleared emergency', () => {
        mockHighLevelStatus.mockReturnValue({highLevelStatus: {state: 1, state_name: 'IDLE', emergency: false}});
        mockEmergency.mockReturnValue({active_emergency: false, latched_emergency: false, clearanceUnverified: true});
        renderStatus();
        expect(screen.getByText(en.telemetry.emergencyClearanceUnverified)).toBeInTheDocument();
        expect(screen.queryByText(en.utils.stateIdle)).not.toBeInTheDocument();
        expect(rearm).not.toHaveBeenCalled();
    });

    it('qualifies a reported state instead of claiming proven acquisition freshness', () => {
        mockHighLevelStatus.mockReturnValue({highLevelStatus: {state: 2, state_name: 'MOWING'}});
        renderStatus();
        expect(screen.getByText(en.telemetry.freshnessUnverified)).toBeInTheDocument();
    });

    beforeEach(() => {
        rearm.mockReset();
        mockStatus.mockReturnValue({});
        mockEmergency.mockReturnValue({active_emergency: false, latched_emergency: false});
        mockGnss.mockReturnValue({
            fix_valid: true,
            fix_type: GnssStatusConstants.FIX_TYPE_RTK_FIXED,
        });
    });

    it('notifies the operator when rearming is rejected', async () => {
        const log = vi.spyOn(console, 'error').mockImplementation(() => undefined);
        mockHighLevelStatus.mockReturnValue({highLevelStatus: {state_name: 'IDLE', emergency: true}});
        rearm.mockRejectedValueOnce(new Error('Firmware refused rearm'));
        renderStatus();
        fireEvent.click(screen.getByText(en.mowerStatus.rearm));
        expect(await screen.findByText('Firmware refused rearm')).toBeInTheDocument();
        expect(rearm).toHaveBeenCalledOnce();
        expect(screen.queryByText(/success/i)).not.toBeInTheDocument();
        log.mockRestore();
    });

    it('displays idle state', () => {
        mockHighLevelStatus.mockReturnValue({
            highLevelStatus: {state_name: 'IDLE', battery_percent: 75, is_charging: false},
        });
        renderStatus();
        expect(screen.getByText(en.utils.stateIdle)).toBeInTheDocument();
    });

    it('displays mowing state', () => {
        mockHighLevelStatus.mockReturnValue({
            highLevelStatus: {state_name: 'MOWING', battery_percent: 50, is_charging: false},
        });
        renderStatus();
        expect(screen.getByText(en.utils.stateMowing)).toBeInTheDocument();
    });

    it('falls back to Offline when state_name is absent', () => {
        mockHighLevelStatus.mockReturnValue({highLevelStatus: {}});
        renderStatus();
        expect(screen.getAllByText(en.telemetry.statusUnknown).length).toBeGreaterThan(0);
    });

    it('renders the compact canonical GNSS solution label', () => {
        mockHighLevelStatus.mockReturnValue({highLevelStatus: {state_name: 'IDLE'}});
        mockGnss.mockReturnValue({
            fix_valid: true,
            fix_type: GnssStatusConstants.FIX_TYPE_DGPS,
        });
        renderStatus();
        expect(screen.getByText(`60% · ${en.gpsStatus.compactDgps}`)).toBeInTheDocument();
    });
});
