import {describe, it, expect, vi, beforeEach} from 'vitest';
import {render, screen} from '@testing-library/react';
import {App} from 'antd';
import {MowerStatus} from './MowerStatus.tsx';
// Assert against the active locale (pinned to English in test/setup.ts) rather
// than hardcoded copy, so reworded translations don't break these tests.
import en from '../i18n/locales/en.json';
import {GnssStatusConstants} from '../types/ros.ts';

// MowerStatus reads snake_case rosbridge fields and pulls GPS from
// useGnssStatus / battery from usePower+useSettings — NOT from highLevelStatus.
// All hooks are mocked so the test exercises the real state-label + derivation
// logic in isolation.
const mockHighLevelStatus = vi.fn();
vi.mock('../hooks/useHighLevelStatus.ts', () => ({
    useHighLevelStatus: () => mockHighLevelStatus(),
}));
vi.mock('../hooks/useStatus.ts', () => ({useStatus: () => ({})}));
vi.mock('../hooks/useEmergency.ts', () => ({
    useEmergency: () => ({active_emergency: false, latched_emergency: false}),
}));
vi.mock('../hooks/usePower.ts', () => ({usePower: () => ({v_battery: 0})}));
const mockGnss = vi.fn();
vi.mock('../hooks/useGnssStatus.ts', () => ({useGnssStatus: () => mockGnss()}));
vi.mock('../hooks/useSettings.ts', () => ({useSettings: () => ({settings: {}})}));
vi.mock('../hooks/useApi.ts', () => ({useApi: () => ({request: vi.fn()})}));
vi.mock('./MowerActions.tsx', () => ({useMowerAction: () => () => vi.fn()}));
vi.mock('../theme/ThemeContext.tsx', () => ({
    useThemeMode: () => ({
        colors: {primary: '#52c41a', warning: '#faad14', danger: '#ff4d4f', text: '#000', muted: '#888'},
    }),
}));

const renderStatus = () => render(<App><MowerStatus/></App>);

describe('MowerStatus', () => {
    beforeEach(() => {
        mockGnss.mockReturnValue({
            fix_valid: true,
            fix_type: GnssStatusConstants.FIX_TYPE_RTK_FIXED,
        });
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
        expect(screen.getByText(en.utils.stateOffline)).toBeInTheDocument();
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
