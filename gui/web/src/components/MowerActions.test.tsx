import {beforeEach, describe, expect, it, vi} from 'vitest';
import {act, fireEvent, render, renderHook, screen, waitFor} from '@testing-library/react';
import {App} from 'antd';
import {MowerActions, useMowerAction} from './MowerActions.tsx';
import en from '../i18n/locales/en.json';

const callCreate = vi.fn();
vi.mock('../hooks/useApi.ts', () => ({
    useApi: () => ({mowglinext: {callCreate}}),
}));
vi.mock('../hooks/useHighLevelStatus.ts', () => ({
    useHighLevelStatus: () => ({highLevelStatus: {state: 4, state_name: 'MOWING'}}),
}));
vi.mock('../hooks/useCoverageResumeAvailable.ts', () => ({useCoverageResumeAvailable: () => false}));
vi.mock('../theme/ThemeContext.tsx', () => ({useThemeMode: () => ({colors: {}})}));

describe('map blade controls', () => {
    beforeEach(() => callCreate.mockReset().mockResolvedValue({}));

    it('shows partial OFF success as a warning', async () => {
        callCreate.mockResolvedValue({data: {warning: 'OFF requested. The mower may turn the blade back on.'}});
        render(<App><MowerActions bare/></App>);
        fireEvent.click(screen.getByText(en.mowerActions.more));
        fireEvent.click(await screen.findByText(en.mowerActions.bladeOff));
        expect(await screen.findByText('OFF requested. The mower may turn the blade back on.')).toBeVisible();
    });

    it.each([
        [en.mowerActions.bladeForward, 1, 0],
        [en.mowerActions.bladeBackward, 1, 1],
        [en.mowerActions.bladeOff, 0, 0],
    ])('routes %s through the session blade controller', async (label, enabled, direction) => {
        render(<App><MowerActions bare/></App>);
        fireEvent.click(screen.getByText(en.mowerActions.more));
        fireEvent.click(await screen.findByText(label));
        await waitFor(() => expect(callCreate).toHaveBeenCalledExactlyOnceWith('blade_control', {
            mow_enabled: enabled, mow_direction: direction,
        }));
    });
});

describe('useMowerAction controller correlation', () => {
    it('forwards optional request parameters to the existing service route', async () => {
        callCreate.mockReset().mockResolvedValue({});
        const {result} = renderHook(() => useMowerAction(), {wrapper: App});
        const params = {headers: {'X-Mowgli-Gamepad-Session': 'opaque-session'}};
        await act(async () => { await result.current('high_level_control', {Command: 3}, params)(); });
        expect(callCreate).toHaveBeenCalledExactlyOnceWith('high_level_control', {Command: 3}, params);
    });
});
