import {cleanup, fireEvent, render, screen} from '@testing-library/react';
import {afterEach, beforeEach, describe, expect, it, vi} from 'vitest';
import {ConfigProvider} from 'antd';
import {RainSection} from './RainSection';
import type {ExternalRainState} from '../../hooks/useExternalRainState';
import en from '../../i18n/locales/en.json';

const mock = vi.hoisted(() => ({state: null as ExternalRainState | null}));
vi.mock('../../hooks/useExternalRainState.ts', () => ({useExternalRainState: () => mock.state}));
vi.mock('../../theme/ThemeContext.tsx', () => ({useThemeMode: () => ({colors: {}})}));
afterEach(cleanup);
beforeEach(() => {mock.state = null;});

describe('RainSection merged numeric and boolean contracts', () => {
    it('preserves the two compact inputs and emits a boolean from the external Switch', () => {
        const change = vi.fn();
        const view = render(<ConfigProvider theme={{token: {motion: false}}}><RainSection
            values={{rain_mode: 2, rain_delay_minutes: 30, rain_debounce_sec: 5, use_external_rain: false, external_rain_max_age_minutes: 120}}
            onChange={change}/></ConfigProvider>);
        expect(view.container.querySelectorAll('.ant-space-compact')).toHaveLength(2);
        expect(screen.getByRole('spinbutton', {name: `${en.settingsRain.resumeDelay}, min`})).toHaveValue('30');
        expect(screen.getByRole('spinbutton', {name: `${en.settingsRain.debounce}, s`})).toHaveValue('5');
        expect(screen.getByRole('spinbutton', {name: `${en.settingsRain.externalExpiry}, min`})).toBeDisabled();
        fireEvent.click(screen.getByRole('switch', {name: en.settingsRain.useExternal}));
        expect(change).toHaveBeenCalledExactlyOnceWith('use_external_rain', true);
        view.rerender(<RainSection values={{rain_mode: 2, use_external_rain: true}} onChange={change}/>);
        change.mockClear();
        fireEvent.click(screen.getByRole('switch', {name: en.settingsRain.useExternal}));
        expect(change).toHaveBeenCalledExactlyOnceWith('use_external_rain', false);
    });
    it('keeps null and numeric edits for existing fields and the unchanged expiration input', () => {
        const change = vi.fn();
        render(<RainSection values={{rain_mode: 2, rain_delay_minutes: 30, rain_debounce_sec: 5, use_external_rain: true, external_rain_max_age_minutes: 120}} onChange={change}/>);
        const inputs = [
            ['rain_delay_minutes', `${en.settingsRain.resumeDelay}, min`],
            ['rain_debounce_sec', `${en.settingsRain.debounce}, s`],
            ['external_rain_max_age_minutes', `${en.settingsRain.externalExpiry}, min`],
        ];
        for (const [key, label] of inputs) {
            change.mockClear();
            fireEvent.change(screen.getByRole('spinbutton', {name: label}), {target: {value: ''}});
            expect(change).toHaveBeenLastCalledWith(key, null);
            fireEvent.change(screen.getByRole('spinbutton', {name: label}), {target: {value: '15'}});
            expect(change).toHaveBeenLastCalledWith(key, 15);
        }
    });
    it.each(['wet', 'dry', 'expired', 'no_data', 'disabled'] as const)('renders external state %s from the incoming hook contract', state => {
        mock.state = {state, enabled: true, received: false, wet: false, active: false, onboard_wet: false, age_s: null, expires_in_s: null, max_age_s: 0};
        render(<RainSection values={{rain_mode: 2}} onChange={vi.fn()}/>);
        expect(screen.getByText(en.settingsRain[`externalState_${state}`])).toBeVisible();
    });
});
