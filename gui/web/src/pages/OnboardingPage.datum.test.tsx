import {cleanup, fireEvent, render, screen, waitFor} from '@testing-library/react';
import {afterEach, beforeEach, describe, expect, it, vi} from 'vitest';
import {MemoryRouter} from 'react-router-dom';
import {ConfigProvider} from 'antd';
import type {SettingsValues} from '../types/settings.generated';
import {getColors} from '../theme/colors';
import en from '../i18n/locales/en.json';

const mock = vi.hoisted(() => {
    const values: SettingsValues = {datum_lat: 12, datum_lon: 34, mower_model: 'YardForce500'};
    return {
        values, fetch: vi.fn<typeof fetch>(), save: vi.fn<(values: SettingsValues) => Promise<void>>(),
        partial: vi.fn(), restart: vi.fn(), error: vi.fn(), success: vi.fn(), warning: vi.fn(),
    };
});
vi.mock('../hooks/useApi.ts', async () => {
    const {Api} = await import('../api/Api');
    const api = new Api<unknown>({customFetch: mock.fetch});
    api.baseUrl = '';
    return {useApi: () => api};
});
vi.mock('../hooks/useSettingsSchema.ts', () => ({useSettingsSchema: () => ({
    values: mock.values, saveValues: mock.save, savePartialValues: mock.partial, loading: false,
})}));
vi.mock('../hooks/useGnssStatus.ts', async () => {
    const {GnssStatusConstants: G} = await import('../types/ros');
    return {useGnssStatus: () => ({fix_type: G.FIX_TYPE_RTK_FIXED, rtk_mode: G.RTK_MODE_FIXED, fix_valid: true})};
});
vi.mock('../hooks/useFirmwareStatus.ts', () => ({useFirmwareStatus: () => ({firmwareCompatible: true, firmwareVersion: 'fixture'})}));
vi.mock('../hooks/useContainerRestart.ts', () => ({useContainerRestart: () => ({pending: false, run: mock.restart})}));
vi.mock('../hooks/useIsMobile', () => ({useIsMobile: () => false}));
vi.mock('../theme/ThemeContext.tsx', () => ({useThemeMode: () => ({colors: getColors('dark')})}));
vi.mock('../components/settings/NtripSection.tsx', () => ({NtripSection: () => null}));
vi.mock('../components/settings/UniversalGnssAdvancedSettings.tsx', () => ({UniversalGnssAdvancedSettings: () => null}));
vi.mock('../components/settings/GnssSerialDeviceConfigField.tsx', () => ({GnssSerialDeviceConfigField: () => null}));
vi.mock('../components/gnss/GnssLiveStatusSummaryCard.tsx', () => ({GnssLiveStatusSummaryCard: () => null}));
vi.mock('antd', async importOriginal => {
    const antd = await importOriginal<typeof import('antd')>();
    return {...antd, App: {...antd.App, useApp: () => ({notification: {
        error: mock.error, success: mock.success, warning: mock.warning,
    }})}};
});

import OnboardingPage from './OnboardingPage';

const scrollDescriptor = Object.getOwnPropertyDescriptor(HTMLElement.prototype, 'scrollTo');
beforeEach(() => {
    vi.clearAllMocks();
    mock.save.mockResolvedValue(undefined);
    mock.fetch.mockImplementation(() => Promise.resolve(new Response(JSON.stringify({message: '48.123456789,-1.987654321'}))));
    Object.defineProperty(HTMLElement.prototype, 'scrollTo', {configurable: true, value: vi.fn()});
});
afterEach(() => {
    cleanup();
    if (scrollDescriptor) Object.defineProperty(HTMLElement.prototype, 'scrollTo', scrollDescriptor);
    else Reflect.deleteProperty(HTMLElement.prototype, 'scrollTo');
});
const latitude = () => screen.getByLabelText(en.onboardingPage.latitudeLabel);
const longitude = () => screen.getByLabelText(en.onboardingPage.longitudeLabel);
function clickText(label: string) {
    const button = screen.getByText(label, {exact: true}).closest('button');
    if (!button) throw new Error(`Missing button ${label}`);
    fireEvent.click(button);
    return button;
}
async function datumStep() {
    render(<MemoryRouter initialEntries={['/onboarding?step=firmware']}>
        <ConfigProvider theme={{token: {motion: false}}}><OnboardingPage/></ConfigProvider>
    </MemoryRouter>);
    clickText(en.onboardingPage.skipAlreadyFlashed);
    await screen.findByRole('region', {name: en.onboardingPage.stepNtrip});
    clickText(en.onboardingPage.next);
    await screen.findByRole('region', {name: en.onboardingPage.stepGps});
    clickText(en.onboardingPage.next);
    await screen.findByRole('region', {name: en.onboardingPage.stepDatum});
    expect(latitude()).toHaveValue('12.000000000');
    expect(longitude()).toHaveValue('34.000000000');
    expect(mock.save).toHaveBeenCalledTimes(2);
    mock.save.mockClear();
    expect(mock.fetch).not.toHaveBeenCalled();
}
function expectNoPersistence() {
    expect(mock.save).not.toHaveBeenCalled();
    expect(mock.partial).not.toHaveBeenCalled();
    expect(mock.restart).not.toHaveBeenCalled();
    expect(mock.success).not.toHaveBeenCalled();
    expect(mock.fetch).toHaveBeenCalledExactlyOnceWith('/mowglinext/call/set_datum', expect.objectContaining({method: 'POST', body: '{}'}));
    expect(screen.getByRole('region', {name: en.onboardingPage.stepDatum})).toBeVisible();
}

const malformed = [
    '48north,2east', 'NaN,2', '48,', '91,181', '48,181', '-91,2', '48,-181', '48,NaN', ',2', '48,Infinity',
    '1e999,2', '1e-400,2', '0,1e-400', '0x10,2', '48', '48,2,3', ' ',
];
describe('Onboarding GNSS datum response boundary', () => {
    it.each(malformed)('rejects %s before either field changes', async message => {
        mock.fetch.mockImplementation(() => Promise.resolve(new Response(JSON.stringify({message}))));
        await datumStep();
        const button = clickText(en.onboardingPage.useCurrentGpsPosition);
        await waitFor(() => expect(button).not.toHaveClass('ant-btn-loading'));
        expect(latitude()).toHaveValue('12.000000000');
        expect(longitude()).toHaveValue('34.000000000');
        expect(mock.error).toHaveBeenCalledOnce();
        const notice: unknown = mock.error.mock.calls[0][0];
        expect(notice).toHaveProperty('message', en.onboardingPage.datumSetFailedTitle);
        expect(notice).toHaveProperty('description', expect.any(String));
        expectNoPersistence();
    });

    it.each([
        '48.123456789,-1.987654321', '0.000000000,0.000000000', '-90,180', '90,-180', '-12.5,-34.5',
        '  +48.125 , -2.5 ', '4.8e1,2e0', '0e-400,2',
    ])('preserves valid coordinates %s without saving or navigating', async message => {
        mock.fetch.mockImplementation(() => Promise.resolve(new Response(JSON.stringify({message, additional: true}))));
        await datumStep();
        clickText(en.onboardingPage.useCurrentGpsPosition);
        const [lat, lon] = message.split(',').map(Number);
        await waitFor(() => expect(latitude()).toHaveValue(lat.toFixed(9)));
        expect(longitude()).toHaveValue(lon.toFixed(9));
        expect(mock.error).not.toHaveBeenCalled();
        expectNoPersistence();
    });

    it.each([null, [], {}, {message: null}, {message: 3}, {message: ['48,2']},
        {message: {lat: 48, lon: 2}}, {message: '48,2', success: false}, {message: '48,2', error: 'Refused'},
    ].map(payload => [payload]))('rejects malformed envelope %j without partial changes', async payload => {
        mock.fetch.mockImplementation(() => Promise.resolve(new Response(JSON.stringify(payload))));
        await datumStep();
        clickText(en.onboardingPage.useCurrentGpsPosition);
        await waitFor(() => expect(mock.error).toHaveBeenCalledOnce());
        expect(latitude()).toHaveValue('12.000000000');
        expect(longitude()).toHaveValue('34.000000000');
        expectNoPersistence();
    });

    it.each([
        {name: 'HTTP 500 textual error', text: JSON.stringify({error: 'No GPS fix received yet'}), status: 500, detail: 'No GPS fix received yet'},
        {name: 'HTTP 400 textual error', text: JSON.stringify({error: 'Request refused'}), status: 400, detail: 'Request refused'},
        {name: 'HTTP 500 error with coordinates', text: JSON.stringify({error: 'Refused', message: '48,2'}), status: 500, detail: 'Refused'},
        {name: 'HTTP 500 empty body', text: '', status: 500},
        {name: 'HTTP 500 invalid JSON', text: 'not JSON', status: 500},
        {name: 'HTTP 500 empty error', text: JSON.stringify({error: ''}), status: 500},
        {name: 'HTTP 500 null error', text: JSON.stringify({error: null}), status: 500},
        {name: 'HTTP 500 numeric error', text: JSON.stringify({error: 42}), status: 500},
        {name: 'HTTP 500 object error', text: JSON.stringify({error: {detail: 'Refused'}}), status: 500},
        {name: 'HTTP 500 no detail', text: '{}', status: 500},
        {name: 'HTTP 500 null body', text: 'null', status: 500},
        {name: 'HTTP 200 invalid JSON', text: 'not JSON', status: 200},
    ])('handles $name through the existing notification', async ({text, status, detail}) => {
        mock.fetch.mockImplementation(() => Promise.resolve(new Response(text, {status})));
        await datumStep();
        const button = clickText(en.onboardingPage.useCurrentGpsPosition);
        await waitFor(() => expect(mock.error).toHaveBeenCalledOnce());
        expect(mock.error).toHaveBeenCalledExactlyOnceWith({
            message: en.onboardingPage.datumSetFailedTitle,
            description: detail ?? en.onboardingPage.datumSetFailedDesc,
        });
        expect(latitude()).toHaveValue('12.000000000');
        expect(longitude()).toHaveValue('34.000000000');
        await waitFor(() => expect(button).not.toHaveClass('ant-btn-loading'));
        expect(button).toBeEnabled();
        expectNoPersistence();
    });

    it.each([
        {name: 'textual primitive', rejection: 'Direct refusal', detail: 'Direct refusal'},
        {name: 'empty primitive', rejection: '', detail: en.onboardingPage.datumSetFailedDesc},
        {name: 'native Error', rejection: new Error('Native refusal'), detail: 'Native refusal'},
        {name: 'non-textual message', rejection: {message: {detail: 'Refused'}}, detail: en.onboardingPage.datumSetFailedDesc},
    ])('handles $name without coercing non-textual content', async ({rejection, detail}) => {
        mock.fetch.mockRejectedValue(rejection);
        await datumStep();
        const button = clickText(en.onboardingPage.useCurrentGpsPosition);
        await waitFor(() => expect(mock.error).toHaveBeenCalledExactlyOnceWith({
            message: en.onboardingPage.datumSetFailedTitle, description: detail,
        }));
        expect(latitude()).toHaveValue('12.000000000');
        expect(longitude()).toHaveValue('34.000000000');
        await waitFor(() => expect(button).not.toHaveClass('ant-btn-loading'));
        expect(button).toBeEnabled();
        expectNoPersistence();
    });

    it('allows a deliberate retry after HTTP refusal with one command per click', async () => {
        mock.fetch.mockResolvedValueOnce(new Response(JSON.stringify({error: 'Refused'}), {status: 500}));
        await datumStep();
        const button = clickText(en.onboardingPage.useCurrentGpsPosition);
        await waitFor(() => expect(mock.error).toHaveBeenCalledOnce());
        await waitFor(() => expect(button).not.toHaveClass('ant-btn-loading'));
        expect(latitude()).toHaveValue('12.000000000');
        expect(longitude()).toHaveValue('34.000000000');
        expectNoPersistence();
        expect(button).toBeEnabled();
        clickText(en.onboardingPage.useCurrentGpsPosition);
        await waitFor(() => expect(latitude()).toHaveValue('48.123456789'));
        expect(longitude()).toHaveValue('-1.987654321');
        expect(mock.fetch).toHaveBeenCalledTimes(2);
        for (const call of mock.fetch.mock.calls) {
            expect(call).toEqual(['/mowglinext/call/set_datum', expect.objectContaining({method: 'POST', body: '{}'})]);
        }
        expect(mock.error).toHaveBeenCalledOnce();
        expect(mock.success).not.toHaveBeenCalled();
        expect(mock.save).not.toHaveBeenCalled();
        expect(mock.partial).not.toHaveBeenCalled();
        expect(mock.restart).not.toHaveBeenCalled();
        expect(screen.getByRole('region', {name: en.onboardingPage.stepDatum})).toBeVisible();
    });

    it('preserves the network error detail without retry or partial change', async () => {
        mock.fetch.mockRejectedValue(new TypeError('Connection lost'));
        await datumStep();
        clickText(en.onboardingPage.useCurrentGpsPosition);
        await waitFor(() => expect(mock.error).toHaveBeenCalledExactlyOnceWith({
            message: en.onboardingPage.datumSetFailedTitle, description: 'Connection lost',
        }));
        expect(latitude()).toHaveValue('12.000000000');
        expect(longitude()).toHaveValue('34.000000000');
        expectNoPersistence();
    });
});
