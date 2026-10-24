import {cleanup, fireEvent, render, screen, waitFor} from '@testing-library/react';
import {afterEach, beforeEach, describe, expect, it, vi} from 'vitest';
import type {SettingsValues} from '../../types/settings.generated';
import {PositioningSection} from './PositioningSection';
import en from '../../i18n/locales/en.json';
const mocks = vi.hoisted(() => ({fetch: vi.fn<typeof globalThis.fetch>(), change: vi.fn<(key: string, value: string | number | null) => void>(),
    persist: vi.fn<(values: SettingsValues) => Promise<boolean>>(), save: vi.fn(), restart: vi.fn(), notification: {success: vi.fn(), error: vi.fn()}}));
vi.mock('../../hooks/useApi.ts', async () => {
    const {Api} = await import('../../api/Api');
    const api = new Api<unknown>({customFetch: mocks.fetch}); api.baseUrl = '';
    return {useApi: () => api};
});
vi.mock('../../hooks/useGnssStatus.ts', () => ({useGnssStatus: () => null}));
vi.mock('../gnss/GnssLiveStatusSummaryCard.tsx', () => ({GnssLiveStatusSummaryCard: () => null}));
vi.mock('./UniversalGnssAdvancedSettings.tsx', () => ({UniversalGnssAdvancedSettings: () => null}));
vi.mock('./GnssSerialDeviceConfigField.tsx', () => ({GnssSerialDeviceConfigField: () => null}));
vi.mock('./GnssReceiverActionsCard.tsx', () => ({GnssReceiverActionsCard: ({onPersistBeforeAction}: {onPersistBeforeAction: () => Promise<boolean>}) =>
    <button onClick={() => {void onPersistBeforeAction();}}>Persist fixture</button>}));
vi.mock('antd', async original => ({...await original<typeof import('antd')>(), App: {useApp: () => ({notification: mocks.notification})}}));
const json = (value: unknown, status = 200) => new Response(JSON.stringify(value), {status});
function renderSection(values: SettingsValues = {datum_lat: 12, datum_lon: 34}) {
    return render(<PositioningSection values={values} onChange={mocks.change} isDirty={false} saving={false} gpsRestarting={false}
        onSave={mocks.save} onSaveAndRestartGps={mocks.restart} onPersistGnssSettings={mocks.persist}/>);
}
async function capture() {
    fireEvent.click(screen.getByRole('button', {name: en.settingsPositioning.useCurrentGpsPosition}));
    await waitFor(() => expect(mocks.notification.success.mock.calls.length + mocks.notification.error.mock.calls.length).toBe(1));
}
beforeEach(() => {vi.clearAllMocks(); mocks.persist.mockResolvedValue(true); mocks.fetch.mockImplementation(() => Promise.resolve(json({message: '48.123456789,-1.987654321'})));});
afterEach(cleanup);

describe('set_datum response integrity', () => {
    it.each(['48.123456789,-1.987654321', '0.000000000,0.000000000', '-90.000000000,180.000000000'])('adopts a complete valid coordinate pair without rounding or implicit save: %s', async message => {
        mocks.fetch.mockImplementation(() => Promise.resolve(json({message}))); renderSection(); await capture();
        const [lat, lon] = message.split(',').map(Number);
        expect(mocks.change.mock.calls).toEqual([['datum_lat', lat], ['datum_lon', lon]]);
        expect(mocks.notification.success).toHaveBeenCalledOnce(); expect(mocks.notification.error).not.toHaveBeenCalled();
        expect(mocks.fetch).toHaveBeenCalledOnce();
        expect(mocks.fetch).toHaveBeenCalledWith('/mowglinext/call/set_datum', expect.objectContaining({method: 'POST', body: '{}'}));
        expect(mocks.persist).not.toHaveBeenCalled(); expect(mocks.save).not.toHaveBeenCalled(); expect(mocks.restart).not.toHaveBeenCalled();
    });
    it.each([null, [], {}, {message: 3}, {message: ['0,0']}, {message: ''}, {message: '48'}, {message: '1,2,3'},
        {message: 'NaN,0'}, {message: '0,Infinity'}, {message: '1e999,0'}, {message: '1e-400,0'}, {message: '91,0'}, {message: '0,-181'},
        {message: '48north,2'}, {message: ',0'}, {message: '0x10,0'}, {message: '0,0', error: 'rejected'}, {message: '0,0', success: false},
    ].map(value => [value]))('rejects malformed or invalid coordinates without any local change: %j', async raw => {
        mocks.fetch.mockImplementation(() => Promise.resolve(json(raw))); renderSection(); await capture();
        expect(mocks.change).not.toHaveBeenCalled(); expect(mocks.notification.success).not.toHaveBeenCalled();
        expect(screen.getByLabelText(en.settingsPositioning.latitude)).toHaveValue('12.000000000');
        expect(screen.getByLabelText(en.settingsPositioning.longitude)).toHaveValue('34.000000000');
        expect(mocks.fetch).toHaveBeenCalledOnce(); expect(mocks.persist).not.toHaveBeenCalled();
    });
    it.each([json({error: 'No RTK-fixed GPS fix available'}, 500), new Response('broken JSON', {status: 200}), new Response('broken JSON', {status: 500})])('handles HTTP and generated-client decoding failures without false success', async response => {
        mocks.fetch.mockResolvedValue(response); renderSection(); await capture();
        expect(mocks.change).not.toHaveBeenCalled(); expect(mocks.notification.success).not.toHaveBeenCalled(); expect(mocks.fetch).toHaveBeenCalledOnce();
    });
    it('surfaces the structured backend refusal', async () => {
        mocks.fetch.mockImplementation(() => Promise.resolve(json({error: 'No GPS fix received yet'}, 500)));
        renderSection(); await capture(); expect(mocks.notification.error).toHaveBeenCalledWith(expect.objectContaining({description: 'No GPS fix received yet'}));
    });
    it('handles transport loss without a retry, implicit save or success', async () => {
        mocks.fetch.mockRejectedValue(new TypeError('Connection lost')); renderSection(); await capture();
        expect(mocks.change).not.toHaveBeenCalled(); expect(mocks.notification.success).not.toHaveBeenCalled(); expect(mocks.fetch).toHaveBeenCalledOnce();
        expect(mocks.persist).not.toHaveBeenCalled();
    });
    it('retains null on manual coordinate clearing and does not save it implicitly', () => {
        renderSection(); fireEvent.change(screen.getByLabelText(en.settingsPositioning.latitude), {target: {value: ''}});
        expect(mocks.change).toHaveBeenCalledWith('datum_lat', null);
        expect(mocks.fetch).not.toHaveBeenCalled(); expect(mocks.persist).not.toHaveBeenCalled(); expect(mocks.save).not.toHaveBeenCalled();
    });
    it('uses the existing GNSS allowlist without filling absent keys or modifying zero, null or empty values', async () => {
        renderSection({gnss_receiver_family: 'auto', gnss_config_baud: 0, gnss_profile_rate_hz: null, gnss_serial_device: '', datum_lat: 12, ntrip_password: 'fake-unrelated'});
        fireEvent.click(screen.getByRole('switch', {name: en.onboardingPage.expertMode}));
        expect(mocks.persist).not.toHaveBeenCalled(); expect(mocks.fetch).not.toHaveBeenCalled();
        fireEvent.click(screen.getByRole('button', {name: 'Persist fixture'}));
        await waitFor(() => expect(mocks.persist).toHaveBeenCalledExactlyOnceWith({gnss_receiver_family: 'auto', gnss_config_baud: 0, gnss_profile_rate_hz: null, gnss_serial_device: ''}));
        expect(mocks.fetch).not.toHaveBeenCalled(); expect(mocks.change).not.toHaveBeenCalled();
    });
});
