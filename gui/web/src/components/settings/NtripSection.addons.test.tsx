import {cleanup, fireEvent, render, screen} from '@testing-library/react';
import {afterEach, beforeEach, expect, it, vi, type MockInstance} from 'vitest';
import {useState} from 'react';
import {NtripSection} from './NtripSection';
import {getColors} from '../../theme/colors';
import type {SettingsValues} from '../../types/settings.generated';
import en from '../../i18n/locales/en.json';
const mock = vi.hoisted(() => ({api: {request: vi.fn<(args: {method: string; path: string}) => Promise<{data: {stations: unknown[]}}>>() }, change: vi.fn(), mobile: false}));
vi.mock('../../hooks/useApi.ts', () => ({useApi: () => mock.api}));
vi.mock('../../hooks/useIsMobile.ts', () => ({useIsMobile: () => mock.mobile}));
vi.mock('../../theme/ThemeContext.tsx', () => ({useThemeMode: () => ({colors: getColors('dark')})}));
vi.mock('./NtripStationMap.tsx', () => ({NtripStationMap: () => null}));
let errors: MockInstance<typeof console.error>, warnings: MockInstance<typeof console.warn>;
beforeEach(() => {
    mock.api.request.mockReset().mockResolvedValue({data: {stations: []}}); mock.change.mockClear();
    errors = vi.spyOn(console, 'error'); warnings = vi.spyOn(console, 'warn');
});
afterEach(() => {
    cleanup();
    for (const spy of [errors, warnings]) expect(spy.mock.calls.flat().join(' ')).not.toMatch(/\[antd: Input\].*addon(Before|After)/);
    vi.restoreAllMocks();
});
function Harness() {
    const [values, setValues] = useState<SettingsValues>({ntrip_enabled: true, ntrip_host: '', ntrip_user: 'fictional-user', ntrip_password: 'fake-secret'});
    return <NtripSection values={values} onChange={(key: string, value: string | number | boolean | null) => {
        mock.change(key, value); setValues(previous => ({...previous, [key]: value}));
    }}/>;
}
it.each([false, true])('keeps NTRIP raw and provider credentials controlled at mobile=%s', async mobile => {
    mock.mobile = mobile; render(<Harness/>);
    fireEvent.click(screen.getByRole('button', {name: /SAPOS/}));
    fireEvent.click(screen.getByText(en.settingsNtrip.advancedRawFields));
    const hosts = await screen.findAllByLabelText(en.settingsNtrip.host);
    const initialRequests = mock.api.request.mock.calls.length;
    fireEvent.change(hosts[0], {target: {value: 'caster.example.test'}});
    expect(hosts.every(input => input instanceof HTMLInputElement && input.value === 'caster.example.test')).toBe(true);
    const passwords = screen.getAllByLabelText(en.settingsNtrip.pass);
    fireEvent.change(passwords[0], {target: {value: 'fake-updated'}});
    expect(passwords.every(input => input instanceof HTMLInputElement && input.value === 'fake-updated')).toBe(true);
    expect(passwords.every(input => input.getAttribute('type') === 'password')).toBe(true);
    const changes = mock.change.mock.calls.length;
    fireEvent.click(screen.getAllByRole('img', {name: 'eye-invisible'})[0]);
    expect(passwords[0]).toHaveAttribute('type', 'text'); expect(passwords[1]).toHaveAttribute('type', 'password');
    expect(mock.change).toHaveBeenCalledTimes(changes);
    fireEvent.change(screen.getAllByLabelText(en.settingsNtrip.user)[1], {target: {value: 'fictional-edited'}});
    expect(screen.getAllByLabelText(en.settingsNtrip.user).every(input => input instanceof HTMLInputElement && input.value === 'fictional-edited')).toBe(true);
    fireEvent.change(screen.getByLabelText(en.settingsNtrip.mountpoint), {target: {value: 'FAKE_BASE'}});
    expect(mock.change).toHaveBeenLastCalledWith('ntrip_mountpoint', 'FAKE_BASE');
    // Editing/toggling alone performs no configuration write or reload request.
    expect(mock.api.request).toHaveBeenCalledTimes(initialRequests);
    expect(mock.api.request.mock.calls.every(([args]) => args.method === 'GET')).toBe(true);
});
it('preserves the email addon and the disabled NTRIP view', async () => {
    render(<Harness/>);
    fireEvent.click(screen.getByRole('button', {name: /RTK2go/}));
    const email = await screen.findByLabelText(en.settingsNtrip.emailAddon);
    fireEvent.change(email, {target: {value: 'fictional@example.test'}});
    expect(mock.change).toHaveBeenLastCalledWith('ntrip_user', 'fictional@example.test');
    fireEvent.click(screen.getByRole('switch', {name: en.settingsNtrip.title}));
    expect(screen.queryByLabelText(en.settingsNtrip.emailAddon)).not.toBeInTheDocument();
    expect(mock.change).toHaveBeenLastCalledWith('ntrip_enabled', false);
});
