import {act, cleanup, fireEvent, render, screen, waitFor} from '@testing-library/react';
import {afterEach, beforeEach, expect, it, vi} from 'vitest';

import {MemoryRouter} from 'react-router-dom';
import {ConfigProvider} from 'antd';
import type {SettingsValues} from '../types/settings.generated';
import {getColors} from '../theme/colors';
import en from '../i18n/locales/en.json';

const mock = vi.hoisted(() => {
    const values: SettingsValues = {datum_lat: 12, datum_lon: 34, mower_model: 'YardForce500'};
    return {
        values, fetch: vi.fn<typeof fetch>(), save: vi.fn<(values: SettingsValues) => Promise<boolean>>(),
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
    mock.fetch.mockReset();
    mock.values.imu_yaw = .1;
    mock.save.mockResolvedValue(true);
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
vi.mock('../hooks/useCalibrationStatus.ts',()=>({useCalibrationStatus:()=>({status:{dock:{present:true},imu:{present:true},mag:{present:true}},refresh:async()=>{}})}));
vi.mock('../components/settings/SensorsSection.tsx',()=>({SensorsSection:()=>null}));
vi.mock('../components/RobotComponentEditor.tsx',()=>({RobotComponentEditor:()=>null}));
beforeEach(()=>{vi.stubGlobal('fetch',mock.fetch);});afterEach(()=>{vi.unstubAllGlobals();});
async function calibrationStep() {
    await datumStep();
    await waitFor(() => expect(screen.getByText(en.onboardingPage.saveAndFinish, {exact: true}).closest('button')).not.toHaveClass('ant-btn-loading'));
    clickText(en.onboardingPage.saveAndFinish);
    await screen.findByRole('region', {name: en.onboardingPage.stepSensors});
    await waitFor(() => expect(screen.getByText(en.onboardingPage.next, {exact: true}).closest('button')).not.toHaveClass('ant-btn-loading'));
    clickText(en.onboardingPage.next);
    await screen.findByRole('region', {name: en.onboardingPage.stepCalibration});
    mock.fetch.mockClear();
    mock.save.mockClear();
}
function pending() {
    let resolve!: (value: Response) => void;
    let reject!: (error: unknown) => void;
    const promise = new Promise<Response>((yes, no) => {resolve = yes; reject = no;});
    return {promise, resolve, reject};
}
const valid = {success: true, message: 'Complete', imu_yaw_rad: .25, imu_yaw_deg: 14.32, samples_used: 250, std_dev_deg: .2};
function expectNoAutomaticApply() {
    expect(mock.save).not.toHaveBeenCalled();
    expect(mock.partial).not.toHaveBeenCalled();
    expect(mock.restart).not.toHaveBeenCalled();
    expect(mock.success).not.toHaveBeenCalled();
}

it('ignores a late error after Start then Back to the real Sensors step', async () => {
    const request = pending();
    mock.fetch.mockReturnValue(request.promise);
    await calibrationStep();
    const start = clickText(en.onboardingPage.startImuYawCalibration);
    expect(start).toBeDisabled();
    fireEvent.click(start);
    expect(mock.fetch).toHaveBeenCalledOnce();
    expect(screen.queryByText(en.onboardingPage.applyToSettings, {exact: true})).not.toBeInTheDocument();
    expect(screen.queryByText(en.onboardingPage.discard, {exact: true})).not.toBeInTheDocument();
    clickText(en.onboardingPage.back);
    await screen.findByRole('region', {name: en.onboardingPage.stepSensors});
    await act(async () => {request.reject(new TypeError('Late calibration connection loss')); await Promise.resolve();});
    expect(mock.error).not.toHaveBeenCalled();
    expectNoAutomaticApply();
    expect(mock.fetch).toHaveBeenCalledExactlyOnceWith('/api/calibration/imu-yaw',
        expect.objectContaining({method: 'POST', body: '{"duration_sec":30}'}));
    expect(mock.fetch.mock.calls[0][1]?.signal?.aborted).toBe(false);
});
it.each(['success', 'error'])('keeps the new instance active after a late old %s', async outcome => {
    const old = pending(), current = pending();
    mock.fetch.mockReturnValueOnce(old.promise).mockReturnValueOnce(current.promise);
    await calibrationStep();
    clickText(en.onboardingPage.startImuYawCalibration);
    clickText(en.onboardingPage.back);
    await screen.findByRole('region', {name: en.onboardingPage.stepSensors});
    clickText(en.onboardingPage.next);
    await screen.findByRole('region', {name: en.onboardingPage.stepCalibration});
    await waitFor(() => expect(screen.getByText(en.onboardingPage.next, {exact: true}).closest('button')).not.toHaveClass('ant-btn-loading'));
    // Next's normal save belongs to leaving Sensors, before either request completes.
    mock.save.mockClear();
    const start = clickText(en.onboardingPage.startImuYawCalibration);
    expect(mock.fetch).toHaveBeenCalledTimes(2);
    await act(async () => {
        if (outcome === 'success') old.resolve(new Response(JSON.stringify({...valid, imu_yaw_deg: 5.73})));
        else old.reject(new TypeError('Old instance failure'));
        await Promise.resolve();
    });
    expect(start).toBeDisabled();
    expect(screen.queryByText(en.onboardingPage.applyToSettings, {exact: true})).not.toBeInTheDocument();
    expect(mock.error).not.toHaveBeenCalled();
    expectNoAutomaticApply();
    await act(async () => {current.resolve(new Response(JSON.stringify(valid))); await Promise.resolve();});
    await screen.findByText('imu_yaw = 14.32° (σ ±0.20°)');
    expect(screen.getByText(en.onboardingPage.applyToSettings, {exact: true})).toBeVisible();
    expect(screen.queryByText('imu_yaw = 5.73° (σ ±0.20°)')).not.toBeInTheDocument();
    expectNoAutomaticApply();
    for (const [url, options] of mock.fetch.mock.calls) {
        expect(url).toBe('/api/calibration/imu-yaw');
        expect(options).toMatchObject({method: 'POST', body: '{"duration_sec":30}'});
        expect(options?.signal?.aborted).toBe(false);
    }
});
it('preserves Reset, Start, Discard and explicit Apply with no automatic save', async () => {
    mock.fetch.mockResolvedValueOnce(new Response(JSON.stringify({success: false, message: 'Not enough samples'})));
    await calibrationStep();
    clickText(en.onboardingPage.startImuYawCalibration);
    await screen.findByText('Not enough samples');
    clickText(en.onboardingPage.reset);
    expect(mock.fetch).toHaveBeenCalledOnce();
    mock.fetch.mockImplementation(() => Promise.resolve(new Response(JSON.stringify(valid))));
    clickText(en.onboardingPage.startImuYawCalibration);
    await screen.findByText(en.onboardingPage.applyToSettings, {exact: true});
    expectNoAutomaticApply();
    clickText(en.onboardingPage.discard);
    expect(mock.fetch).toHaveBeenCalledTimes(2);
    expect(screen.queryByText(en.onboardingPage.applyToSettings, {exact: true})).not.toBeInTheDocument();
    clickText(en.onboardingPage.startImuYawCalibration);
    await screen.findByText(en.onboardingPage.applyToSettings, {exact: true});
    expectNoAutomaticApply();
    clickText(en.onboardingPage.applyToSettings);
    expect(mock.success).toHaveBeenCalledOnce();
    expect(mock.error).not.toHaveBeenCalled();
    expect(mock.fetch).toHaveBeenCalledTimes(3);
    expect(mock.save).not.toHaveBeenCalled();
    expect(mock.partial).not.toHaveBeenCalled();
    expect(screen.getByText('14.32°', {exact: true})).toBeInTheDocument();
});

it('stays on Calibration with its applied draft after a failed save and permits retry', async () => {
    mock.fetch.mockImplementation(() => Promise.resolve(new Response(JSON.stringify(valid))));
    await calibrationStep();
    clickText(en.onboardingPage.startImuYawCalibration);
    await screen.findByText(en.onboardingPage.applyToSettings, {exact: true});
    clickText(en.onboardingPage.applyToSettings);
    mock.save.mockResolvedValueOnce(false);
    const next = clickText(en.onboardingPage.next);
    await waitFor(() => expect(next).not.toHaveClass('ant-btn-loading'));
    expect(screen.getByRole('region', {name: en.onboardingPage.stepCalibration})).toBeVisible();
    expect(mock.save).toHaveBeenLastCalledWith(expect.objectContaining({imu_yaw: .25}));
    expect(mock.restart).not.toHaveBeenCalled();
    clickText(en.onboardingPage.next);
    await screen.findByRole('region', {name: en.onboardingPage.stepComplete});
    expect(mock.save).toHaveBeenCalledTimes(2);
    expect(mock.save).toHaveBeenLastCalledWith(expect.objectContaining({imu_yaw: .25}));
    expect(mock.restart).not.toHaveBeenCalled();
});
