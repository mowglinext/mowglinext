import {act, cleanup, fireEvent, render, screen, waitFor} from '@testing-library/react';
import {afterEach, beforeEach, describe, expect, it, vi} from 'vitest';
import type {ModalFuncProps} from 'antd';
import {RobotComponentEditor} from './RobotComponentEditor';
import {useSettingsManager} from '../hooks/useSettingsManager';
import {SensorsSection} from './settings/SensorsSection';
import {isJsonObject} from '../utils/json';
import {getColors} from '../theme/colors';

const mock = vi.hoisted(() => ({
    api: {settings: {yamlList: vi.fn(), yamlCreate: vi.fn()}, mowglinext: {mapDockingCreate: vi.fn()}, request: vi.fn()},
    notification: {success: vi.fn<(config: {message?: unknown; description?: unknown}) => void>(), warning: vi.fn<(config: {message?: unknown; description?: unknown}) => void>(), error: vi.fn(), info: vi.fn()},
    modal: {confirm: vi.fn<(props: ModalFuncProps) => void>()},
    charging: true, x: 10, y: 20, yaw: 0,
}));
vi.mock('antd', async importOriginal => ({...await importOriginal<typeof import('antd')>(), App: {useApp: () => ({notification: mock.notification, modal: mock.modal})}}));
vi.mock('../hooks/useHardwareBackend.ts', () => ({useHardwareBackend: () => ({authority: 'confirmed', runtimeRouting: 'unavailable', backend: 'mowgli', parameterRoutes: {}, defaultOverrides: {}, loading: false})}));
vi.mock('../hooks/useContainerRestart.ts', () => ({useContainerRestart: () => ({pending: false, run: vi.fn()})}));
vi.mock('../hooks/useApi.ts', () => ({useApi: () => mock.api}));
vi.mock('../hooks/useStatus.ts', () => ({useStatus: () => ({is_charging: mock.charging})}));
vi.mock('../hooks/usePose.ts', () => ({usePose: () => ({pose: {pose: {position: {x: mock.x, y: mock.y}}}, motion_heading: mock.yaw})}));
vi.mock('../hooks/useCalibrationStatus.ts', () => ({useCalibrationStatus: () => ({status: null})}));
vi.mock('../hooks/useImuYawCalibration.ts', () => ({useImuYawCalibration: () => ({calibOpen: false, calibRunning: false, calibResult: null})}));
vi.mock('../hooks/useRobotDescription.ts', () => ({useRobotDescription: () => ({baseLength: .54, baseWidth: .4, baseHeight: .19, chassisCenterX: .18,
    wheelRadius: .04475, wheelWidth: .04, wheelTrack: .325, wheelXOffset: 0, casterRadius: .03, casterXOffset: .4, casterTrack: .36, bladeRadius: .09, fromUrdf: true, sensors: ['lidar', 'imu', 'gps'].map(id => ({id, x: 0, y: 0, z: .1, roll: 0, pitch: 0, yaw: 0, length: .04, width: .04, height: .04}))})}));
vi.mock('../theme/ThemeContext.tsx', () => ({useThemeMode: () => ({colors: getColors('light'), mode: 'light'})}));
vi.mock('./robot/LayeredMower', () => ({LayeredMower: () => null, SensorGraphic: () => null}));
vi.mock('./robot/MowerPreview', () => ({MowerView: () => null, MowerVisualControls: () => null}));
vi.mock('./robot/DockGraphic', () => ({DockGraphic: () => null}));

const stored = (x = 1.123456, y = 2, yaw = .1234567) => ({data: {dock: {present: true, pose_values_valid: true, dock_pose_x: x, dock_pose_y: y, dock_pose_yaw_rad: yaw}}});
const values = {lidar_enabled: true, dock_pose_x: 3, dock_pose_y: 4, dock_pose_yaw: 1, lidar_x: 0, lidar_y: 0, lidar_z: 0, lidar_yaw: 0};
async function confirmCapture() {
    fireEvent.click(screen.getByRole('button', {name: /Set dock position$/}));
    expect(mock.modal.confirm).toHaveBeenCalledOnce();
    const onOk = mock.modal.confirm.mock.calls[0][0].onOk;
    if (!onOk) throw new Error('Confirmation callback missing');
    await act(async () => {await onOk();});
}
beforeEach(() => {
    vi.stubGlobal('ResizeObserver', class {observe() {} disconnect() {}});
    mock.api.mowglinext.mapDockingCreate.mockReset().mockResolvedValue({data: {}});
    mock.api.request.mockReset().mockResolvedValue(stored());
    mock.api.settings.yamlList.mockReset().mockResolvedValue({data: {...values}});
    mock.api.settings.yamlCreate.mockReset().mockResolvedValue({});
    Object.values(mock.notification).forEach(fn => fn.mockClear());
    mock.modal.confirm.mockClear(); mock.charging = true; mock.x = 10; mock.y = 20; mock.yaw = 0;
});
afterEach(() => {cleanup(); vi.unstubAllGlobals(); vi.restoreAllMocks();});

describe('dock GPS capture confirmation', () => {
    it('writes once after confirmation and adopts exact canonical GPS position and independent yaw', async () => {
        const change = vi.fn(); render(<RobotComponentEditor values={values} onChange={change}/>);
        expect(mock.api.mowglinext.mapDockingCreate).not.toHaveBeenCalled();
        await confirmCapture();
        expect(mock.api.mowglinext.mapDockingCreate).toHaveBeenCalledExactlyOnceWith({docking_pose: {position: {x: 10, y: 20, z: 0}, orientation: {x: 0, y: 0, z: 0, w: 1}}, use_gps_position: true, yaw_source: 0});
        expect(change.mock.calls).toEqual([['dock_pose_x', 1.123456], ['dock_pose_y', 2], ['dock_pose_yaw', .1234567]]);
        expect(mock.api.request).toHaveBeenCalledExactlyOnceWith({path: '/calibration/status', method: 'GET', format: 'json'});
        expect(mock.notification.success).toHaveBeenCalledOnce();
        expect(mock.notification.warning).not.toHaveBeenCalled();
    });
    it('accepts zero request coordinates and all-zero persisted values with present=false', async () => {
        mock.x = mock.y = mock.yaw = 0;
        mock.api.request.mockResolvedValue({data: {dock: {present: false, pose_values_valid: true, dock_pose_x: 0, dock_pose_y: 0, dock_pose_yaw_rad: 0}}});
        const change = vi.fn(); render(<RobotComponentEditor values={values} onChange={change}/>);
        await confirmCapture();
        expect(change.mock.calls).toEqual([['dock_pose_x', 0], ['dock_pose_y', 0], ['dock_pose_yaw', 0]]);
    });
    it.each([{data: {}, error: {error: 'ROS rejected'}}, {data: {}, ok: false}, {data: {}, status: 500}, {data: {unexpected: true}}])('rejects failed/unexpected write acknowledgement %j without reading or changing locals', async response => {
        mock.api.mowglinext.mapDockingCreate.mockResolvedValue(response);
        const change = vi.fn(); render(<RobotComponentEditor values={values} onChange={change}/>);
        await confirmCapture();
        expect(change).not.toHaveBeenCalled(); expect(mock.api.request).not.toHaveBeenCalled();
        expect(mock.api.mowglinext.mapDockingCreate).toHaveBeenCalledOnce();
        expect(mock.notification.success).not.toHaveBeenCalled(); expect(mock.notification.warning).toHaveBeenCalledOnce();
    });
    it.each([{data: {}}, {data: {dock: {pose_values_valid: true, dock_pose_x: 0, dock_pose_y: 0}}}, {data: {dock: {pose_values_valid: true, dock_pose_x: 0, dock_pose_y: null, dock_pose_yaw_rad: 0}}}, {data: {dock: {present: true, dock_pose_x: 0, dock_pose_y: 0, dock_pose_yaw_rad: 0}}}, {data: stored().data, error: {error: 'Read failed'}}])('preserves local coordinates without retry after unconfirmed readback %j', async response => {
        mock.api.request.mockResolvedValue(response);
        const change = vi.fn(); render(<RobotComponentEditor values={values} onChange={change}/>);
        await confirmCapture();
        expect(change).not.toHaveBeenCalled(); expect(mock.api.request).toHaveBeenCalledOnce();
        expect(mock.api.mowglinext.mapDockingCreate).toHaveBeenCalledOnce();
        expect(mock.notification.success).not.toHaveBeenCalled();
        expect(mock.notification.warning.mock.calls[0][0].message).toContain('stored coordinates not confirmed');
    });
    it('reports failed readback after a possibly successful service call without fused fallback', async () => {
        mock.api.request.mockRejectedValue(new Error('Connection lost'));
        const change = vi.fn(); render(<RobotComponentEditor values={values} onChange={change}/>);
        await confirmCapture();
        expect(change).not.toHaveBeenCalled(); expect(mock.notification.success).not.toHaveBeenCalled();
        expect(mock.notification.warning.mock.calls[0][0].description).toContain('no automatic retry');
        expect(mock.api.mowglinext.mapDockingCreate).toHaveBeenCalledOnce();
    });
    it('blocks capture without charging or a finite pose and rechecks contact after confirmation', async () => {
        mock.charging = false;
        const change = vi.fn(); const view = render(<RobotComponentEditor values={values} onChange={change}/>);
        expect(screen.getByRole('button', {name: /Set dock position$/})).toBeDisabled();
        mock.charging = true; mock.x = NaN; view.rerender(<RobotComponentEditor values={values} onChange={change}/>);
        expect(screen.getByRole('button', {name: /Set dock position$/})).toBeDisabled();
        mock.x = 10; view.rerender(<RobotComponentEditor values={values} onChange={change}/>);
        fireEvent.click(screen.getByRole('button', {name: /Set dock position$/}));
        mock.charging = false; view.rerender(<RobotComponentEditor values={values} onChange={change}/>);
        await act(async () => {await mock.modal.confirm.mock.calls[0][0].onOk?.();});
        expect(mock.api.mowglinext.mapDockingCreate).not.toHaveBeenCalled(); expect(change).not.toHaveBeenCalled();
    });
    it('prevents duplicate confirmation writes and callbacks after unmount', async () => {
        let resolve: (value: unknown) => void = () => {};
        mock.api.mowglinext.mapDockingCreate.mockReturnValue(new Promise(r => {resolve = r;}));
        const change = vi.fn(); const view = render(<RobotComponentEditor values={values} onChange={change}/>);
        fireEvent.click(screen.getByRole('button', {name: /Set dock position$/}));
        const onOk = mock.modal.confirm.mock.calls[0][0].onOk;
        let pending: unknown;
        act(() => {pending = onOk?.();});
        await act(async () => {await onOk?.();});
        expect(mock.api.mowglinext.mapDockingCreate).toHaveBeenCalledOnce();
        view.unmount(); await act(async () => {resolve({data: {}}); await pending;});
        expect(change).not.toHaveBeenCalled(); expect(mock.api.request).not.toHaveBeenCalled();
        expect(mock.notification.success).not.toHaveBeenCalled();
    });
});

describe('sensor editor coordinates and dragging', () => {
    it('renders zero values and rejects malformed inputs without displaying them as zero', () => {
        const view = render(<RobotComponentEditor values={{...values, imu_x: 'bad'}} onChange={vi.fn()}/>);
        expect(view.container.querySelector('input[data-setting-key="lidar_x"]')).toHaveValue('0.000');
        expect(view.container.querySelector('input[data-setting-key="imu_x"]')).toHaveValue('');
        expect(screen.getByText(/Some sensor values are invalid/)).toBeInTheDocument();
    });
    it('preserves REP-103 axes, zero yaw and drag inverse projection, and removes window listeners', () => {
        const change = vi.fn(); const view = render(<RobotComponentEditor values={values} onChange={change}/>);
        const svg = screen.getByTestId('sensor-top-editor');
        const rectSpy = vi.spyOn(svg, 'getBoundingClientRect').mockReturnValue(new DOMRect(0, 0, 520, 480));
        const control = view.container.querySelector('[data-sensor-control="lidar"]');
        const hit = control?.querySelector('circle');
        if (!hit || !control) throw new Error('LiDAR drag control missing');
        const cx = Number(hit.getAttribute('cx')), cy = Number(hit.getAttribute('cy'));
        const yawLine = control.querySelector('line');
        expect(Number(yawLine?.getAttribute('x2'))).toBe(cx);
        expect(Number(yawLine?.getAttribute('y2'))).toBeLessThan(cy);
        fireEvent.mouseDown(hit); fireEvent.mouseMove(window, {clientX: cx, clientY: cy});
        expect(change.mock.calls.slice(-2)).toEqual([['lidar_x', 0], ['lidar_y', 0]]);
        change.mockClear(); fireEvent.mouseMove(window, {clientX: cx - 10, clientY: cy - 10});
        expect(change.mock.calls[0][0]).toBe('lidar_x'); expect(change.mock.calls[0][1]).toBeGreaterThan(0);
        expect(change.mock.calls[1][0]).toBe('lidar_y'); expect(change.mock.calls[1][1]).toBeGreaterThan(0);
        const scale = (cy - Number(yawLine?.getAttribute('y2'))) / .06;
        expect(change.mock.calls[0][1]).toBeCloseTo(Math.round(10 / scale * 1000) / 1000, 3);
        fireEvent.mouseUp(window);
        const rotate = control.querySelector('circle[style*="crosshair"]');
        if (!rotate) throw new Error('Rotation handle missing');
        change.mockClear(); fireEvent.mouseDown(rotate); fireEvent.mouseMove(window, {clientX: cx - 20, clientY: cy});
        expect(change.mock.calls).toEqual([['lidar_yaw', 1.5708]]);
        change.mockClear(); rectSpy.mockReturnValue(new DOMRect(0, 0, 0, 0));
        fireEvent.mouseMove(window, {clientX: cx, clientY: cy});
        fireEvent.touchMove(window, {touches: []});
        expect(change).not.toHaveBeenCalled();
        fireEvent.mouseUp(window); change.mockClear(); fireEvent.mouseMove(window, {clientX: 0, clientY: 0});
        expect(change).not.toHaveBeenCalled();
    });
});

function SettingsSensorsHarness() {
    const manager = useSettingsManager();
    if (manager.loading) return <span>Loading settings</span>;
    return <>
        <SensorsSection values={manager.values} onChange={manager.handleChange}
            onDockPosePersisted={manager.acceptPersistedDockPose}
            onDockCaptureStart={manager.beginDockCapture} onDockCaptureFinished={manager.finishDockCapture}/>
        <output data-testid="settings-form">{JSON.stringify(manager.values)}</output>
        <output data-testid="settings-dirty">{manager.dirtyCount}</output>
        <button onClick={() => {void manager.save();}}>Save test settings</button>
        <button onClick={() => manager.handleChange('mower_visual_colour', '#123456')}>Edit unrelated</button>
        <button onClick={() => manager.handleChange('dock_pose_yaw', .9)}>Edit dock</button>
    </>;
}
async function renderSettingsSensors() {
    const view = render(<SettingsSensorsHarness/>);
    await screen.findByTestId('settings-form');
    // This endpoint is independently requested by the manager on initial load.
    mock.api.request.mockClear();
    return view;
}
describe('Settings sensor capture integration', () => {
    it('accepts persisted pose B over saved pose A and leaves it clean without another Save', async () => {
        await renderSettingsSensors();
        await confirmCapture();
        expect(screen.getByTestId('settings-form').textContent).toContain('"dock_pose_x":1.123456');
        expect(screen.getByTestId('settings-form').textContent).toContain('"dock_pose_yaw":0.1234567');
        expect(screen.getByTestId('settings-dirty')).toHaveTextContent('0');
        expect(mock.api.settings.yamlCreate).not.toHaveBeenCalled();
        fireEvent.click(screen.getByText('Save test settings'));
        await waitFor(() => expect(mock.notification.info).toHaveBeenCalled());
        expect(mock.api.mowglinext.mapDockingCreate).toHaveBeenCalledOnce();
        expect(mock.api.settings.yamlCreate).not.toHaveBeenCalled();
    });
    it('saves an unrelated edit without resending captured coordinates or overwriting later calibrated yaw', async () => {
        let persisted: Record<string, unknown> = {...values};
        mock.api.settings.yamlCreate.mockImplementation((payload: unknown) => {
            if (!isJsonObject(payload)) throw new Error('Expected YAML object');
            persisted = {...persisted, ...payload};
            return Promise.resolve({});
        });
        await renderSettingsSensors(); await confirmCapture();
        // An independent calibration changes the canonical yaw after capture.
        persisted = {...persisted, dock_pose_x: 1.123456, dock_pose_y: 2, dock_pose_yaw: .75};
        fireEvent.click(screen.getByText('Edit unrelated'));
        fireEvent.click(screen.getByText('Save test settings'));
        await waitFor(() => expect(mock.api.settings.yamlCreate).toHaveBeenCalledOnce());
        expect(mock.api.settings.yamlCreate).toHaveBeenCalledWith({mower_visual_colour: '#123456'});
        expect(mock.api.mowglinext.mapDockingCreate).toHaveBeenCalledOnce();
        expect(persisted.dock_pose_yaw).toBe(.75);
        await waitFor(() => expect(screen.getByTestId('settings-dirty')).toHaveTextContent('0'));
    });
    it('keeps an explicit manual dock edit after confirmed capture available to the regular Save flow', async () => {
        await renderSettingsSensors(); await confirmCapture();
        fireEvent.click(screen.getByText('Edit dock'));
        fireEvent.click(screen.getByText('Save test settings'));
        await waitFor(() => expect(mock.api.mowglinext.mapDockingCreate).toHaveBeenCalledTimes(2));
        expect(mock.api.settings.yamlCreate).toHaveBeenCalledWith({dock_pose_yaw: .9});
        expect(mock.api.mowglinext.mapDockingCreate.mock.calls[1][0]).toMatchObject({
            use_gps_position: false, yaw_source: 1, docking_pose: {position: {x: 1.123456, y: 2, z: 0}},
        });
    });
    it.each(['malformed', 'failed_read', 'failed_ack'])('keeps dirty dock edits visible but blocks stale YAML/manual writes after %s', async failure => {
        await renderSettingsSensors(); fireEvent.click(screen.getByText('Edit dock'));
        const before = screen.getByTestId('settings-form').textContent;
        if (failure === 'malformed') mock.api.request.mockResolvedValue({data: {dock: {pose_values_valid: true}}});
        else if (failure === 'failed_read') mock.api.request.mockRejectedValue(new Error('Connection lost'));
        else mock.api.mowglinext.mapDockingCreate.mockRejectedValue(new Error('Write acknowledgement lost'));
        await confirmCapture();
        expect(screen.getByTestId('settings-form').textContent).toBe(before);
        expect(screen.getByTestId('settings-dirty')).toHaveTextContent('1');
        fireEvent.click(screen.getByText('Save test settings'));
        await waitFor(() => expect(mock.notification.warning.mock.calls.some(([config]) => typeof config.message === 'string' && config.message.includes('Reload settings'))).toBe(true));
        expect(mock.api.settings.yamlCreate).not.toHaveBeenCalled();
        expect(mock.api.mowglinext.mapDockingCreate).toHaveBeenCalledOnce();
        // A later explicitly requested capture can establish a valid baseline.
        mock.modal.confirm.mockClear(); mock.api.mowglinext.mapDockingCreate.mockResolvedValue({data: {}});
        mock.api.request.mockResolvedValue(stored()); await confirmCapture();
        expect(screen.getByTestId('settings-dirty')).toHaveTextContent('0');
    });
    it('blocks Save during pending readback so a late settings response cannot restore the old baseline', async () => {
        await renderSettingsSensors();
        let resolve: (value: unknown) => void = () => {};
        mock.api.request.mockReturnValue(new Promise(r => {resolve = r;}));
        fireEvent.click(screen.getByText('Edit unrelated'));
        fireEvent.click(screen.getByRole('button', {name: /Set dock position$/}));
        let pending: unknown;
        await act(async () => {pending = mock.modal.confirm.mock.calls[0][0].onOk?.(); await Promise.resolve();});
        fireEvent.click(screen.getByText('Save test settings'));
        await waitFor(() => expect(mock.notification.warning.mock.calls.some(([config]) => typeof config.message === 'string' && config.message.includes('capture is in progress'))).toBe(true));
        expect(mock.api.settings.yamlCreate).not.toHaveBeenCalled();
        await act(async () => {resolve(stored()); await pending;});
        expect(screen.getByTestId('settings-dirty')).toHaveTextContent('1');
        fireEvent.click(screen.getByText('Save test settings'));
        await waitFor(() => expect(mock.api.settings.yamlCreate).toHaveBeenCalledWith({mower_visual_colour: '#123456'}));
        expect(mock.api.mowglinext.mapDockingCreate).toHaveBeenCalledOnce();
    });
    it.each(['lidar_x', 'imu_yaw', 'gps_z'])('clears %s by deleting its YAML key, not by writing zero or a manual dock pose', async key => {
        mock.api.settings.yamlList.mockResolvedValue({data: {...values, [key]: .1}});
        const view = await renderSettingsSensors();
        const input = view.container.querySelector(`input[data-setting-key="${key}"]`);
        if (!input) throw new Error('Sensor input missing');
        fireEvent.change(input, {target: {value: ''}}); fireEvent.blur(input);
        expect(screen.getByTestId('settings-form').textContent).toContain(`"${key}":null`);
        fireEvent.click(screen.getByText('Save test settings'));
        await waitFor(() => expect(mock.api.settings.yamlCreate).toHaveBeenCalledWith({[key]: null}));
        await waitFor(() => expect(screen.getByTestId('settings-form').textContent).not.toContain(`"${key}"`));
        expect(screen.getByTestId('settings-dirty')).toHaveTextContent('0');
        expect(mock.api.mowglinext.mapDockingCreate).not.toHaveBeenCalled();
    });
});
