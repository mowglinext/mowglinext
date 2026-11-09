import {act, cleanup, fireEvent, render, screen, waitFor, within} from '@testing-library/react';
import {afterEach, beforeEach, describe, expect, it, vi, type MockInstance} from 'vitest';
import type {ModalFuncProps} from 'antd';
import {RobotComponentEditor} from './RobotComponentEditor';
import {getColors} from '../theme/colors';

const mock = vi.hoisted(() => ({
    api: {settings: {yamlList: vi.fn(), yamlCreate: vi.fn()}, mowglinext: {mapDockingCreate: vi.fn()}, request: vi.fn()},
    notification: {success: vi.fn<(config: {message?: unknown; description?: unknown}) => void>(), warning: vi.fn<(config: {message?: unknown; description?: unknown}) => void>(), error: vi.fn(), info: vi.fn()},
    modal: {confirm: vi.fn<(props: ModalFuncProps) => void>()},
    charging: true, x: 10, y: 20, yaw: 0,
}));
vi.mock('antd', async importOriginal => ({...await importOriginal<typeof import('antd')>(), App: {useApp: () => ({notification: mock.notification, modal: mock.modal})}}));
vi.mock('../hooks/useApi.ts', () => ({useApi: () => mock.api}));
vi.mock('../hooks/useStatus.ts', () => ({useStatus: () => ({is_charging: mock.charging})}));
vi.mock('../hooks/usePose.ts', () => ({usePose: () => ({pose: {pose: {position: {x: mock.x, y: mock.y}}}, motion_heading: mock.yaw})}));
vi.mock('../hooks/useCalibrationStatus.ts', () => ({useCalibrationStatus: () => ({status: null})}));
vi.mock('../hooks/useRobotDescription.ts', () => ({useRobotDescription: () => ({baseLength: .54, baseWidth: .4, baseHeight: .19, chassisCenterX: .18,
    wheelRadius: .04475, wheelWidth: .04, wheelTrack: .325, wheelXOffset: 0, casterRadius: .03, casterXOffset: .4, casterTrack: .36, bladeRadius: .09, fromUrdf: true, sensors: ['lidar', 'imu', 'gps'].map(id => ({id, x: 0, y: 0, z: .1, roll: 0, pitch: 0, yaw: 0, length: .04, width: .04, height: .04}))})}));
vi.mock('../theme/ThemeContext.tsx', () => ({useThemeMode: () => ({colors: getColors('light'), mode: 'light'})}));
vi.mock('./robot/LayeredMower', () => ({LayeredMower: () => null, SensorGraphic: () => null}));
vi.mock('./robot/MowerPreview', () => ({MowerView: () => null, MowerVisualControls: () => null}));
vi.mock('./robot/DockGraphic', () => ({DockGraphic: () => null}));

const result = {success: true, message: 'Complete', imu_yaw_rad: .123456, imu_yaw_deg: 7.07, samples_used: 250, std_dev_deg: .2};
const fetchMock = vi.fn<typeof fetch>();
let warnings: MockInstance<typeof console.warn>;
let errors: MockInstance<typeof console.warn>;
beforeEach(() => {
    vi.stubGlobal('ResizeObserver', class {observe() {} disconnect() {}});
    vi.stubGlobal('fetch', fetchMock);
    fetchMock.mockReset().mockResolvedValue(new Response(JSON.stringify(result)));
    Object.values(mock.notification).forEach(fn => fn.mockClear());
    warnings = vi.spyOn(console, 'warn');
    errors = vi.spyOn(console, 'error');
});
afterEach(() => {
    cleanup();
    // Spies retain the original console implementation; warnings remain visible.
    for (const spy of [warnings, errors]) {
        expect(spy.mock.calls.flat().join(' ')).not.toMatch(/\[antd: Modal\].*destroyOnClose/);
    }
    vi.unstubAllGlobals(); vi.restoreAllMocks();
});
async function openCalibration() {
    fireEvent.click(screen.getByRole('button', {name: /Auto-calibrate IMU mounting yaw/}));
    return within(await screen.findByRole('dialog'));
}
async function expectClosed() {
    const dialog = screen.getByRole('dialog');
    await waitFor(() => expect(dialog).toHaveClass('ant-zoom-leave-active'));
    fireEvent.animationEnd(dialog);
    fireEvent.transitionEnd(dialog);
    await waitFor(() => expect(screen.queryByRole('dialog')).not.toBeInTheDocument());
    expect(document.querySelector('.ant-modal-body')).toBeNull();
}

describe('IMU modal lifecycle with the real calibration hook', () => {
    it('rejects non-finite derived roll on real Apply without applying yaw or pitch', async () => {
        fetchMock.mockResolvedValue(new Response(JSON.stringify({...result,
            imu_pitch_rad: .012345, imu_pitch_deg: .7073,
            imu_roll_rad: Number.MAX_VALUE, imu_roll_deg: 1,
            stationary_samples_used: 150,
        })));
        const change = vi.fn(); render(<RobotComponentEditor values={{imu_yaw: 0}} onChange={change}/>);
        const dialog = await openCalibration();
        fireEvent.click(dialog.getByRole('button', {name: /Start$/}));
        const apply = await dialog.findByRole('button', {name: 'Apply'});
        expect(change).not.toHaveBeenCalled();
        fireEvent.click(apply);
        expect(change).not.toHaveBeenCalled();
        expect(mock.notification.success).not.toHaveBeenCalled();
        expect(mock.notification.error).toHaveBeenCalledExactlyOnceWith({
            message: 'IMU yaw calibration failed', description: 'An error occured',
        });
        expect(screen.getByRole('dialog')).toBeInTheDocument();
        expect(fetchMock).toHaveBeenCalledExactlyOnceWith('/api/calibration/imu-yaw',
            expect.objectContaining({method: 'POST', body: '{"duration_sec":30}'}));
        fireEvent.click(dialog.getByRole('button', {name: 'Discard'}));
        await expectClosed();
        const reopened = await openCalibration();
        expect(reopened.getByRole('button', {name: /Start$/})).toBeInTheDocument();
        expect(fetchMock).toHaveBeenCalledOnce();
        expect(change).not.toHaveBeenCalled();
    });
    it('cancels and reopens without a request or applying settings', async () => {
        const change = vi.fn(); render(<RobotComponentEditor values={{imu_yaw: 0}} onChange={change}/>);
        let dialog = await openCalibration();
        fireEvent.click(dialog.getByRole('button', {name: 'Cancel'}));
        await expectClosed();
        dialog = await openCalibration();
        expect(dialog.getByRole('button', {name: /Start$/})).toBeInTheDocument();
        expect(fetchMock).not.toHaveBeenCalled(); expect(change).not.toHaveBeenCalled();
    });
    it('blocks Escape, mask and close controls while running, and discards without applying', async () => {
        let resolve: (response: Response) => void = () => {};
        fetchMock.mockReturnValue(new Promise<Response>(r => {resolve = r;}));
        const change = vi.fn(); render(<RobotComponentEditor values={{imu_yaw: 0}} onChange={change}/>);
        const dialog = await openCalibration();
        fireEvent.click(dialog.getByRole('button', {name: /Start$/}));
        expect(fetchMock).toHaveBeenCalledExactlyOnceWith('/api/calibration/imu-yaw', expect.objectContaining({method: 'POST', body: '{"duration_sec":30}'}));
        expect(dialog.queryByRole('button', {name: /Start$/})).not.toBeInTheDocument();
        expect(dialog.queryByRole('button', {name: 'Close'})).not.toBeInTheDocument();
        const wrap = document.querySelector('.ant-modal-wrap');
        if (!wrap) throw new Error('Modal wrapper missing');
        fireEvent.keyDown(wrap, {key: 'Escape', keyCode: 27});
        fireEvent.click(wrap);
        expect(screen.getByRole('dialog')).toBeInTheDocument();
        expect(fetchMock).toHaveBeenCalledOnce(); expect(change).not.toHaveBeenCalled();
        await act(async () => {resolve(new Response(JSON.stringify(result))); await Promise.resolve();});
        expect(dialog.getByText('imu_yaw = 7.07° (0.1235 rad)')).toBeInTheDocument();
        expect(dialog.getByRole('button', {name: 'Apply'})).toBeInTheDocument();
        expect(change).not.toHaveBeenCalled();
        fireEvent.click(dialog.getByRole('button', {name: 'Discard'}));
        await expectClosed();
        const reopened = await openCalibration();
        expect(reopened.queryByRole('button', {name: 'Apply'})).not.toBeInTheDocument();
        expect(reopened.getByRole('button', {name: /Start$/})).toBeInTheDocument();
        expect(fetchMock).toHaveBeenCalledOnce(); expect(change).not.toHaveBeenCalled();
    });
    it('retains a failure until Retry, makes no implicit request, and applies only on explicit Apply', async () => {
        fetchMock.mockResolvedValueOnce(new Response(JSON.stringify({...result, success: false, message: 'Not enough samples'})));
        const change = vi.fn(); render(<RobotComponentEditor values={{imu_yaw: 0}} onChange={change}/>);
        const dialog = await openCalibration();
        fireEvent.click(dialog.getByRole('button', {name: /Start$/}));
        await waitFor(() => expect(dialog.getByText('Not enough samples')).toBeInTheDocument());
        expect(dialog.queryByRole('button', {name: 'Apply'})).not.toBeInTheDocument();
        fireEvent.click(dialog.getByRole('button', {name: 'Retry'}));
        expect(dialog.queryByText('Not enough samples')).not.toBeInTheDocument();
        expect(fetchMock).toHaveBeenCalledOnce(); expect(change).not.toHaveBeenCalled();
        fireEvent.click(dialog.getByRole('button', {name: /Start$/}));
        await waitFor(() => expect(dialog.getByRole('button', {name: 'Apply'})).toBeInTheDocument());
        expect(change).not.toHaveBeenCalled();
        fireEvent.click(dialog.getByRole('button', {name: 'Apply'}));
        await expectClosed();
        expect(change).toHaveBeenCalledExactlyOnceWith('imu_yaw', .1235);
        expect(fetchMock).toHaveBeenCalledTimes(2);
        const reopened = await openCalibration();
        expect(reopened.getByRole('button', {name: /Start$/})).toBeInTheDocument();
        expect(fetchMock).toHaveBeenCalledTimes(2);
    });
    it('reports HTTP errors and permits closing/reopening without another request', async () => {
        fetchMock.mockResolvedValue(new Response('Rejected', {status: 500}));
        const change = vi.fn(); render(<RobotComponentEditor values={{imu_yaw: 0}} onChange={change}/>);
        const dialog = await openCalibration();
        fireEvent.click(dialog.getByRole('button', {name: /Start$/}));
        await waitFor(() => expect(mock.notification.error).toHaveBeenCalledOnce());
        expect(mock.notification.error).toHaveBeenCalledWith(expect.objectContaining({description: 'HTTP 500: Rejected'}));
        fireEvent.click(dialog.getByRole('button', {name: 'Cancel'}));
        await expectClosed(); await openCalibration();
        expect(fetchMock).toHaveBeenCalledOnce(); expect(change).not.toHaveBeenCalled();
    });
});
