import yamlSettingsProducer from '../../../pkg/api/settings.go?raw';
import {act, renderHook, waitFor} from "@testing-library/react";
import {beforeEach, expect, it, vi} from "vitest";

const mocks = vi.hoisted(() => ({
    hardware: {authority: "confirmed", runtimeRouting: "available", robotName: "", backend: "mavros", loading: false, defaultOverrides: {}, parameterRoutes: {} as Record<string, {parameter: string; runtime: string}>},
    api: {settings: {yamlList: vi.fn(), yamlCreate: vi.fn()}, request: vi.fn<(args: {path: string; body?: unknown}) => Promise<{data: Record<string, unknown>}>>()},
    notification: {success: vi.fn(), info: vi.fn(), warning: vi.fn(), error: vi.fn()},
}));
vi.mock("./useApi.ts", () => ({useApi: () => mocks.api}));
vi.mock("./useHardwareBackend.ts", () => ({useHardwareBackend: () => mocks.hardware}));
vi.mock("./useContainerRestart.ts", () => ({useContainerRestart: () => ({pending: false, run: vi.fn()})}));
vi.mock("react-i18next", () => ({useTranslation: () => ({t: (key: string) => key})}));
vi.mock("antd", () => ({App: {useApp: () => ({notification: mocks.notification})}}));
import {isJsonObject} from '../utils/json';
import {useSettingsManager} from "./useSettingsManager";

beforeEach(() => {
    vi.clearAllMocks();
    sessionStorage.clear();
    mocks.hardware.authority = "confirmed";
    mocks.hardware.runtimeRouting = "available";
    mocks.hardware.backend = "mavros";
    mocks.hardware.parameterRoutes = {};
    mocks.api.settings.yamlList.mockResolvedValue({data: {ticks_per_meter: 48, chassis_width: .45, caster_x_offset: .3}});
    mocks.api.settings.yamlCreate.mockResolvedValue({});
    mocks.api.request.mockImplementation(args => Promise.resolve({data: args.path === '/params' && isJsonObject(args.body)
        ? {parameters: args.body.parameters} : {}}));
});

it('keeps a newer draft when the retired root StrictMode initial load resolves late', async () => {
    const finish: Array<(value: unknown) => void> = [];
    mocks.api.settings.yamlList.mockImplementation(() => new Promise(resolve => finish.push(resolve)));
    const view = renderHook(useSettingsManager, {reactStrictMode: true});
    expect(finish).toHaveLength(2);
    await act(() => Promise.resolve( finish[1]({data: {ticks_per_meter: 48}})));
    expect(view.result.current.loading).toBe(false);
    act(() => view.result.current.handleChange('ticks_per_meter', 75));
    expect(view.result.current.dirtyCount).toBe(1);
    await act(() => Promise.resolve( finish[0]({data: {ticks_per_meter: 48}})));
    expect(view.result.current.values.ticks_per_meter).toBe(75);
    expect(view.result.current.dirtyCount).toBe(1);
    expect(mocks.api.settings.yamlCreate).not.toHaveBeenCalled();
    view.unmount();
});

it('keeps edits made while the active initial load is pending', async () => {
    let finish!: (value: unknown) => void;
    mocks.api.settings.yamlList.mockImplementation(() => new Promise(resolve => {finish = resolve;}));
    const {result, rerender} = renderHook(useSettingsManager);
    act(() => result.current.handleChange('ticks_per_meter', 75));
    act(() => result.current.handleBulkChange({chassis_width: .6, removed_parameter: null}));
    await act(() => Promise.resolve( finish({data: {ticks_per_meter: 48, chassis_width: .45, untouched: 'server', removed_parameter: 1}})));
    expect(result.current.values).toMatchObject({ticks_per_meter: 75, chassis_width: .6, untouched: 'server', removed_parameter: null});
    expect(result.current.dirtyCount).toBe(3);
    rerender();
    expect(mocks.api.settings.yamlList).toHaveBeenCalledTimes(1);
});

it('waits for the active root StrictMode load and ignores a retired failure', async () => {
    const requests: Array<{resolve: (value: unknown) => void; reject: (error: Error) => void}> = [];
    mocks.api.settings.yamlList.mockImplementation(() => new Promise((resolve, reject) => requests.push({resolve, reject})));
    const {result} = renderHook(useSettingsManager, {reactStrictMode: true});
    expect(requests).toHaveLength(2);
    await act(() => Promise.resolve( requests[0].reject(new Error('retired read'))));
    expect(result.current.loading).toBe(true);
    expect(mocks.notification.error).not.toHaveBeenCalled();
    await act(() => Promise.resolve( requests[1].resolve({data: {ticks_per_meter: 48}})));
    expect(result.current.loading).toBe(false);
    expect(result.current.values.ticks_per_meter).toBe(48);
    expect(result.current.dirtyCount).toBe(0);
});

it('ignores an initial-load rejection after unmount', async () => {
    let reject!: (error: Error) => void;
    mocks.api.settings.yamlList.mockImplementation(() => new Promise((_resolve, fail) => {reject = fail;}));
    const view = renderHook(useSettingsManager);
    view.unmount();
    await act(() => Promise.resolve( reject(new Error('unmounted read'))));
    expect(mocks.notification.error).not.toHaveBeenCalled();
});

it.each(["mavros", "mowgli", "openmower"])("saves display-only edits without live push or ROS restart on %s", async backend => {
    mocks.hardware.backend = backend;
    const {result} = renderHook(() => useSettingsManager());
    await waitFor(() => expect(result.current.loading).toBe(false));
    act(() => result.current.handleBulkChange({mower_visual_colour: "#112233", dock_visual_length: .8}));
    await act(() => result.current.save());
    expect(mocks.api.settings.yamlCreate).toHaveBeenCalledWith({mower_visual_colour: "#112233", dock_visual_length: .8});
    expect(result.current.restartRequired).toBe(false);
    expect(mocks.api.request.mock.calls.some(([args]) => args.path === "/params")).toBe(false);
    expect(mocks.notification.success).toHaveBeenCalledWith(expect.objectContaining({description: "settingsSections.toasts.savedDisplaySettings"}));
});

it.each([
    ["mavros", "available", "mavros/esc_wheel_odometry.ticks_per_meter"],
    ["mowgli", "available", "hardware_bridge.ticks_per_meter"],
    ["mavros", "pending", "mavros/esc_wheel_odometry.ticks_per_meter"],
    ["openmower", "unavailable", "hardware_bridge.ticks_per_meter"],
])("uses confirmed runtime routes and restart fallback: %s/%s", async (backend, runtime, parameter) => {
    mocks.hardware.backend = backend;
    mocks.hardware.parameterRoutes = {ticks_per_meter: {parameter, runtime}};
    const {result} = renderHook(() => useSettingsManager());
    await waitFor(() => expect(result.current.loading).toBe(false));
    act(() => result.current.handleBulkChange({ticks_per_meter: 49, mower_visual_style: "worx"}));
    await act(() => result.current.save());
    const live = mocks.api.request.mock.calls.filter(([args]) => args.path === "/params");
    expect(result.current.restartRequired).toBe(runtime !== "available");
    if (runtime === "available") {
        expect(live).toHaveLength(1);
        expect(live[0][0].body).toEqual({parameters: [{name: parameter, value: 49, type: "float64"}]});
        expect(mocks.notification.success).toHaveBeenCalledWith(expect.objectContaining({description: "settingsSections.toasts.savedApplyingLive"}));
    } else expect(live).toHaveLength(0);
    expect(mocks.api.settings.yamlCreate).toHaveBeenCalledWith({ticks_per_meter: 49, mower_visual_style: "worx"});
});

it("geometry edits still require restart alongside confirmed live updates", async () => {
    mocks.hardware.parameterRoutes = {ticks_per_meter: {parameter: "mavros/esc_wheel_odometry.ticks_per_meter", runtime: "available"}};
    const {result} = renderHook(() => useSettingsManager());
    await waitFor(() => expect(result.current.loading).toBe(false));
    act(() => result.current.handleBulkChange({ticks_per_meter: 49, chassis_width: .5}));
    await act(() => result.current.save());
    expect(result.current.restartRequired).toBe(true);
    expect(mocks.api.settings.yamlCreate).toHaveBeenCalledWith({ticks_per_meter: 49, chassis_width: .5});
});

it.each(['loading', 'unavailable', 'legacy_absent', 'unsupported', 'malformed'])('loaded settings do not gain live authority while metadata is %s', async authority => {
    mocks.hardware.authority = authority;
    mocks.hardware.parameterRoutes = {ticks_per_meter: {parameter: 'mavros/esc_wheel_odometry.ticks_per_meter', runtime: 'available'}};
    const {result} = renderHook(useSettingsManager);
    await waitFor(() => expect(result.current.loading).toBe(false));
    act(() => result.current.handleChange('ticks_per_meter', 49));
    await act(() => result.current.save());
    expect(mocks.api.request.mock.calls.some(([args]) => args.path === '/params')).toBe(false);
    expect(result.current.restartRequired).toBe(true); expect(result.current.dirtyCount).toBe(0);
});

it.each(['rejected', 'missing_echo', 'wrong_echo'])('YAML remains saved when runtime application is %s', async failure => {
    mocks.hardware.parameterRoutes = {ticks_per_meter: {parameter: 'mavros/esc_wheel_odometry.ticks_per_meter', runtime: 'available'}};
    mocks.api.request.mockImplementation(args => {
        if (args.path !== '/params') return Promise.resolve({data: {}});
        if (failure === 'rejected') return Promise.reject(new Error('ROS rejected'));
        return Promise.resolve({data: {parameters: failure === 'wrong_echo' ? [{name: 'mavros/esc_wheel_odometry.ticks_per_meter', value: 50}] : []}});
    });
    const {result} = renderHook(useSettingsManager); await waitFor(() => expect(result.current.loading).toBe(false));
    act(() => result.current.handleChange('ticks_per_meter', 49)); await act(() => result.current.save());
    expect(mocks.api.settings.yamlCreate).toHaveBeenCalledWith({ticks_per_meter: 49});
    expect(result.current.dirtyCount).toBe(0); expect(result.current.restartRequired).toBe(true);
    expect(mocks.notification.warning).toHaveBeenCalled(); expect(mocks.notification.error).not.toHaveBeenCalled();
    expect(mocks.notification.success).not.toHaveBeenCalledWith(expect.objectContaining({message: 'settingsSections.toasts.liveApplied'}));
});

it('reports a runtime value the bridge adjusted instead of calling it unconfirmed', async () => {
    const parameter = 'hardware_bridge.ticks_per_meter';
    mocks.hardware.backend = 'mowgli';
    mocks.hardware.parameterRoutes = {ticks_per_meter: {parameter, runtime: 'available'}};
    mocks.api.request.mockImplementation(args => Promise.resolve({data: args.path !== '/params' ? {} : {
        parameters: [{name: parameter, value: 45}], adjusted: true,
        adjustments: [{name: parameter, requested_value: 49, applied_value: 45}]}}));
    const {result} = renderHook(useSettingsManager); await waitFor(() => expect(result.current.loading).toBe(false));
    act(() => result.current.handleChange('ticks_per_meter', 49)); await act(() => result.current.save());
    expect(result.current.restartRequired).toBe(false);
    expect(mocks.notification.warning).toHaveBeenCalledWith(expect.objectContaining({message: 'settingsSections.toasts.liveAdjusted'}));
    expect(mocks.notification.warning).not.toHaveBeenCalledWith(expect.objectContaining({message: 'settingsSections.toasts.drivePidUpdateFailed'}));
    expect(mocks.notification.success).not.toHaveBeenCalledWith(expect.objectContaining({message: 'settingsSections.toasts.liveApplied'}));
});

it.each([NaN, Infinity, -Infinity])('does not silently persist nonfinite %s as a JSON null', async value => {
    const {result} = renderHook(useSettingsManager); await waitFor(() => expect(result.current.loading).toBe(false));
    act(() => result.current.handleChange('ticks_per_meter', value)); await act(() => result.current.save());
    expect(mocks.api.settings.yamlCreate).not.toHaveBeenCalled(); expect(mocks.notification.error).toHaveBeenCalled();
});

it('an external saver still works without hardware routing', async () => {
    mocks.hardware.authority = 'unavailable'; const save = vi.fn().mockResolvedValue(true);
    const {result} = renderHook(useSettingsManager); await waitFor(() => expect(result.current.loading).toBe(false));
    act(() => result.current.registerExternalSaver('remote_access', {dirtyCount: 1, save}));
    await act(() => result.current.save()); expect(save).toHaveBeenCalledTimes(1);
    expect(mocks.api.settings.yamlCreate).not.toHaveBeenCalled();
});

it('keeps edits made while YAML persistence is in progress dirty', async () => {
    let finish: (result: unknown) => void = () => {};
    mocks.api.settings.yamlCreate.mockImplementation(() => new Promise(resolve => {finish = resolve;}));
    const {result} = renderHook(useSettingsManager); await waitFor(() => expect(result.current.loading).toBe(false));
    act(() => result.current.handleChange('ticks_per_meter', 49));
    let saving: Promise<void> = Promise.resolve();
    act(() => {saving = result.current.save();});
    act(() => result.current.handleChange('ticks_per_meter', 50));
    await act(async () => {finish({}); await saving;});
    expect(result.current.values.ticks_per_meter).toBe(50); expect(result.current.dirtyCount).toBe(1);
});

it.each(['resolved', 'rejected'])('retains canonical persistence after a %s sidecar failure response', async mode => {
    const response = {error: {persisted: true, error: 'sidecar write failed'}};
    if (mode === 'resolved') mocks.api.settings.yamlCreate.mockResolvedValue(response);
    else mocks.api.settings.yamlCreate.mockRejectedValue(response);
    mocks.hardware.parameterRoutes = {ticks_per_meter: {parameter: 'mavros/esc_wheel_odometry.ticks_per_meter', runtime: 'available'}};
    const {result} = renderHook(useSettingsManager); await waitFor(() => expect(result.current.loading).toBe(false));
    act(() => result.current.handleChange('ticks_per_meter', 49)); await act(() => result.current.save());
    expect(result.current.dirtyCount).toBe(0); expect(result.current.restartRequired).toBe(true);
    expect(mocks.notification.warning).toHaveBeenCalledWith(expect.objectContaining({description: 'sidecar write failed'}));
    expect(mocks.notification.error).not.toHaveBeenCalled();
    expect(mocks.notification.success).not.toHaveBeenCalled();
    expect(mocks.notification.info).not.toHaveBeenCalled();
    expect(mocks.notification.warning).toHaveBeenCalledTimes(1);
    expect(mocks.notification.warning.mock.invocationCallOrder[0]).toBeGreaterThan(mocks.api.settings.yamlCreate.mock.invocationCallOrder[0]);
    expect(mocks.api.request.mock.calls.some(([args]) => args.path === '/params')).toBe(false);
});

it('a YAML rejection keeps edits dirty and never dispatches runtime writes', async () => {
    mocks.api.settings.yamlCreate.mockRejectedValue(new Error('write failed'));
    mocks.hardware.parameterRoutes = {ticks_per_meter: {parameter: 'mavros/esc_wheel_odometry.ticks_per_meter', runtime: 'available'}};
    const {result} = renderHook(useSettingsManager); await waitFor(() => expect(result.current.loading).toBe(false));
    act(() => result.current.handleChange('ticks_per_meter', 49)); await act(() => result.current.save());
    expect(result.current.dirtyCount).toBe(1); expect(mocks.notification.error).toHaveBeenCalled();
    expect(mocks.api.request.mock.calls.some(([args]) => args.path === '/params')).toBe(false);
});

it.each(['pending_image', 'restart_required', 'unavailable'])('reports backend-wide %s independently of a route claiming available', async state => {
    mocks.hardware.runtimeRouting = state;
    mocks.hardware.parameterRoutes = {ticks_per_meter: {parameter: 'mavros/esc_wheel_odometry.ticks_per_meter', runtime: 'available'}};
    const {result} = renderHook(useSettingsManager); await waitFor(() => expect(result.current.loading).toBe(false));
    act(() => result.current.handleChange('ticks_per_meter', 49)); await act(() => result.current.save());
    expect(result.current.dirtyCount).toBe(0); expect(result.current.restartRequired).toBe(true);
    expect(mocks.notification.warning).toHaveBeenCalledWith(expect.objectContaining({description: `settingsSections.toasts.routing.${state}`}));
    expect(mocks.notification.success).not.toHaveBeenCalled();
    expect(mocks.api.request.mock.calls.some(([args]) => args.path === '/params')).toBe(false);
});

it('partial persistence records the saved keys but reports sidecar failure', async () => {
    mocks.api.settings.yamlCreate.mockRejectedValue({error: {persisted: true, error: 'sidecar failed'}});
    const {result} = renderHook(useSettingsManager); await waitFor(() => expect(result.current.loading).toBe(false));
    let ok = true;
    await act(async () => {ok = await result.current.savePartialValues({ticks_per_meter: 49});});
    expect(ok).toBe(false); expect(result.current.values.ticks_per_meter).toBe(49);
    expect(result.current.dirtyCount).toBe(0); expect(result.current.restartRequired).toBe(true);
    expect(mocks.notification.error).not.toHaveBeenCalled();
});

it('emits only the partial-persistence warning after the response and keeps newer edits dirty', async () => {
    let finish: (response: unknown) => void = () => {};
    mocks.api.settings.yamlCreate.mockImplementation(() => new Promise(resolve => {finish = resolve;}));
    mocks.hardware.parameterRoutes = {ticks_per_meter: {parameter: 'mavros/esc_wheel_odometry.ticks_per_meter', runtime: 'available'}};
    const {result} = renderHook(useSettingsManager); await waitFor(() => expect(result.current.loading).toBe(false));
    act(() => result.current.handleChange('ticks_per_meter', 49));
    let saving = Promise.resolve();
    act(() => {saving = result.current.save();});
    expect(mocks.notification.success).not.toHaveBeenCalled();
    expect(mocks.notification.warning).not.toHaveBeenCalled();
    act(() => result.current.handleChange('ticks_per_meter', 50));
    await act(async () => {finish({error: {persisted: true, error: 'env sync failed'}}); await saving;});
    expect(mocks.notification.warning.mock.calls).toEqual([[{message: 'settingsSections.toasts.savedRuntimeFailed', description: 'env sync failed'}]]);
    expect(mocks.notification.success).not.toHaveBeenCalled();
    expect(mocks.notification.info).not.toHaveBeenCalled(); expect(mocks.notification.error).not.toHaveBeenCalled();
    expect(result.current.values.ticks_per_meter).toBe(50);
    expect(result.current.dirtyCount).toBe(1); expect(result.current.restartRequired).toBe(true);
    expect(mocks.api.request.mock.calls.some(([args]) => args.path === '/params')).toBe(false);
});

it('protects partial dock saves after an unconfirmed GPS capture while allowing unrelated partial writes', async () => {
    const {result} = renderHook(useSettingsManager);
    await waitFor(() => expect(result.current.loading).toBe(false));
    expect(result.current.beginDockCapture()).toBe(true);
    result.current.finishDockCapture();
    let saved: boolean | undefined;
    await act(async () => {saved = await result.current.savePartialValues({dock_pose_yaw: .5});});
    expect(saved).toBe(false); expect(mocks.api.settings.yamlCreate).not.toHaveBeenCalled();
    await act(async () => {saved = await result.current.savePartialValues({ntrip_host: 'host'});});
    expect(saved).toBe(true);
    expect(mocks.api.settings.yamlCreate).toHaveBeenCalledExactlyOnceWith({ntrip_host: 'host'});
});

it.each(['regular', 'partial'])('refuses a capture during a %s YAML write, then releases the guard', async kind => {
    let finish: (value: unknown) => void = () => {};
    mocks.api.settings.yamlCreate.mockReturnValue(new Promise(resolve => {finish = resolve;}));
    const {result} = renderHook(useSettingsManager);
    await waitFor(() => expect(result.current.loading).toBe(false));
    act(() => result.current.handleChange('mower_visual_colour', '#123456'));
    let pending: Promise<unknown>;
    act(() => {pending = kind === 'regular' ? result.current.save() : result.current.savePartialValues({ntrip_host: 'host'});});
    expect(result.current.beginDockCapture()).toBe(false);
    await act(async () => {finish({}); await pending;});
    expect(result.current.beginDockCapture()).toBe(true);
    result.current.finishDockCapture();
});

it('owns advanced key authorization across all sections and backend-retired keys', async () => {
    const producer = yamlSettingsProducer;
    const retiredBlock = /var retiredParamKeys = map\[string\]bool\{([\s\S]*?)\n\}/.exec(producer)?.[1];
    expect(retiredBlock).toBeDefined();
    const retired = [...(retiredBlock ?? '').matchAll(/"([^"]+)":\s*true/g)].map(match => match[1]);
    expect(retired.length).toBeGreaterThan(20);
    mocks.api.settings.yamlList.mockResolvedValue({data: {custom: 0, deleted: null, use_scan_matching: true}});
    const {result} = renderHook(useSettingsManager);
    await waitFor(() => expect(result.current.loading).toBe(false));
    for (const key of retired) {
        expect(result.current.validateAdvancedKey(key), key).toBe('protectedKey');
        expect(result.current.advancedKeys).not.toContain(key);
    }
    for (const key of ['dock_pose_x', 'dock_pose_y', 'dock_pose_yaw', 'gps_x', 'gnss_receiver_family', 'openmower_wheel_kp']) {
        expect(result.current.validateAdvancedKey(key), key).toBe('protectedKey');
    }
    expect(result.current.validateAdvancedKey('custom')).toBe('duplicateKey');
    expect(result.current.validateAdvancedKey('deleted')).toBe('duplicateKey');
    expect(result.current.validateAdvancedKey('custom_new')).toBeUndefined();
    for (const name of ['', ' x ', 'a.b', '__proto__']) expect(result.current.validateAdvancedKey(name)).toBe('invalidName');
    expect(mocks.api.settings.yamlCreate).not.toHaveBeenCalled();
});

it('counts the IrriSense saver in its containing Weather section until explicit revert', async () => {
    const {result} = renderHook(useSettingsManager);
    await waitFor(() => expect(result.current.loading).toBe(false));
    const revert = vi.fn();
    act(() => result.current.registerExternalSaver('irrisense', {dirtyCount: 2, save: vi.fn(), revert}));
    act(() => result.current.handleChange('robot_name', 'Draft'));
    expect(result.current.dirtyCount).toBe(3);
    expect(result.current.isSectionDirty('weather')).toBe(true);
    act(() => result.current.revert());
    expect(revert).toHaveBeenCalledOnce();
    act(() => result.current.registerExternalSaver('irrisense', {dirtyCount: 0, save: vi.fn(), revert}));
    expect(result.current.dirtyCount).toBe(0);
    expect(result.current.isSectionDirty('weather')).toBe(false);
});
