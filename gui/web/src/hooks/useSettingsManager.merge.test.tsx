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
