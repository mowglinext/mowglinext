import {act, renderHook, waitFor} from "@testing-library/react";
import {beforeEach, expect, it, vi} from "vitest";

const mocks = vi.hoisted(() => ({
    hardware: {backend: "mavros", loading: false, defaultOverrides: {}, parameterRoutes: {} as Record<string, {parameter: string; runtime: string}>},
    api: {settings: {yamlList: vi.fn(), yamlCreate: vi.fn()}, request: vi.fn<(args: {path: string; body?: unknown}) => Promise<{data: Record<string, unknown>}>>()},
    notification: {success: vi.fn(), info: vi.fn(), warning: vi.fn(), error: vi.fn()},
}));
vi.mock("./useApi.ts", () => ({useApi: () => mocks.api}));
vi.mock("./useHardwareBackend.ts", () => ({useHardwareBackend: () => mocks.hardware}));
vi.mock("./useContainerRestart.ts", () => ({useContainerRestart: () => ({pending: false, run: vi.fn()})}));
vi.mock("react-i18next", () => ({useTranslation: () => ({t: (key: string) => key})}));
vi.mock("antd", () => ({App: {useApp: () => ({notification: mocks.notification})}}));
import {useSettingsManager} from "./useSettingsManager";

beforeEach(() => {
    vi.clearAllMocks();
    mocks.hardware.backend = "mavros";
    mocks.hardware.parameterRoutes = {};
    mocks.api.settings.yamlList.mockResolvedValue({data: {ticks_per_meter: 48, chassis_width: .45, caster_x_offset: .3}});
    mocks.api.settings.yamlCreate.mockResolvedValue({});
    mocks.api.request.mockResolvedValue({data: {}});
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
    ["mavros", "available", "esc_wheel_odometry.ticks_per_meter"],
    ["mowgli", "available", "hardware_bridge.ticks_per_meter"],
    ["mavros", "pending", "esc_wheel_odometry.ticks_per_meter"],
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
        expect(live[0][0].body).toEqual({parameters: [{name: parameter, value: 49}]});
        expect(mocks.notification.success).toHaveBeenCalledWith(expect.objectContaining({description: "settingsSections.toasts.savedApplyingLive"}));
    } else expect(live).toHaveLength(0);
    expect(mocks.api.settings.yamlCreate).toHaveBeenCalledWith({ticks_per_meter: 49, mower_visual_style: "worx"});
});

it("geometry edits still require restart alongside confirmed live updates", async () => {
    mocks.hardware.parameterRoutes = {ticks_per_meter: {parameter: "esc_wheel_odometry.ticks_per_meter", runtime: "available"}};
    const {result} = renderHook(() => useSettingsManager());
    await waitFor(() => expect(result.current.loading).toBe(false));
    act(() => result.current.handleBulkChange({ticks_per_meter: 49, chassis_width: .5}));
    await act(() => result.current.save());
    expect(result.current.restartRequired).toBe(true);
    expect(mocks.api.settings.yamlCreate).toHaveBeenCalledWith({ticks_per_meter: 49, chassis_width: .5});
});
