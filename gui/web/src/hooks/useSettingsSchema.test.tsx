import {act, renderHook, waitFor} from '@testing-library/react';
import {beforeEach, expect, it, vi} from 'vitest';

const mock = vi.hoisted(() => ({
    api: {settings: {schemaList: vi.fn(), yamlList: vi.fn(), yamlCreate: vi.fn()}},
    notification: {success: vi.fn(), error: vi.fn()},
}));
vi.mock('./useApi', () => ({useApi: () => mock.api}));
vi.mock('antd', () => ({App: {useApp: () => ({notification: mock.notification})}}));
import {useSettingsSchema} from './useSettingsSchema';

beforeEach(() => {
    vi.clearAllMocks();
    mock.api.settings.schemaList.mockResolvedValue({data: {properties: {}}});
    mock.api.settings.yamlList.mockResolvedValue({data: {imu_yaw: .1}});
});

it.each(['rejection', 'error response'])('reports %s to the caller and accepts a successful retry', async failure => {
    const {result} = renderHook(useSettingsSchema);
    await waitFor(() => expect(result.current.values.imu_yaw).toBe(.1));
    if (failure === 'rejection') mock.api.settings.yamlCreate.mockRejectedValueOnce(new Error('save failed'));
    else mock.api.settings.yamlCreate.mockResolvedValueOnce({error: {message: 'save failed'}});
    let saved: boolean | undefined;
    await act(async () => {saved = await result.current.saveValues({imu_yaw: .25});});
    expect(saved).toBe(false);
    expect(result.current.values.imu_yaw).toBe(.1);
    expect(result.current.restartRequired).toBe(false);
    expect(result.current.loading).toBe(false);
    expect(mock.notification.error).toHaveBeenCalledOnce();
    expect(mock.notification.success).not.toHaveBeenCalled();
    mock.api.settings.yamlCreate.mockResolvedValueOnce({});
    await act(async () => {saved = await result.current.saveValues({imu_yaw: .25});});
    expect(saved).toBe(true);
    expect(result.current.values.imu_yaw).toBe(.25);
    expect(result.current.restartRequired).toBe(true);
});
