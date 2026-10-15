import {act, renderHook, waitFor} from '@testing-library/react';
import {afterEach, beforeEach, describe, expect, it, vi} from 'vitest';
import {useConfig} from './useConfig';
import {useSettings} from './useSettings';

const mock = vi.hoisted(() => ({
    language: 'en',
    config: {keysGetCreate: vi.fn(), keysSetCreate: vi.fn()},
    settings: {settingsList: vi.fn(), yamlList: vi.fn()},
    notification: {error: vi.fn(), success: vi.fn()},
}));
vi.mock('./useApi.ts', () => ({useApi: () => ({config: mock.config, settings: mock.settings})}));
vi.mock('antd', () => ({App: {useApp: () => ({notification: mock.notification})}}));
vi.mock('react-i18next', () => ({useTranslation: () => ({t: (key: string) => `${mock.language}:${key}`})}));

describe('configuration load lifetime', () => {
    beforeEach(() => { mock.language = 'en'; mock.config.keysGetCreate.mockReset(); mock.config.keysSetCreate.mockReset(); mock.notification.error.mockClear(); });

    it('reads its initial key set once without refetching or writing during rerenders', async () => {
        mock.config.keysGetCreate.mockResolvedValue({data: {bearing: '12'}});
        const {result, rerender} = renderHook(() => useConfig(['bearing']));
        await waitFor(() => expect(result.current.config.bearing).toBe('12'));
        mock.language = 'fr';
        rerender();
        expect(mock.config.keysGetCreate).toHaveBeenCalledExactlyOnceWith({bearing: ''});
        expect(mock.config.keysSetCreate).not.toHaveBeenCalled();
        expect(result.current.config.bearing).toBe('12');
    });

    it('reports a pending load failure using the language committed since it started', async () => {
        let reject: (error: Error) => void = () => { throw new Error('Not initialized'); };
        const pending = new Promise<never>((_resolve, rejectRequest) => { reject = rejectRequest; });
        mock.config.keysGetCreate.mockReturnValue(pending);
        const {rerender} = renderHook(() => useConfig(['bearing']));
        mock.language = 'fr';
        rerender();
        await act(async () => { reject(new Error('Read failed')); await Promise.resolve(); });
        expect(mock.notification.error).toHaveBeenCalledExactlyOnceWith({message: 'fr:hookNotifications.failedToLoadConfig', description: 'Read failed'});
        expect(mock.config.keysGetCreate).toHaveBeenCalledOnce();
    });
});


describe('configuration payload validation', () => {
    beforeEach(() => { mock.language = 'en'; mock.config.keysGetCreate.mockReset(); mock.config.keysSetCreate.mockReset(); mock.notification.error.mockClear(); });

    it('rejects malformed read data without storing it or making a write', async () => {
        mock.config.keysGetCreate.mockResolvedValue({data: {bearing: 12}});
        const {result} = renderHook(() => useConfig(['bearing']));
        await waitFor(() => expect(mock.notification.error).toHaveBeenCalledOnce());
        expect(result.current.config).toEqual({});
        expect(mock.config.keysSetCreate).not.toHaveBeenCalled();
    });

    it('retains the previous configuration when a write echo is malformed', async () => {
        mock.config.keysGetCreate.mockResolvedValue({data: {bearing: '12'}});
        mock.config.keysSetCreate.mockResolvedValue({data: {bearing: false}});
        const {result} = renderHook(() => useConfig(['bearing']));
        await waitFor(() => expect(result.current.config.bearing).toBe('12'));
        await act(async () => { await expect(result.current.setConfig({bearing: '24'})).rejects.toThrow('Configuration value for bearing must be a string'); });
        expect(mock.config.keysSetCreate).toHaveBeenCalledExactlyOnceWith({bearing: '24'});
        expect(result.current.config.bearing).toBe('12');
        expect(mock.notification.error).not.toHaveBeenCalled();
    });

    it('does not submit an invalid write payload arriving from untyped JavaScript', async () => {
        mock.config.keysGetCreate.mockResolvedValue({data: {}});
        const {result} = renderHook(() => useConfig(['bearing']));
        await waitFor(() => expect(mock.config.keysGetCreate).toHaveBeenCalledOnce());
        // Reflect models the JS boundary without an unsafe assertion in TypeScript.
        await act(async () => {
            const writeResult: unknown = Reflect.apply(result.current.setConfig, undefined, [{bearing: 24}]);
            await expect(Promise.resolve(writeResult)).rejects.toThrow('Configuration value for bearing must be a string');
        });
        expect(mock.config.keysSetCreate).not.toHaveBeenCalled();
        expect(mock.notification.error).not.toHaveBeenCalled();
    });

    it('accepts a valid write echo and merges only the returned keys', async () => {
        mock.config.keysGetCreate.mockResolvedValue({data: {bearing: '12', other: ''}});
        mock.config.keysSetCreate.mockResolvedValue({data: {bearing: '24'}});
        const {result} = renderHook(() => useConfig(['bearing', 'other']));
        await waitFor(() => expect(result.current.config.bearing).toBe('12'));
        await act(async () => { await result.current.setConfig({bearing: '24'}); });
        expect(result.current.config).toEqual({bearing: '24', other: ''});
        expect(mock.notification.error).not.toHaveBeenCalled();
    });
});


describe('configuration failure propagation to the settings caller', () => {
    beforeEach(() => {
        mock.config.keysGetCreate.mockReset().mockResolvedValue({data: {}});
        mock.config.keysSetCreate.mockReset();
        mock.settings.settingsList.mockReset().mockResolvedValue({data: {settings: {}}});
        mock.settings.yamlList.mockReset().mockResolvedValue({data: {}});
        mock.notification.error.mockClear();
        mock.notification.success.mockClear();
    });

    it('does not announce saved settings when the configuration write rejects', async () => {
        mock.config.keysSetCreate.mockRejectedValueOnce({error: {error: 'Storage unavailable'}});
        const {result} = renderHook(useSettings);
        await waitFor(() => expect(result.current.loading).toBe(false));
        await act(async () => {
            const saved: unknown = Reflect.apply(result.current.setSettings, undefined, [{'system.ros.nodeName': 'new-name'}]);
            await saved;
        });
        expect(mock.notification.error).toHaveBeenCalledExactlyOnceWith({message: `${mock.language}:useSettingsHook.saveFailed`, description: 'Storage unavailable'});
        expect(mock.notification.success).not.toHaveBeenCalled();
        expect(result.current.loading).toBe(false);
    });

    it('does not submit or announce an invalid string-store value', async () => {
        const {result} = renderHook(useSettings);
        await waitFor(() => expect(result.current.loading).toBe(false));
        await act(async () => {
            const saved: unknown = Reflect.apply(result.current.setSettings, undefined, [{'system.ros.nodeName': 42}]);
            await saved;
        });
        expect(mock.config.keysSetCreate).not.toHaveBeenCalled();
        expect(mock.notification.error).toHaveBeenCalledExactlyOnceWith({message: `${mock.language}:useSettingsHook.saveFailed`, description: 'Configuration value for system.ros.nodeName must be a string'});
        expect(mock.notification.success).not.toHaveBeenCalled();
        expect(result.current.loading).toBe(false);
    });
});

describe('Batch 10 settings boundaries', () => {
    beforeEach(() => {
        mock.config.keysGetCreate.mockReset().mockResolvedValue({data: {}});
        mock.config.keysSetCreate.mockReset();
        mock.settings.settingsList.mockReset().mockResolvedValue({data: {settings: {}}});
        mock.settings.yamlList.mockReset().mockResolvedValue({data: {}});
        mock.notification.error.mockClear(); mock.notification.success.mockClear();
    });
    describe('shell settings read envelope', () => {
        afterEach(() => {
            expect(mock.settings.settingsList).toHaveBeenCalledOnce();
            expect(mock.settings.yamlList).toHaveBeenCalledOnce();
            expect(mock.config.keysGetCreate).toHaveBeenCalledOnce();
            expect(mock.config.keysSetCreate).not.toHaveBeenCalled();
            expect(mock.notification.success).not.toHaveBeenCalled();
        });
        it.each([
            {settings: {}},
            {settings: {zero: 0, disabled: false, text: '', nil: null}},
        ])('accepts the canonical envelope without writes or notifications: %j', async data => {
            mock.settings.settingsList.mockResolvedValue({data});
            const {result, rerender} = renderHook(useSettings);
            await waitFor(() => expect(result.current.loading).toBe(false));
            expect(result.current.settings).toEqual(data.settings);
            rerender();
            expect(mock.notification.error).not.toHaveBeenCalled();
        });
        it.each([
            {}, {settings: null}, {settings: []}, {settings: 'invalid'},
            {settings: 0}, {settings: false}, {settings: {invalid: undefined}},
        ])('rejects malformed envelopes once while keeping YAML usable: %j', async data => {
            mock.settings.settingsList.mockResolvedValue({data});
            mock.settings.yamlList.mockResolvedValue({data: {lidar_x: 0, enabled: false, text: '', nil: null}});
            const {result, rerender} = renderHook(useSettings);
            await waitFor(() => expect(result.current.loading).toBe(false));
            expect(result.current.settings).toEqual({lidar_x: 0, enabled: false, text: '', nil: null});
            expect(mock.notification.error).toHaveBeenCalledExactlyOnceWith({
                message: `${mock.language}:useSettingsHook.loadFailed`, description: 'Invalid settings payload',
            });
            rerender();
            expect(mock.notification.error).toHaveBeenCalledOnce();
        });
        it.each(['error body', 'HTTP failure', 'rejected request'])('reports %s once without losing YAML', async failure => {
            const response = {status: 500, ok: false, error: {error: 'Shell read failed'}};
            if (failure === 'rejected request') mock.settings.settingsList.mockRejectedValue(response);
            else mock.settings.settingsList.mockResolvedValue(failure === 'error body'
                ? {data: {settings: {}}, error: response.error} : response);
            mock.settings.yamlList.mockResolvedValue({data: {lidar_x: 0}});
            const {result, rerender} = renderHook(useSettings);
            await waitFor(() => expect(result.current.loading).toBe(false));
            expect(result.current.settings).toEqual({lidar_x: 0});
            expect(mock.notification.error).toHaveBeenCalledExactlyOnceWith({
                message: `${mock.language}:useSettingsHook.loadFailed`, description: 'Shell read failed',
            });
            rerender();
            expect(mock.notification.error).toHaveBeenCalledOnce();
        });
    });
    it('keeps DB ownership, YAML precedence and optional values', async () => {
        mock.config.keysGetCreate.mockResolvedValue({data: {'system.ros.nodeName': 'db', 'system.mqtt.enabled': 'false'}});
        mock.settings.settingsList.mockResolvedValue({data: {settings: {'system.ros.nodeName': 'shell', shell: '', zero: 1}}});
        mock.settings.yamlList.mockResolvedValue({data: {'system.ros.nodeName': 'yaml', zero: 0, list: [1], nil: null}});
        const {result} = renderHook(useSettings);
        await waitFor(() => expect(result.current.loading).toBe(false));
        expect(result.current.settings).toEqual({'system.ros.nodeName': 'db', 'system.mqtt.enabled': false, shell: '', zero: 0, list: [1], nil: null});
    });
    it.each([404, 405])('supports older backends without YAML endpoint (%s)', async status => {
        mock.settings.yamlList.mockResolvedValue({status, ok: false, error: {error: 'Missing'}});
        const {result} = renderHook(useSettings);
        await waitFor(() => expect(result.current.loading).toBe(false));
        expect(mock.notification.error).not.toHaveBeenCalled();
    });
    it.each([{data: {}}, {data: {settings: []}}, {data: {settings: {}}, error: {error: 'Failed'}}])('reports invalid shell responses while retaining independent YAML data %j', async response => {
        mock.settings.settingsList.mockResolvedValue(response);
        mock.settings.yamlList.mockResolvedValue({data: {lidar_x: 0}});
        const {result} = renderHook(useSettings);
        await waitFor(() => expect(result.current.loading).toBe(false));
        expect(result.current.settings.lidar_x).toBe(0);
        expect(mock.notification.error).toHaveBeenCalledOnce();
    });
    it('does not route YAML settings to the DB or claim persistence', async () => {
        const {result} = renderHook(useSettings);
        await waitFor(() => expect(result.current.loading).toBe(false));
        await act(async () => result.current.setSettings({dock_pose_x: 0, nested: {array: [1], nil: null}}));
        expect(mock.config.keysSetCreate).not.toHaveBeenCalled();
        expect(mock.notification.success).not.toHaveBeenCalled();
    });
    it.each([{data: {}}, {data: {'system.ros.nodeName': 'different'}}, {data: {'system.ros.nodeName': 'new'}, error: {error: 'Rejected'}}])('never announces success for an unconfirmed DB write %j', async response => {
        mock.config.keysSetCreate.mockResolvedValue(response);
        const {result} = renderHook(useSettings);
        await waitFor(() => expect(result.current.loading).toBe(false));
        await act(async () => result.current.setSettings({'system.ros.nodeName': 'new'}));
        expect(mock.notification.success).not.toHaveBeenCalled();
        expect(mock.notification.error).toHaveBeenCalledOnce();
    });
    it('waits for DB loading and retains a successful edit against a late initial read', async () => {
        let resolve: (value: unknown) => void = () => {};
        mock.config.keysGetCreate.mockReturnValue(new Promise(r => {resolve = r;}));
        mock.config.keysSetCreate.mockResolvedValue({data: {'system.ros.nodeName': 'new'}});
        const {result} = renderHook(useSettings);
        await act(async () => {await Promise.resolve();});
        expect(result.current.loading).toBe(true);
        await act(async () => result.current.setSettings({'system.ros.nodeName': 'new'}));
        await act(async () => {resolve({data: {'system.ros.nodeName': 'old'}}); await Promise.resolve();});
        await waitFor(() => expect(result.current.loading).toBe(false));
        expect(result.current.settings['system.ros.nodeName']).toBe('new');
    });
    it('submits only one concurrent DB write and avoids post-unmount notifications', async () => {
        let resolve: (value: unknown) => void = () => {};
        mock.config.keysSetCreate.mockReturnValue(new Promise(r => {resolve = r;}));
        const {result, unmount} = renderHook(useSettings);
        await waitFor(() => expect(result.current.loading).toBe(false));
        let first: Promise<void>;
        act(() => {first = result.current.setSettings({'system.ros.nodeName': 'new'});});
        await act(async () => result.current.setSettings({'system.ros.nodeName': 'second'}));
        expect(mock.config.keysSetCreate).toHaveBeenCalledOnce();
        unmount();
        await act(async () => {resolve({data: {'system.ros.nodeName': 'new'}}); await first;});
        expect(mock.notification.success).not.toHaveBeenCalled();
    });
});
