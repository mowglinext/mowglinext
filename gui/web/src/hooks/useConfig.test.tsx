import {act, renderHook, waitFor} from '@testing-library/react';
import {beforeEach, describe, expect, it, vi} from 'vitest';
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
