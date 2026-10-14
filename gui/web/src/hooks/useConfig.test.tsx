import {act, renderHook, waitFor} from '@testing-library/react';
import {beforeEach, describe, expect, it, vi} from 'vitest';
import {useConfig} from './useConfig';

const mock = vi.hoisted(() => ({
    language: 'en',
    config: {keysGetCreate: vi.fn(), keysSetCreate: vi.fn()},
    notification: {error: vi.fn()},
}));
vi.mock('./useApi.ts', () => ({useApi: () => ({config: mock.config})}));
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
