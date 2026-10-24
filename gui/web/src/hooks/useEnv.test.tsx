import {renderHook, waitFor} from '@testing-library/react';
import {beforeEach, describe, expect, it, vi} from 'vitest';
import {useEnv} from './useEnv';

const mock = vi.hoisted(() => ({config: {envsList: vi.fn()}, notification: {error: vi.fn()}}));
vi.mock('./useApi.ts', () => ({useApi: () => ({config: mock.config})}));
vi.mock('antd', () => ({App: {useApp: () => ({notification: mock.notification})}}));
vi.mock('react-i18next', () => ({useTranslation: () => ({t: (key: string) => key})}));

describe('environment response boundary', () => {
    beforeEach(() => { mock.config.envsList.mockReset(); mock.notification.error.mockClear(); });

    it.each([{}, {tileUri: ''}, {tileUri: '/tiles/{z}/{x}/{y}.png'}])('accepts optional environment data without inventing an overlay: %j', data => {
        mock.config.envsList.mockResolvedValue({data});
        const {result} = renderHook(useEnv);
        return waitFor(() => expect(result.current).toEqual(data));
    });

    it.each([null, [], {tileUri: 0}, {tileUri: {url: '/tiles'}}])('reports malformed environment data and keeps the empty fallback: %j', data => {
        mock.config.envsList.mockResolvedValue({data});
        const {result} = renderHook(useEnv);
        return waitFor(() => {
            expect(mock.notification.error).toHaveBeenCalledOnce();
            expect(result.current).toEqual({});
        });
    });
});
