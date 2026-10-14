import {act, cleanup, fireEvent, render, screen} from '@testing-library/react';
import {afterEach, expect, it, vi} from 'vitest';
import type {ModalFuncProps} from 'antd';
import {ParametersPage} from './ParametersPage';
import {getColors} from '../theme/colors';
const mocks = vi.hoisted(() => ({api: {request: vi.fn<(args: {method: string; path: string; body?: unknown}) => Promise<{data: unknown}>>()}, modal: {confirm: vi.fn<(props: ModalFuncProps) => void>()}, notification: {success: vi.fn(), error: vi.fn()}}));
vi.mock('../hooks/useApi.ts', () => ({useApi: () => mocks.api}));
vi.mock('../hooks/useIsMobile.ts', () => ({useIsMobile: () => false}));
vi.mock('../theme/ThemeContext.tsx', () => ({useThemeMode: () => ({colors: getColors('dark')})}));
vi.mock('antd', async original => ({...await original<typeof import('antd')>(), App: {useApp: () => ({modal: mocks.modal, notification: mocks.notification})}}));
afterEach(cleanup);
it('retains live ROS safety confirmation before a numeric Enter commit', async () => {
    const api = mocks.api;
    api.request.mockImplementation((args: {method: string}) => Promise.resolve({data: args.method === 'GET'
        ? {parameters: [{name: 'transit_speed', value: .2, type: 'double'}]} : {}}));
    mocks.modal.confirm.mockClear(); const error = vi.spyOn(console, 'error');
    try {
        render(<ParametersPage/>);
        const input = await screen.findByRole('spinbutton');
        fireEvent.change(input, {target: {value: '0.3'}}); fireEvent.keyDown(input, {key: 'Enter', code: 'Enter'});
        expect(mocks.modal.confirm).toHaveBeenCalledOnce();
        expect(api.request.mock.calls.some(([args]) => args.method === 'POST')).toBe(false);
        await act(async () => {await mocks.modal.confirm.mock.calls[0][0].onOk?.();});
        expect(api.request).toHaveBeenCalledWith(expect.objectContaining({path: '/params', method: 'POST'}));
        expect(error.mock.calls.some(args => args.some(arg => typeof arg === 'string' && arg.includes('[antd: InputNumber]') && /addonAfter|addonBefore/.test(arg)))).toBe(false);
    } finally {error.mockRestore();}
});
