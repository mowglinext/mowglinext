import {act, cleanup, fireEvent, render, screen} from '@testing-library/react';
import {afterEach, beforeEach, expect, it, vi, type MockInstance} from 'vitest';
import type {ModalFuncProps} from 'antd';
import {ParametersPage} from './ParametersPage';
import {getColors} from '../theme/colors';
const mocks = vi.hoisted(() => ({api: {request: vi.fn<(args: {method: string; path: string; body?: unknown}) => Promise<{data: unknown}>>()}, modal: {confirm: vi.fn<(props: ModalFuncProps) => void>()}, notification: {success: vi.fn(), error: vi.fn()}}));
vi.mock('../hooks/useApi.ts', () => ({useApi: () => mocks.api}));
vi.mock('../hooks/useIsMobile.ts', () => ({useIsMobile: () => false}));
vi.mock('../theme/ThemeContext.tsx', () => ({useThemeMode: () => ({colors: getColors('dark')})}));
vi.mock('antd', async original => ({...await original<typeof import('antd')>(), App: {useApp: () => ({modal: mocks.modal, notification: mocks.notification})}}));
let errors: MockInstance<typeof console.error>, warnings: MockInstance<typeof console.warn>;
beforeEach(() => {
    mocks.api.request.mockReset(); mocks.modal.confirm.mockClear();
    errors = vi.spyOn(console, 'error'); warnings = vi.spyOn(console, 'warn');
});
afterEach(() => {
    cleanup();
    for (const spy of [errors, warnings]) expect(spy.mock.calls.flat().join(' ')).not.toMatch(/\[antd: Input\].*addon(Before|After)/);
    vi.restoreAllMocks();
});
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

// String-valued fixtures deliberately exercise the text branch without changing
// its existing coercion contract or server-side parameter validation.
it.each(['Enter', 'blur'])('preserves confirmation and exact text values for %s, without a second ROS write', async event => {
    mocks.api.request.mockImplementation(args => Promise.resolve({data: args.method === 'GET'
        ? {parameters: [{name: 'transit_speed', value: '0.2', type: 'string'}]} : {}}));
    render(<ParametersPage/>);
    const input = await screen.findByDisplayValue('0.2');
    fireEvent.change(input, {target: {value: '0.30'}});
    expect(mocks.api.request.mock.calls.filter(([args]) => args.method === 'POST')).toHaveLength(0);
    if (event === 'Enter') {fireEvent.keyDown(input, {key: 'Enter', code: 'Enter'}); fireEvent.keyUp(input, {key: 'Enter', code: 'Enter'});}
    else fireEvent.blur(input);
    expect(mocks.modal.confirm).toHaveBeenCalledOnce();
    expect(mocks.api.request.mock.calls.filter(([args]) => args.method === 'POST')).toHaveLength(0);
    await act(async () => {await mocks.modal.confirm.mock.calls[0][0].onCancel?.();});
    expect(input).toHaveValue('0.2');
    fireEvent.change(input, {target: {value: '0.30'}});
    fireEvent.keyDown(input, {key: 'Enter', code: 'Enter'});
    await act(async () => {await mocks.modal.confirm.mock.calls[1][0].onOk?.();});
    expect(mocks.api.request.mock.calls.filter(([args]) => args.method === 'POST')).toEqual([[expect.objectContaining({
        path: '/params', body: {parameters: [{name: 'transit_speed', value: '0.30'}]},
    })]]);
    fireEvent.blur(input);
    expect(mocks.api.request.mock.calls.filter(([args]) => args.method === 'POST')).toHaveLength(1);
    expect(mocks.modal.confirm).toHaveBeenCalledTimes(2);
});
it('keeps expert text fields disabled until acknowledgment, and opening or searching performs no writes', async () => {
    mocks.api.request.mockResolvedValue({data: {parameters: [{name: 'safety_profile', value: 'fictional-profile', type: 'string'}]}});
    render(<ParametersPage/>);
    fireEvent.click(screen.getByText('Expert'));
    const input = await screen.findByDisplayValue('fictional-profile');
    expect(input).toBeDisabled();
    expect(mocks.modal.confirm).not.toHaveBeenCalled();
    expect(mocks.api.request.mock.calls.filter(([args]) => args.method === 'POST')).toHaveLength(0);
    fireEvent.click(screen.getByRole('button', {name: 'I understand the risks'}));
    expect(input).not.toBeDisabled();
    fireEvent.change(screen.getByPlaceholderText('Search a parameter…'), {target: {value: 'safety_profile'}});
    expect(mocks.api.request.mock.calls.filter(([args]) => args.method === 'POST')).toHaveLength(0);
});
