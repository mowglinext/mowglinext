import {act, cleanup, fireEvent, render, screen} from '@testing-library/react';
import {afterEach, beforeEach, expect, it, vi} from 'vitest';
import type {ModalFuncProps} from 'antd';
import {PowerMenu} from './PowerMenu';
const mocks = vi.hoisted(() => ({fetch: vi.fn<typeof globalThis.fetch>(), confirm: vi.fn<(props: ModalFuncProps) => void>(),
    notification: {success: vi.fn(), error: vi.fn(), warning: vi.fn()}, board: vi.fn()}));
vi.mock('../hooks/useApi.ts', async () => {
    const {Api} = await import('../api/Api');
    const api = new Api<unknown>({customFetch: mocks.fetch});
    api.baseUrl = "";
    return {useApi: () => api};
});
vi.mock('./MowerActions.tsx', () => ({useMowerAction: () => () => mocks.board}));
vi.mock('antd', async original => ({...await original<typeof import('antd')>(), App: {useApp: () => ({modal: {confirm: mocks.confirm}, notification: mocks.notification})}}));
const json = (value: unknown, status = 200) => new Response(JSON.stringify(value), {status});
const posts = () => mocks.fetch.mock.calls.filter(([, init]) => init?.method === 'POST');
let polling: ReturnType<typeof vi.fn<typeof globalThis.fetch>>;
beforeEach(() => {
    vi.clearAllMocks();
    mocks.fetch.mockResolvedValue(json({}));
    polling = vi.fn<typeof globalThis.fetch>().mockResolvedValue(json({}, 503));
    vi.stubGlobal('fetch', polling);
});
afterEach(() => {cleanup(); vi.useRealTimers(); vi.unstubAllGlobals();});
async function confirmHost(label: 'Restart Host' | 'Shut down Host') {
    const view = render(<PowerMenu hostOnly><button>Power</button></PowerMenu>);
    fireEvent.click(screen.getByRole('button', {name: 'Power'}));
    fireEvent.click(await screen.findByText(label));
    expect(mocks.confirm).toHaveBeenCalledOnce(); expect(posts()).toHaveLength(0);
    return view;
}
async function approve() {await act(async () => {await mocks.confirm.mock.calls[0][0].onOk?.();});}
it('requires contextual confirmation and Cancel sends no request', async () => {
    await confirmHost('Shut down Host');
    await act(async () => {await mocks.confirm.mock.calls[0][0].onCancel?.();});
    expect(posts()).toHaveLength(0); expect(mocks.notification.success).not.toHaveBeenCalled();
});
it('acknowledges shutdown only after explicit HTTP success; repeated confirm cannot resend', async () => {
    await confirmHost('Shut down Host'); await approve(); await approve();
    expect(posts()).toHaveLength(1); expect(posts()[0][0]).toBe('/system/shutdown');
    expect(mocks.notification.success).toHaveBeenCalledOnce(); expect(polling).not.toHaveBeenCalled();
});
it.each(['Shut down Host', 'Restart Host'] as const)('reports HTTP rejection truthfully for %s', async label => {
    mocks.fetch.mockImplementation(() => Promise.resolve(json({error: 'Host denied'}, 500)));
    await confirmHost(label); await approve();
    expect(mocks.notification.error).toHaveBeenCalledWith(expect.objectContaining({description: 'Host denied'}));
    expect(mocks.notification.success).not.toHaveBeenCalled(); expect(polling).not.toHaveBeenCalled();
    expect(screen.queryByRole('dialog')).not.toBeInTheDocument();
});
it.each([null, {ok: false}, {error: ['invalid']}])('does not confirm shutdown with malformed acknowledgement %j', async raw => {
    mocks.fetch.mockImplementation(() => Promise.resolve(json(raw)));
    await confirmHost('Shut down Host'); await approve();
    expect(mocks.notification.warning).toHaveBeenCalledOnce(); expect(mocks.notification.success).not.toHaveBeenCalled(); expect(posts()).toHaveLength(1);
});
it('lost shutdown response is unconfirmed, with no reconnect or automatic retry', async () => {
    mocks.fetch.mockRejectedValue(new TypeError('Connection lost'));
    await confirmHost('Shut down Host'); await approve();
    expect(mocks.notification.warning).toHaveBeenCalledWith(expect.objectContaining({message: 'Status unknown'}));
    expect(mocks.notification.error).not.toHaveBeenCalled(); expect(mocks.notification.success).not.toHaveBeenCalled(); expect(posts()).toHaveLength(1);
});
it('reboot transport loss keeps reconnect observation without claiming success or resending commands', async () => {
    mocks.fetch.mockRejectedValue(new TypeError('Connection lost'));
    const view = await confirmHost('Restart Host'); vi.useFakeTimers(); await approve(); await approve();
    expect(mocks.notification.warning).toHaveBeenCalledOnce(); expect(mocks.notification.error).not.toHaveBeenCalled();
    expect(screen.getByRole('dialog')).toBeInTheDocument();
    await act(async () => {await vi.advanceTimersByTimeAsync(15000);});
    expect(polling).toHaveBeenCalledWith('/api/system/info', {cache: 'no-store'}); expect(posts()).toHaveLength(1);
    view.unmount(); await vi.advanceTimersByTimeAsync(30000); expect(polling).toHaveBeenCalledOnce();
});
it('prevents concurrent power requests while shutdown is pending', async () => {
    let complete: ((response: Response) => void) | undefined;
    mocks.fetch.mockImplementation(() => new Promise(resolve => {complete = resolve;}));
    await confirmHost('Shut down Host');
    let first: unknown;
    await act(async () => {first = mocks.confirm.mock.calls[0][0].onOk?.(); await Promise.resolve();});
    await approve(); expect(posts()).toHaveLength(1);
    await act(async () => {complete?.(json({})); await first;});
    expect(mocks.notification.success).toHaveBeenCalledOnce();
});
it.each([500, 0])('observes detached GUI restart errors (HTTP status %s) without another command', async status => {
    mocks.fetch.mockImplementation((_url, init) => {
        if (init?.method === 'GET') return Promise.resolve(json({containers: [{id: 'gui', names: ['/mowgli-gui']}]}));
        return status ? Promise.resolve(json({error: 'Docker denied'}, status)) : Promise.reject(new TypeError('lost'));
    });
    const view = render(<PowerMenu><button>Power</button></PowerMenu>);
    fireEvent.click(screen.getByRole('button', {name: 'Power'})); fireEvent.click(await screen.findByText('Restart Mowgli'));
    vi.useFakeTimers(); await approve();
    await act(async () => {await vi.advanceTimersByTimeAsync(0);});
    if (status) {
        expect(mocks.notification.error).toHaveBeenCalledOnce();
        await act(async () => {await vi.advanceTimersByTimeAsync(15000);}); expect(polling).not.toHaveBeenCalled();
        expect(screen.getByRole('dialog')).toHaveClass('ant-zoom-leave');
    } else {
        expect(mocks.notification.warning).toHaveBeenCalledOnce(); expect(screen.getByRole('dialog')).toBeInTheDocument();
        await act(async () => {await vi.advanceTimersByTimeAsync(15000);}); expect(polling).toHaveBeenCalledOnce();
    }
    expect(posts()).toHaveLength(1); expect(mocks.notification.success).not.toHaveBeenCalled(); view.unmount();
});
