import {StrictMode} from 'react';
import {act, cleanup, fireEvent, render, screen} from '@testing-library/react';
import {afterEach, beforeEach, describe, expect, it, vi} from 'vitest';
import {App} from 'antd';

const api = vi.hoisted(() => ({containers: {containersList: vi.fn(), containersCreate: vi.fn()}}));
vi.mock('../hooks/useApi.ts', () => ({useApi: () => api}));

import LogsPage from './LogsPage';

// Keep the real useWS and react-use-websocket subscription/cleanup paths.
// Only the browser transport and Docker HTTP requests are simulated.
class Socket extends EventTarget {
    static CONNECTING = 0;
    static OPEN = 1;
    static CLOSING = 2;
    static CLOSED = 3;
    static instances: Socket[] = [];
    readyState = Socket.CONNECTING;
    onopen: ((event: Event) => void) | null = null;
    onmessage: ((event: MessageEvent) => void) | null = null;
    onclose: ((event: CloseEvent) => void) | null = null;
    onerror: ((event: Event) => void) | null = null;
    constructor(readonly url: string) { super(); Socket.instances.push(this); }
    send = vi.fn();
    close = vi.fn(() => { this.readyState = Socket.CLOSING; });
    open() { this.readyState = Socket.OPEN; this.onopen?.(new Event('open')); }
    line(line: string) { this.onmessage?.(new MessageEvent('message', {data: btoa(line)})); }
    closed() { this.readyState = Socket.CLOSED; this.onclose?.(new CloseEvent('close')); }
}

function deferred<T>() {
    let resolve!: (value: T) => void;
    let reject!: (reason: Error) => void;
    const promise = new Promise<T>((yes, no) => { resolve = yes; reject = no; });
    return {promise, resolve, reject};
}

const containers = (a = 'running', b = 'running') => ({data: {containers: [
    {id: 'alpha', names: ['/alpha'], state: a, labels: {}},
    {id: 'beta', names: ['/beta'], state: b, labels: {}},
]}});
type ListResult = ReturnType<typeof containers>;
const commandResult = {data: {}};

async function settle(work: () => void = () => {}) {
    await act(async () => { work(); await Promise.resolve(); });
}

async function mount(strict = false) {
    const page = <App><LogsPage/></App>;
    const view = render(strict ? <StrictMode>{page}</StrictMode> : page);
    await settle();
    expect(current().url).toMatch(/\/alpha\/logs$/);
    act(() => current().open());
    return view;
}

const current = () => Socket.instances[Socket.instances.length - 1];

async function select(name: 'alpha' | 'beta') {
    fireEvent.mouseDown(screen.getByRole('combobox'));
    const option = document.querySelector(`.ant-select-item-option[title="${name}"]`);
    expect(option).not.toBeNull();
    fireEvent.click(option!);
    await settle();
    expect(document.querySelector('.ant-select-selection-item')).toHaveAttribute('title', name);
    expect(current().url).toMatch(new RegExp(`/${name}/logs$`));
    act(() => current().open());
}

function command(name: 'Start' | 'Stop' | 'Restart') {
    fireEvent.click(screen.getByRole('button', {name}));
}

function flush() { act(() => { vi.advanceTimersByTime(100); }); }

describe('LogsPage operation ownership', () => {
    beforeEach(() => {
        vi.useFakeTimers();
        vi.stubGlobal('WebSocket', Socket);
        Socket.instances = [];
        api.containers.containersList.mockReset().mockResolvedValue(containers());
        api.containers.containersCreate.mockReset();
    });
    afterEach(async () => {
        cleanup();
        await settle(() => Socket.instances.forEach(socket => socket.closed()));
        vi.clearAllTimers();
        vi.useRealTimers();
        vi.unstubAllGlobals();
    });

    it.each(['Restart', 'Start'] as const)('does not reopen alpha after a late %s while beta is selected', async name => {
        api.containers.containersList.mockResolvedValue(containers(name === 'Start' ? 'exited' : 'running'));
        const pending = deferred<typeof commandResult>();
        api.containers.containersCreate.mockReturnValueOnce(pending.promise);
        await mount();
        command(name);
        expect(api.containers.containersCreate).toHaveBeenCalledWith('alpha', name.toLowerCase());
        await select('beta');
        const beta = current();
        const opened = Socket.instances.length;
        await settle(() => pending.resolve(commandResult));
        expect(Socket.instances).toHaveLength(opened);
        expect(current()).toBe(beta);
        expect(beta.close).not.toHaveBeenCalled();
        act(() => beta.line('INFO beta remains authoritative'));
        flush();
        expect(screen.getByText('INFO beta remains authoritative')).toBeInTheDocument();
    });

    it('does not close beta after a late stop of alpha', async () => {
        const pending = deferred<typeof commandResult>();
        api.containers.containersCreate.mockReturnValueOnce(pending.promise);
        await mount();
        command('Stop');
        await select('beta');
        const beta = current();
        await settle(() => pending.resolve(commandResult));
        expect(beta.close).not.toHaveBeenCalled();
        act(() => beta.line('INFO beta after old stop'));
        flush();
        expect(screen.getByText('INFO beta after old stop')).toBeInTheDocument();
    });

    it('keeps beta authoritative when multiple alpha commands resolve backwards', async () => {
        const restart = deferred<typeof commandResult>();
        const stop = deferred<typeof commandResult>();
        api.containers.containersCreate.mockReturnValueOnce(restart.promise).mockReturnValueOnce(stop.promise);
        await mount();
        command('Restart');
        command('Stop');
        await select('beta');
        const beta = current();
        const count = Socket.instances.length;
        await settle(() => stop.resolve(commandResult));
        await settle(() => restart.resolve(commandResult));
        expect(Socket.instances).toHaveLength(count);
        expect(current()).toBe(beta);
        expect(beta.close).not.toHaveBeenCalled();
    });

    it('does not revive an earlier restart after a newer stop of the same selection', async () => {
        const restart = deferred<typeof commandResult>();
        const stop = deferred<typeof commandResult>();
        api.containers.containersCreate.mockReturnValueOnce(restart.promise).mockReturnValueOnce(stop.promise);
        await mount();
        const alpha = current();
        command('Restart');
        command('Stop');
        await settle(() => stop.resolve(commandResult));
        expect(alpha.close).toHaveBeenCalledOnce();
        await settle(() => alpha.closed());
        const count = Socket.instances.length;
        await settle(() => restart.resolve(commandResult));
        expect(Socket.instances).toHaveLength(count);
    });

    it.each(['Start', 'Restart', 'Stop'] as const)('preserves %s on the current selection and refreshes the list', async name => {
        api.containers.containersList.mockResolvedValue(containers(name === 'Start' ? 'exited' : 'running'));
        const pending = deferred<typeof commandResult>();
        api.containers.containersCreate.mockReturnValueOnce(pending.promise);
        await mount();
        const alpha = current();
        command(name);
        await settle(() => pending.resolve(commandResult));
        expect(api.containers.containersList).toHaveBeenCalledTimes(2);
        if (name === 'Stop') {
            expect(alpha.close).toHaveBeenCalledOnce();
        } else {
            // Restarting the same URI is not an added reconnect policy.
            expect(current().url).toMatch(/\/alpha\/logs$/);
            expect(current().close).not.toHaveBeenCalled();
            act(() => current().line('INFO current command works'));
            flush();
            expect(screen.getByText('INFO current command works')).toBeInTheDocument();
        }
    });

    it('invalidates the original operation across alpha -> beta -> alpha', async () => {
        const pending = deferred<typeof commandResult>();
        api.containers.containersCreate.mockReturnValueOnce(pending.promise);
        await mount();
        command('Stop');
        await select('beta');
        await select('alpha');
        const freshAlpha = current();
        await settle(() => pending.resolve(commandResult));
        expect(freshAlpha.close).not.toHaveBeenCalled();
        api.containers.containersCreate.mockResolvedValueOnce(commandResult);
        command('Restart');
        await settle();
        expect(api.containers.containersCreate).toHaveBeenLastCalledWith('alpha', 'restart');
        expect(current()).toBe(freshAlpha);
        expect(freshAlpha.close).not.toHaveBeenCalled();
    });

    it('does not reopen a socket or request a list after unmount during restart', async () => {
        const pending = deferred<typeof commandResult>();
        api.containers.containersCreate.mockReturnValueOnce(pending.promise);
        const view = await mount();
        command('Restart');
        view.unmount();
        const count = Socket.instances.length;
        const lists = api.containers.containersList.mock.calls.length;
        await settle(() => pending.resolve(commandResult));
        expect(Socket.instances).toHaveLength(count);
        expect(api.containers.containersList).toHaveBeenCalledTimes(lists);
    });

    it('drops alpha pending batches and late callbacks without mixing beta lines or severity', async () => {
        await mount();
        const alpha = current();
        act(() => alpha.line('WARNING user interrupted with ctrl-c (SIGINT)'));
        await select('beta');
        const beta = current();
        // The real shared-socket listener must find no old alpha subscribers.
        act(() => alpha.line('INFO late alpha'));
        await settle(() => alpha.closed());
        act(() => beta.line('ERROR process has died exit code -2'));
        flush();
        expect(screen.queryByText(/late alpha|user interrupted/)).not.toBeInTheDocument();
        expect(screen.getAllByTestId('log-line')).toHaveLength(1);
        expect(screen.getByTestId('log-line')).toHaveTextContent('ERROR');
        expect(screen.getByText('ERROR process has died exit code -2')).toBeInTheDocument();
        expect(screen.getByRole('button', {name: 'Errors 1'})).toBeInTheDocument();
        expect(beta.close).not.toHaveBeenCalled();
    });

    it('cancels a pending batch on unmount and accepts only the new page stream', async () => {
        const view = await mount();
        const old = current();
        act(() => old.line('INFO abandoned batch'));
        view.unmount();
        await settle(() => old.closed());
        await mount();
        act(() => current().line('INFO new page'));
        flush();
        expect(screen.queryByText('INFO abandoned batch')).not.toBeInTheDocument();
        expect(screen.getByText('INFO new page')).toBeInTheDocument();
    });

    it('discards older list refreshes and never changes the current selection', async () => {
        const restart = deferred<typeof commandResult>();
        const stop = deferred<typeof commandResult>();
        const oldList = deferred<ListResult>();
        const newList = deferred<ListResult>();
        api.containers.containersCreate.mockReturnValueOnce(restart.promise).mockReturnValueOnce(stop.promise);
        api.containers.containersList.mockResolvedValueOnce(containers())
            .mockReturnValueOnce(oldList.promise).mockReturnValueOnce(newList.promise);
        await mount();
        command('Restart');
        command('Stop');
        await select('beta');
        await settle(() => restart.resolve(commandResult));
        await settle(() => stop.resolve(commandResult));
        await settle(() => newList.resolve(containers('exited', 'exited')));
        expect(screen.getByRole('button', {name: /Start$/})).toBeInTheDocument();
        await settle(() => oldList.resolve(containers()));
        expect(screen.getByRole('button', {name: /Start$/})).toBeInTheDocument();
        expect(document.querySelector('.ant-select-selection-item')).toHaveAttribute('title', 'beta');
    });

    it('ignores the retired StrictMode initial list and keeps a later selection', async () => {
        const retired = deferred<ListResult>();
        api.containers.containersList.mockReturnValueOnce(retired.promise).mockResolvedValueOnce(containers());
        await mount(true);
        await select('beta');
        await settle(() => retired.resolve(containers('running', 'exited')));
        expect(screen.getByRole('button', {name: 'Stop'})).toBeInTheDocument();
        expect(document.querySelector('.ant-select-selection-item')).toHaveAttribute('title', 'beta');
    });

    it('keeps the selected stream intact after a rejected Docker command', async () => {
        const pending = deferred<typeof commandResult>();
        api.containers.containersCreate.mockReturnValueOnce(pending.promise);
        await mount();
        command('Restart');
        await select('beta');
        const beta = current();
        await settle(() => pending.reject(new Error('Docker refused')));
        expect(beta.close).not.toHaveBeenCalled();
        expect(current()).toBe(beta);
        expect(screen.getByText('Docker refused')).toBeInTheDocument();
    });

    it('preserves functional command refusals without touching the stream', async () => {
        api.containers.containersCreate.mockResolvedValueOnce({error: {error: 'functional refusal'}});
        await mount();
        const alpha = current();
        command('Stop');
        await settle();
        expect(alpha.close).not.toHaveBeenCalled();
        expect(api.containers.containersList).toHaveBeenCalledTimes(1);
        expect(screen.getByText('functional refusal')).toBeInTheDocument();
    });

    it('keeps the current stream and selection when a list refresh fails', async () => {
        api.containers.containersCreate.mockResolvedValueOnce(commandResult);
        api.containers.containersList.mockResolvedValueOnce(containers())
            .mockRejectedValueOnce(new Error('list unavailable'));
        await mount();
        command('Restart');
        await settle();
        expect(current().url).toMatch(/\/alpha\/logs$/);
        expect(current().close).not.toHaveBeenCalled();
        expect(screen.getByText('list unavailable')).toBeInTheDocument();
    });

    it('does not apply a retired initial list after unmount/remount', async () => {
        const retired = deferred<ListResult>();
        api.containers.containersList.mockReturnValueOnce(retired.promise);
        const oldView = render(<App><LogsPage/></App>);
        await settle();
        oldView.unmount();
        api.containers.containersList.mockResolvedValueOnce(containers());
        await mount();
        await select('beta');
        const beta = current();
        const count = Socket.instances.length;
        await settle(() => retired.resolve(containers('running', 'exited')));
        expect(Socket.instances).toHaveLength(count);
        expect(current()).toBe(beta);
        expect(screen.getByRole('button', {name: 'Stop'})).toBeInTheDocument();
    });
});
