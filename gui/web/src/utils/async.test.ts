import {describe, expect, it, vi} from 'vitest';
import {asyncEvent, observeAsync} from './async';

const flush = async () => { await Promise.resolve(); await Promise.resolve(); await Promise.resolve(); };

describe('detached asynchronous callbacks', () => {
    it('starts immediately and preserves concurrent calls and arguments', async () => {
        const events: number[] = [];
        let finish: (() => void) | undefined;
        const pending = new Promise<void>(resolve => { finish = resolve; });
        const callback = asyncEvent(async (value: number) => {
            events.push(value);
            await pending;
            events.push(value * 10);
        }, vi.fn());
        expect(callback(1)).toBeUndefined();
        callback(2);
        expect(events).toEqual([1, 2]);
        finish?.();
        await flush();
        expect(events).toEqual([1, 2, 10, 20]);
    });

    it('reports a rejected command exactly once', async () => {
        const error = new Error('Robot refused rearm');
        const report = vi.fn();
        const callback = asyncEvent(() => Promise.reject(error), report);
        callback();
        await flush();
        expect(report).toHaveBeenCalledExactlyOnceWith(error);
    });

    it('reports synchronous callback failures', () => {
        const error = new Error('Callback failed');
        const report = vi.fn();
        asyncEvent(() => { throw error; }, report)();
        expect(report).toHaveBeenCalledExactlyOnceWith(error);
    });

    it('does not report a failure already handled inline', async () => {
        const inline = vi.fn();
        const report = vi.fn();
        asyncEvent(async () => {
            try { await Promise.reject(new Error('Cannot write settings')); }
            catch (error) { inline(error); }
        }, report)();
        await flush();
        expect(inline).toHaveBeenCalledOnce();
        expect(report).not.toHaveBeenCalled();
    });

    it('logs a failure in the error reporter instead of leaving a new rejection', async () => {
        const brokenReporter = new Error('Toast failed');
        const consoleError = vi.spyOn(console, 'error').mockImplementation(() => undefined);
        observeAsync(Promise.reject(new Error('Request failed')), () => { throw brokenReporter; });
        await flush();
        expect(consoleError).toHaveBeenCalledWith('Asynchronous GUI operation failed', brokenReporter);
        consoleError.mockRestore();
    });
});
