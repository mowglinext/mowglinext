import {describe, expect, it, vi} from 'vitest';
import {Api} from '../api/Api';
import {savePartialSettingsValues} from './settingsSave';

type SaveParams = Parameters<typeof savePartialSettingsValues>[0];

function setup(response: Response = new Response('{}', {status: 200})) {
    const fetchMock = vi.fn<typeof fetch>().mockResolvedValue(response);
    const api = new Api<unknown>({baseUrl: 'http://gui.test', customFetch: fetchMock});
    const notification = {
        success: vi.fn<SaveParams["notification"]["success"]>(),
        error: vi.fn<SaveParams["notification"]["error"]>(),
    };
    const applyChanged = vi.fn<(changed: Record<string, unknown>) => void>();
    const setBusy = vi.fn<(busy: boolean) => void>();
    const markRestartRequired = vi.fn();
    return {
        fetchMock, notification, applyChanged, setBusy, markRestartRequired,
        params: {
            guiApi: api, notification, applyChanged, setBusy, markRestartRequired,
            defaultSuccessMessage: 'Saved', defaultErrorMessage: 'Save failed',
        },
    };
}

describe('savePartialSettingsValues', () => {
    it('does not write or publish a change when the partial values are unchanged', async () => {
        const ctx = setup();
        const ok = await savePartialSettingsValues({
            ...ctx.params, currentValues: {enabled: false, custom: [1, 2]},
            partialValues: {enabled: false, custom: [1, 2]},
        });
        expect(ok).toBe(true);
        expect(ctx.fetchMock).not.toHaveBeenCalled();
        expect(ctx.applyChanged).not.toHaveBeenCalled();
        expect(ctx.notification.success).not.toHaveBeenCalled();
        expect(ctx.markRestartRequired).not.toHaveBeenCalled();
        expect(ctx.setBusy.mock.calls).toEqual([[false]]);
    });

    it('writes only changed keys and preserves opaque values and references after success', async () => {
        const ctx = setup();
        const custom = {nested: [1, null, false]};
        const ok = await savePartialSettingsValues({
            ...ctx.params, currentValues: {enabled: false, untouched: 4},
            partialValues: {enabled: false, custom, nullable: null},
        });
        expect(ok).toBe(true);
        expect(ctx.fetchMock).toHaveBeenCalledTimes(1);
        expect(ctx.fetchMock.mock.calls[0][0]).toBe('http://gui.test/settings/yaml');
        expect(ctx.fetchMock.mock.calls[0][1]?.method).toBe('POST');
        expect(ctx.fetchMock.mock.calls[0][1]?.body).toBe(JSON.stringify({custom, nullable: null}));
        expect(ctx.applyChanged).toHaveBeenCalledExactlyOnceWith({custom, nullable: null});
        expect(ctx.applyChanged.mock.calls[0][0].custom).toBe(custom);
        expect(ctx.markRestartRequired).toHaveBeenCalledTimes(1);
        expect(ctx.notification.success).toHaveBeenCalledWith({message: 'Saved', description: undefined});
        expect(ctx.notification.error).not.toHaveBeenCalled();
        expect(ctx.setBusy.mock.calls).toEqual([[true], [false]]);
    });

    it('retains silent-success and restart options without suppressing the write', async () => {
        const ctx = setup();
        expect(await savePartialSettingsValues({
            ...ctx.params, currentValues: {}, partialValues: {custom: 0},
            options: {silentSuccess: true, markRestartRequired: false},
        })).toBe(true);
        expect(ctx.fetchMock).toHaveBeenCalledTimes(1);
        expect(ctx.applyChanged).toHaveBeenCalledExactlyOnceWith({custom: 0});
        expect(ctx.markRestartRequired).not.toHaveBeenCalled();
        expect(ctx.notification.success).not.toHaveBeenCalled();
        expect(ctx.setBusy.mock.calls).toEqual([[true], [false]]);
    });

    it.each(['http', 'network'])('keeps failed %s writes unapplied and releases busy', async kind => {
        const ctx = setup(new Response(JSON.stringify({error: 'write rejected'}), {status: 503}));
        if (kind === 'network') ctx.fetchMock.mockRejectedValue(new Error('network lost'));
        expect(await savePartialSettingsValues({
            ...ctx.params, currentValues: {}, partialValues: {custom: 'value'},
        })).toBe(false);
        expect(ctx.fetchMock).toHaveBeenCalledTimes(1);
        expect(ctx.applyChanged).not.toHaveBeenCalled();
        expect(ctx.markRestartRequired).not.toHaveBeenCalled();
        expect(ctx.notification.success).not.toHaveBeenCalled();
        expect(ctx.notification.error).toHaveBeenCalledTimes(1);
        const [notice] = ctx.notification.error.mock.calls[0];
        expect(notice.message).toBe('Save failed');
        expect(typeof notice.description).toBe('string');
        if (kind === 'network') expect(notice.description).toBe('network lost');
        expect(ctx.setBusy.mock.calls).toEqual([[true], [false]]);
    });
});
