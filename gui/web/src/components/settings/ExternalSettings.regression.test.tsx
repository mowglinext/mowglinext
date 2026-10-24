import {App} from 'antd';
import {act, fireEvent, render, screen, waitFor} from '@testing-library/react';
import {beforeEach, describe, expect, it, vi} from 'vitest';
import {NotificationsSection} from './NotificationsSection';
import {RemoteAccessSection} from './RemoteAccessSection';
import {IrriSenseSection} from './IrriSenseSection';
import {NOTIFICATION_EVENT_KINDS} from '../../types/notifications';
import type {ExternalSaver} from '../../hooks/useSettingsManager';

interface Request {path: string; method: string; body?: Record<string, unknown>}
const mocks = vi.hoisted(() => ({request: vi.fn<(request: Request) => Promise<unknown>>(), refresh: vi.fn()}));
vi.mock('../../hooks/useApi.ts', () => {
    const api = {request: mocks.request};
    return {useApi: () => api};
});
vi.mock('../../hooks/useRemoteAccessStatus.ts', () => ({
    useRemoteAccessStatus: () => ({status: null, error: null, refresh: mocks.refresh}),
}));
vi.mock('../../hooks/useSoilStatus.ts', () => ({
    useSoilStatus: () => ({status: null, refresh: mocks.refresh}),
}));

const sections = [
    {name: 'notifications', Component: NotificationsSection, label: /Notifications/i, fields: {
        channel: 'telegram', language: 'en', title: 'MowgliNext', ntfyServer: 'https://ntfy.sh', ntfyTopic: '',
        telegramChatId: '', ntfyTokenSet: false, ntfyTokenMasked: '', telegramBotTokenSet: false,
        telegramBotTokenMasked: '', pushoverAppTokenSet: false, pushoverAppTokenMasked: '',
        pushoverUserKey: '', webhookUrl: '', events: Object.fromEntries(NOTIFICATION_EVENT_KINDS.map(k => [k, true])),
        eventKinds: [...NOTIFICATION_EVENT_KINDS], channels: ['telegram', 'ntfy', 'webhook'],
    }},
    {name: 'remote-access', Component: RemoteAccessSection, label: /Remote access/i, fields: {
        hostname: 'mowgli', authKeySet: false, authKeyMasked: '', serveHttps: true,
        image: 'tailscale/tailscale:v1.102.3', defaultImage: 'tailscale/tailscale:v1.102.3', containerName: 'mowgli-remote',
    }},
    {name: 'irrisense', Component: IrriSenseSection, label: /IrriSense/i, fields: {
        baseUrl: 'https://example.invalid', tokenSet: false, tokenMasked: '', gardenId: '', zoneIds: [],
        wetDeficitMm: 2, dryAfterWateringHours: 3, maxStaleMinutes: 90, gateScheduler: true,
    }},
] as const;
const puts = () => mocks.request.mock.calls.map(([r]) => r).filter(r => r.method === 'PUT');

for (const section of sections) {
    describe(section.name + ' partial save reconciliation', () => {
        beforeEach(() => vi.clearAllMocks());
        const begin = async (outcome: 'ok' | 'refused' | 'lost-response' = 'ok') => {
            let stored: Record<string, unknown> = {enabled: false, ...section.fields};
            let release!: () => void;
            const gate = new Promise<void>(resolve => {release = resolve;});
            let count = 0;
            let saver: ExternalSaver | undefined;
            const register = (_id: string, value: ExternalSaver) => {saver = value;};
            mocks.request.mockImplementation(async ({method, path, body}) => {
                if (method === 'PUT') {
                    const first = ++count === 1;
                    if (first) await gate;
                    if (first && outcome === 'refused') throw new Error('refused by server');
                    stored = {...stored, ...body};
                    if (first && outcome === 'lost-response') throw new Error('response lost');
                    return {data: structuredClone(stored)};
                }
                if (path.endsWith('/settings')) return {data: structuredClone(stored)};
                return {data: {enabled: false, configured: false, sentCount: 0, failedCount: 0}};
            });
            const Component = section.Component;
            const view = render(<App><Component registerSaver={register}/></App>);
            const toggle = await screen.findByRole('switch', {name: section.label});
            fireEvent.click(toggle);
            await waitFor(() => expect(saver?.dirtyCount).toBe(1));
            let pending!: Promise<boolean>;
            act(() => {pending = saver!.save();});
            await waitFor(() => expect(puts()).toHaveLength(1));
            expect(puts()[0].body).toEqual({enabled: true});
            return {
                view, toggle, read: () => stored, saver: () => saver!,
                save: async () => {let ok = false; await act(async () => {ok = await saver!.save();}); return ok;},
                finish: async () => {let ok = false; await act(async () => {release(); ok = await pending;}); return ok;},
            };
        };

        it('preserves a newer draft and saves the last intention on the next save', async () => {
            const h = await begin();
            fireEvent.click(h.toggle);
            expect(h.toggle).toHaveAttribute('aria-checked', 'false');
            expect(await h.finish()).toBe(true);
            expect(h.toggle).toHaveAttribute('aria-checked', 'false');
            expect(h.saver().dirtyCount).toBe(1);
            expect(h.read().enabled).toBe(true);
            expect(await h.save()).toBe(true);
            expect(puts()[1].body).toEqual({enabled: false});
            expect(h.read().enabled).toBe(false);
            expect(h.saver().dirtyCount).toBe(0);
        });

        it('acknowledges all submitted edits when no newer edit exists', async () => {
            const h = await begin();
            expect(await h.finish()).toBe(true);
            expect(h.saver().dirtyCount).toBe(0);
            expect(h.toggle).toHaveAttribute('aria-checked', 'true');
            expect(await h.save()).toBe(true);
            expect(puts()).toHaveLength(1);
        });

        it('keeps rejected edits dirty and permits an explicit retry', async () => {
            const h = await begin('refused');
            expect(await h.finish()).toBe(false);
            expect(h.saver().dirtyCount).toBe(1);
            expect(h.read().enabled).toBe(false);
            expect(await screen.findByText('refused by server')).toBeInTheDocument();
            expect(await h.save()).toBe(true);
            expect(h.read().enabled).toBe(true);
            expect(h.saver().dirtyCount).toBe(0);
            expect(puts()).toHaveLength(2);
        });

        it('orders a second save during the first PUT using the latest draft', async () => {
            const h = await begin();
            fireEvent.click(h.toggle);
            let second!: Promise<boolean>;
            act(() => {second = h.saver().save();});
            expect(puts()).toHaveLength(1);
            expect(await h.finish()).toBe(true);
            await act(async () => {expect(await second).toBe(true);});
            expect(puts()).toHaveLength(2);
            expect(puts()[1].body).toEqual({enabled: false});
            expect(h.read().enabled).toBe(false);
            expect(h.saver().dirtyCount).toBe(0);
        });

        it('does not duplicate an unchanged save requested while the first is pending', async () => {
            const h = await begin();
            let second!: Promise<boolean>;
            act(() => {second = h.saver().save();});
            expect(await h.finish()).toBe(true);
            await act(async () => {expect(await second).toBe(true);});
            expect(puts()).toHaveLength(1);
            expect(h.saver().dirtyCount).toBe(0);
        });

        it('does not acknowledge an edit made after a queued Save was requested', async () => {
            const h = await begin();
            fireEvent.click(h.toggle); // Save #2 explicitly submits false.
            let second!: Promise<boolean>;
            act(() => {second = h.saver().save();});
            fireEvent.click(h.toggle); // A later true is not part of either Save.
            expect(await h.finish()).toBe(true);
            await act(async () => {expect(await second).toBe(true);});
            expect(puts()[1].body).toEqual({enabled: false});
            expect(h.read().enabled).toBe(false);
            expect(h.toggle).toHaveAttribute('aria-checked', 'true');
            expect(h.saver().dirtyCount).toBe(1);
            expect(await h.save()).toBe(true);
            expect(h.read().enabled).toBe(true);
            expect(h.saver().dirtyCount).toBe(0);
        });

        it('preserves an edit after an applied write loses its response', async () => {
            const h = await begin('lost-response');
            fireEvent.click(h.toggle);
            expect(await h.finish()).toBe(false);
            expect(h.read().enabled).toBe(true);
            expect(h.toggle).toHaveAttribute('aria-checked', 'false');
            expect(h.saver().dirtyCount).toBe(1);
            expect(await h.save()).toBe(true);
            expect(h.read().enabled).toBe(false);
            expect(h.saver().dirtyCount).toBe(0);
        });

        it('reverts newer unsaved edits without pretending to cancel an in-flight write', async () => {
            const h = await begin();
            fireEvent.click(h.toggle);
            act(() => h.saver().revert?.());
            expect(h.saver().dirtyCount).toBe(0);
            expect(await h.finish()).toBe(true);
            expect(h.read().enabled).toBe(true);
            expect(h.toggle).toHaveAttribute('aria-checked', 'true');
            expect(h.saver().dirtyCount).toBe(0);
            expect(puts()).toHaveLength(1);
        });
    });
}
