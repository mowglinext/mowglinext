import {App} from 'antd';
import {act, fireEvent, render, screen, waitFor, within} from '@testing-library/react';
import {beforeEach, describe, expect, it, vi} from 'vitest';
import {SchedulePage} from './SchedulePage.tsx';

interface ApiCall {
    path: string;
    method: string;
    body?: Record<string, unknown>;
}

const mocks = vi.hoisted(() => ({
    request: vi.fn<(call: ApiCall) => Promise<unknown>>(),
    map: {} as Record<string, unknown>,
}));
vi.mock('../hooks/useApi.ts', () => ({useApi: () => ({request: mocks.request})}));
vi.mock('../hooks/useMowingMap.ts', () => ({useMowingMap: () => mocks.map}));
vi.mock('../hooks/useSettings.ts', () => ({useSettings: () => ({settings: {}})}));
vi.mock('../hooks/useIsMobile', () => ({useIsMobile: () => true}));
vi.mock('../components/schedule/IrriSenseStatusChip.tsx', () => ({IrriSenseStatusChip: () => null}));

interface Stored {
    id: string;
    areaId?: number;
    areaName?: string;
    time: string;
    daysOfWeek: number[];
    enabled: boolean;
    createdAt: string;
}

const stored = (over: Partial<Stored> = {}): Stored => ({
    id: 's1', areaId: 0, time: '09:00', daysOfWeek: [1], enabled: true, createdAt: '2026-10-01T00:00:00Z', ...over,
});

const serve = (schedules: Stored[]) => {
    mocks.request.mockImplementation(({path, method}) => {
        if (path === '/schedules' && method === 'GET') return Promise.resolve({data: {schedules}});
        return Promise.resolve({data: {}});
    });
};

const showPage = async (schedules: Stored[]) => {
    serve(schedules);
    render(<App><SchedulePage/></App>);
    await screen.findByLabelText(/Choose the area for schedule 1/);
};

const putBodies = () => mocks.request.mock.calls.map(([call]) => call).filter(call => call.method === 'PUT');

const pendingDisable = async (outcome: 'success' | 'lost-response' | 'unreadable' | 'malformed-readback' | 'normalized' | 'refused' = 'success') => {
    let current = stored();
    let release!: () => void;
    const pending = new Promise<void>(resolve => { release = resolve; });
    let puts = 0;
    let initialRead = true;
    mocks.request.mockImplementation(async ({method, body}) => {
        if (method === 'PUT') {
            const first = ++puts === 1;
            if (first && outcome === 'refused') {
                await pending;
                throw Object.assign(new Error('overlap refused'), {status: 409});
            }
            current = structuredClone(body) as unknown as Stored;
            if (first && outcome === 'normalized') current.time = '09:15';
            const confirmed = structuredClone(current);
            if (first) {
                await pending;
                if (outcome === 'lost-response' || outcome === 'unreadable' || outcome === 'malformed-readback') throw new Error('response lost');
            }
            return {data: confirmed};
        }
        if (!initialRead && outcome === 'malformed-readback') return {data: {}};
        if (!initialRead && outcome === 'unreadable') throw new Error('readback unavailable');
        initialRead = false;
        return {data: {schedules: [structuredClone(current)]}};
    });
    render(<App><SchedulePage/></App>);
    await screen.findByRole('switch');
    fireEvent.click(screen.getByRole('switch'));
    await waitFor(() => expect(putBodies()).toHaveLength(1));
    return {
        read: () => current,
        release: () => act(async () => { release(); await pending; }),
    };
};

describe('schedule page area picker', () => {
    it('keeps the last area intention queued while an earlier area PUT is pending', async () => {
        const initial = stored();
        let current = structuredClone(initial);
        let release!: () => void;
        const pending = new Promise<void>(resolve => { release = resolve; });
        let puts = 0;
        mocks.request.mockImplementation(async ({method, body}) => {
            if (method === 'PUT') {
                current = structuredClone(body) as unknown as Stored;
                const confirmed = structuredClone(current);
                if (++puts === 1) await pending;
                return {data: confirmed};
            }
            return {data: {schedules: [structuredClone(current)]}};
        });
        render(<App><SchedulePage/></App>);
        fireEvent.click(await screen.findByLabelText(/Choose the area for schedule 1/));
        let dialog = await screen.findByRole('dialog');
        fireEvent.click(within(dialog).getByRole('radio', {name: 'Back garden'}));
        fireEvent.click(within(dialog).getByRole('button', {name: 'Save'}));
        await waitFor(() => expect(putBodies()).toHaveLength(1));
        fireEvent.click(screen.getByLabelText(/Choose the area for schedule 1/));
        dialog = await screen.findByRole('dialog');
        fireEvent.click(within(dialog).getByRole('radio', {name: /All areas/}));
        fireEvent.click(within(dialog).getByRole('button', {name: 'Save'}));
        const countWhilePending = putBodies().length;
        await act(async () => { release(); await pending; });

        await waitFor(() => expect(putBodies()).toHaveLength(2));
        expect(countWhilePending).toBe(1);
        expect(putBodies()[0].body).toEqual({...initial, areaId: 7, areaName: 'Back garden'});
        expect(putBodies()[1].body).toEqual({...initial, areaId: 0, areaName: ''});
        expect(current).toEqual({...initial, areaId: 0, areaName: ''});
        expect(screen.getByRole('switch')).toBeChecked();
        expect(screen.getByLabelText(/Choose the area for schedule 1/)).toHaveTextContent('Applies to all areas');
    });

    it('preserves successive queued day edits and the confirmed disabled state', async () => {
        const server = await pendingDisable();
        fireEvent.click(screen.getByRole('button', {name: 'Wed'}));
        fireEvent.click(screen.getByRole('button', {name: 'Fri'}));
        await server.release();
        await waitFor(() => expect(putBodies()).toHaveLength(3));
        await waitFor(() => expect(screen.getByRole('switch')).not.toBeChecked());
        expect(putBodies()[1].body).toMatchObject({enabled: false, daysOfWeek: [1, 3]});
        expect(putBodies()[2].body).toMatchObject({enabled: false, daysOfWeek: [1, 3, 5]});
        expect(server.read().daysOfWeek).toEqual([1, 3, 5]);
    });

    it('applies two queued toggles of the same day in click order', async () => {
        const server = await pendingDisable();
        fireEvent.click(screen.getByRole('button', {name: 'Wed'}));
        fireEvent.click(screen.getByRole('button', {name: 'Wed'}));
        await server.release();
        await waitFor(() => expect(putBodies()).toHaveLength(3));
        expect(putBodies()[2].body).toMatchObject({enabled: false, daysOfWeek: [1]});
        expect(server.read().daysOfWeek).toEqual([1]);
    });

    it('rebases an edit on readback when a disable was applied but its response was lost', async () => {
        const server = await pendingDisable('lost-response');
        fireEvent.click(screen.getByRole('button', {name: 'Wed'}));
        await server.release();
        await waitFor(() => expect(putBodies()).toHaveLength(2));
        await waitFor(() => expect(screen.getByRole('switch')).not.toBeChecked());
        expect(putBodies()[1].body).toMatchObject({enabled: false, daysOfWeek: [1, 3]});
        expect(server.read().enabled).toBe(false);
        expect(mocks.request.mock.calls.filter(([c]) => c.method === 'GET')).toHaveLength(2);
    });

    it('does not send a stale full update when failed-write readback is unavailable', async () => {
        const server = await pendingDisable('unreadable');
        fireEvent.click(screen.getByRole('button', {name: 'Wed'}));
        await server.release();
        await screen.findByText('Failed to load schedules');
        expect(putBodies()).toHaveLength(1);
        expect(server.read().enabled).toBe(false);
    });

    it('does not treat a malformed readback as confirmation after an uncertain disable', async () => {
        const server = await pendingDisable('malformed-readback');
        fireEvent.click(screen.getByRole('button', {name: 'Wed'}));
        await server.release();
        await screen.findByText('Failed to load schedules');
        expect(putBodies()).toHaveLength(1);
        expect(server.read().enabled).toBe(false);
        expect(screen.getByRole('switch')).toBeChecked();
    });

    it('uses server-normalized fields rather than an earlier render for the queued edit', async () => {
        const server = await pendingDisable('normalized');
        fireEvent.click(screen.getByRole('button', {name: 'Wed'}));
        await server.release();
        await waitFor(() => expect(putBodies()).toHaveLength(2));
        expect(putBodies()[1].body).toMatchObject({enabled: false, time: '09:15', daysOfWeek: [1, 3]});
        expect(server.read().time).toBe('09:15');
    });

    it('keeps a refused disable visible and uses the actual confirmed state for the next edit', async () => {
        const server = await pendingDisable('refused');
        fireEvent.click(screen.getByRole('button', {name: 'Wed'}));
        await server.release();
        await waitFor(() => expect(putBodies()).toHaveLength(2));
        expect(await screen.findByText('Schedules overlap')).toBeInTheDocument();
        expect(screen.getByRole('switch')).toBeChecked();
        expect(putBodies()[1].body).toMatchObject({enabled: true, daysOfWeek: [1, 3]});
        expect(server.read().enabled).toBe(true);
    });

    it('keeps an intentional later enable after the disabled state is confirmed', async () => {
        const server = await pendingDisable();
        await server.release();
        await waitFor(() => expect(screen.getByRole('switch')).not.toBeChecked());
        fireEvent.click(screen.getByRole('switch'));
        await waitFor(() => expect(putBodies()).toHaveLength(2));
        await waitFor(() => expect(screen.getByRole('switch')).toBeChecked());
        expect(putBodies()[1].body).toMatchObject({enabled: true});
        expect(server.read().enabled).toBe(true);
    });
    it('does not reenable a schedule when a day is edited while disabling is pending', async () => {
        let current = stored();
        let release!: () => void;
        const pending = new Promise<void>(resolve => { release = resolve; });
        let puts = 0;
        mocks.request.mockImplementation(async ({method, body}) => {
            if (method === 'PUT') {
                current = structuredClone(body) as unknown as Stored;
                const confirmed = structuredClone(current);
                if (++puts === 1) await pending;
                return {data: confirmed};
            }
            return {data: {schedules: [structuredClone(current)]}};
        });
        render(<App><SchedulePage/></App>);
        await screen.findByRole('switch');
        fireEvent.click(screen.getByRole('switch'));
        await waitFor(() => expect(putBodies()).toHaveLength(1));
        fireEvent.click(screen.getByRole('button', {name: 'Wed'}));
        await act(() => Promise.resolve());
        const countWhilePending = putBodies().length;
        await act(async () => { release(); await pending; });
        await waitFor(() => expect(putBodies()).toHaveLength(2));
        await waitFor(() => expect(screen.getByRole('switch')).not.toBeChecked());
        expect(countWhilePending).toBe(1);
        expect(putBodies()[0].body).toMatchObject({enabled: false});
        expect(putBodies()[1].body).toMatchObject({enabled: false, daysOfWeek: [1, 3]});
        expect(current.enabled).toBe(false);
        expect(current.daysOfWeek).toEqual([1, 3]);
    });
    beforeEach(() => {
        vi.clearAllMocks();
        mocks.map = {
            working_area: [{id: 11, name: 'Front lawn'}, {id: 7, name: 'Back garden'}],
            working_area_indices: [0, 1],
        };
    });

    it('says all areas for a schedule without an area', async () => {
        await showPage([stored()]);
        expect(screen.getByLabelText(/Choose the area for schedule 1/)).toHaveTextContent('Applies to all areas');
    });

    it('shows the live name of the chosen area', async () => {
        await showPage([stored({areaId: 7, areaName: 'old name'})]);
        expect(screen.getByLabelText(/Choose the area for schedule 1/)).toHaveTextContent('Back garden');
    });

    it('flags an area that is gone from the map instead of silently showing all areas', async () => {
        await showPage([stored({areaId: 99, areaName: 'Shed lawn'})]);
        expect(screen.getByLabelText(/Choose the area for schedule 1/)).toHaveTextContent('Shed lawn (removed)');
    });

    it('does not call a known area removed while the map has not loaded yet', async () => {
        mocks.map = {};
        await showPage([stored({areaId: 99, areaName: 'Shed lawn'})]);
        expect(screen.getByLabelText(/Choose the area for schedule 1/)).toHaveTextContent(/^Shed lawn$/);
    });

    it('opens a dialog with all areas and the recorded areas, and saves the pick with its id and name', async () => {
        await showPage([stored()]);
        fireEvent.click(screen.getByLabelText(/Choose the area for schedule 1/));

        const dialog = await screen.findByRole('dialog');
        expect(within(dialog).getByRole('radio', {name: /All areas/})).toBeChecked();
        fireEvent.click(within(dialog).getByRole('radio', {name: 'Back garden'}));
        fireEvent.click(within(dialog).getByRole('button', {name: 'Save'}));

        await waitFor(() => expect(putBodies()).toHaveLength(1));
        expect(putBodies()[0].path).toBe('/schedules/s1');
        expect(putBodies()[0].body).toMatchObject({areaId: 7, areaName: 'Back garden'});
    });

    it('picking all areas clears the id and the name', async () => {
        await showPage([stored({areaId: 11, areaName: 'Front lawn'})]);
        fireEvent.click(screen.getByLabelText(/Choose the area for schedule 1/));

        const dialog = await screen.findByRole('dialog');
        fireEvent.click(within(dialog).getByRole('radio', {name: /All areas/}));
        fireEvent.click(within(dialog).getByRole('button', {name: 'Save'}));

        await waitFor(() => expect(putBodies()).toHaveLength(1));
        expect(putBodies()[0].body).toMatchObject({areaId: 0, areaName: ''});
    });

    it('saving without changing the area sends nothing', async () => {
        await showPage([stored({areaId: 11, areaName: 'Front lawn'})]);
        fireEvent.click(screen.getByLabelText(/Choose the area for schedule 1/));

        const dialog = await screen.findByRole('dialog');
        fireEvent.click(within(dialog).getByRole('button', {name: 'Save'}));

        // Give the (async) save handler a turn: it must decide there is nothing to send.
        await new Promise(resolve => setTimeout(resolve, 50));
        expect(putBodies()).toHaveLength(0);
    });

    it('toggling a per-area schedule on or off keeps its area', async () => {
        await showPage([stored({areaId: 7, areaName: 'Back garden', enabled: true})]);

        fireEvent.click(screen.getByRole('switch'));

        await waitFor(() => expect(putBodies()).toHaveLength(1));
        expect(putBodies()[0].body).toMatchObject({enabled: false, areaId: 7, areaName: 'Back garden'});
    });

    it('changing the time or a day of a per-area schedule keeps its area', async () => {
        await showPage([stored({areaId: 11, areaName: 'Front lawn', daysOfWeek: [1, 2]})]);

        fireEvent.click(screen.getByRole('button', {name: 'Wed'}));

        await waitFor(() => expect(putBodies()).toHaveLength(1));
        expect(putBodies()[0].body).toMatchObject({areaId: 11, areaName: 'Front lawn', daysOfWeek: [1, 2, 3]});
    });

    it('explains an overlap rejection from the server', async () => {
        await showPage([stored({enabled: false})]);
        mocks.request.mockImplementation(({method}) => {
            if (method === 'PUT') {
                return Promise.reject(Object.assign(
                    new Error('overlaps the enabled schedule starting at 09:30'), {status: 409}));
            }
            return Promise.resolve({data: {schedules: [stored({enabled: false})]}});
        });

        fireEvent.click(screen.getByRole('switch'));

        expect(await screen.findByText('Schedules overlap')).toBeInTheDocument();
        expect(await screen.findByText(/overlaps the enabled schedule starting at 09:30/)).toBeInTheDocument();
    });
});
