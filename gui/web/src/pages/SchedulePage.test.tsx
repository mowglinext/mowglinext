import {App} from 'antd';
import {fireEvent, render, screen, waitFor, within} from '@testing-library/react';
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

describe('schedule page area picker', () => {
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
