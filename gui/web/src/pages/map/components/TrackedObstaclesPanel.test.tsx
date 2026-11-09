import {act, cleanup, fireEvent, render, screen, waitFor, within} from '@testing-library/react';
import {afterEach, beforeEach, describe, expect, it, vi} from 'vitest';
import type {ComponentProps} from 'react';
import {App, ConfigProvider, type ModalFuncProps} from 'antd';
import {TrackedObstaclesPanel} from './TrackedObstaclesPanel.tsx';

type Confirmation = Pick<ModalFuncProps, 'content'> & {onOk: () => Promise<void>};
const mocks = vi.hoisted(() => ({
    realModal: false,
    confirm: vi.fn<(options: Confirmation) => Promise<boolean>>(),
    success: vi.fn<(options: {message: string}) => void>(),
    error: vi.fn<(options: {message: string; description?: string}) => void>(),
    warning: vi.fn(),
    callCreate: vi.fn<(...args: unknown[]) => Promise<unknown>>(),
}));
vi.mock('../../../hooks/useApi.ts', () => ({useApi: () => ({mowglinext: {callCreate: mocks.callCreate}})}));
vi.mock('../../../theme/ThemeContext.tsx', () => ({useThemeMode: () => ({colors: {}})}));
vi.mock('antd', async original => {
    const actual = await original<typeof import('antd')>();
    return {
        ...actual,
        App: Object.assign((props: ComponentProps<typeof actual.App>) => <actual.App {...props}/>, {
            useApp: () => {
                const app = actual.App.useApp();
                return {...app, modal: mocks.realModal ? app.modal : {confirm: mocks.confirm},
                    notification: {success: mocks.success, error: mocks.error, warning: mocks.warning}};
            },
        }),
    };
});

const showPanel = (props: Partial<ComponentProps<typeof TrackedObstaclesPanel>> = {}) => render(<TrackedObstaclesPanel
    obstacles={[{id: 42}]} obstacleAreaIndex={{42: null}} areaNames={{}}
    selectedObstacleId={null} onHoverObstacle={() => {}} {...props}
/>);
const polygon = {points: [{x: 1, y: 2, z: 0}, {x: 3, y: 2, z: 0}, {x: 1, y: 4, z: 0}]};
const inArea = {obstacles: [{id: 42, polygon}], obstacleAreaIndex: {42: 3}, areaNames: {3: 'Front lawn'}};
beforeEach(() => {
    vi.clearAllMocks(); mocks.realModal = false;
    mocks.confirm.mockResolvedValue(true);
    mocks.callCreate.mockReset().mockResolvedValue({data: {message: 'removed'}});
});
afterEach(cleanup);

describe('tracked obstacle ignore', () => {
    it('allows ignoring outside an area and waits for confirmation', async () => {
        showPanel();
        expect(screen.getByRole('button', {name: 'Promote'})).toBeDisabled();
        fireEvent.click(screen.getByRole('button', {name: 'Ignore'}));
        expect(mocks.callCreate).not.toHaveBeenCalled();
        expect(mocks.confirm.mock.calls[0][0].content).toContain('may reappear');
        await mocks.confirm.mock.calls[0][0].onOk();
        expect(mocks.callCreate).toHaveBeenCalledExactlyOnceWith('ignore_obstacle', {obstacle_id: 42});
        expect(mocks.success).toHaveBeenCalledOnce();
    });
    it('shows backend failures and leaves the dialog retryable', async () => {
        mocks.callCreate.mockRejectedValue({error: {error: 'Obstacle 42 not found'}});
        showPanel();
        fireEvent.click(screen.getByRole('button', {name: 'Ignore'}));
        await expect(mocks.confirm.mock.calls[0][0].onOk()).rejects.toEqual({error: {error: 'Obstacle 42 not found'}});
        expect(mocks.error).toHaveBeenCalledWith(expect.objectContaining({description: 'Obstacle 42 not found'}));
        expect(mocks.success).not.toHaveBeenCalled();
    });
});

describe('tracked obstacle promotion', () => {
    it('waits for confirmation and preserves the area, id and polygon on success', async () => {
        showPanel(inArea);
        fireEvent.click(screen.getByRole('button', {name: 'Promote'}));
        expect(mocks.callCreate).not.toHaveBeenCalled();
        await mocks.confirm.mock.calls[0][0].onOk();
        expect(mocks.callCreate).toHaveBeenCalledExactlyOnceWith('promote_obstacle', {area_index: 3, obstacle_id: 42, polygon});
        expect(mocks.success).toHaveBeenCalledOnce();
        expect(mocks.success.mock.calls[0][0].message).toContain('Front lawn');
        expect(mocks.error).not.toHaveBeenCalled();
    });
    it('keeps the existing empty polygon fallback', async () => {
        showPanel({obstacleAreaIndex: {42: 3}});
        fireEvent.click(screen.getByRole('button', {name: 'Promote'}));
        await mocks.confirm.mock.calls[0][0].onOk();
        expect(mocks.callCreate).toHaveBeenCalledExactlyOnceWith('promote_obstacle', {area_index: 3, obstacle_id: 42, polygon: {points: []}});
    });
    it('disables promotion outside any area without opening a confirmation', () => {
        showPanel();
        fireEvent.click(screen.getByRole('button', {name: 'Promote'}));
        expect(mocks.confirm).not.toHaveBeenCalled();
        expect(mocks.callCreate).not.toHaveBeenCalled();
    });
    it('rechecks the area at click time before sending any command', () => {
        const areas: Record<number, number | null> = {42: 3};
        showPanel({...inArea, obstacleAreaIndex: areas});
        areas[42] = null;
        fireEvent.click(screen.getByRole('button', {name: 'Promote'}));
        expect(mocks.warning).toHaveBeenCalledOnce();
        expect(mocks.confirm).not.toHaveBeenCalled();
        expect(mocks.callCreate).not.toHaveBeenCalled();
    });
    it.each([
        {kind: 'response error', response: {error: {error: 'Promotion refused'}}, rejection: undefined, detail: 'Promotion refused'},
        {kind: 'API rejection', response: undefined, rejection: {error: {error: 'Backend unavailable'}}, detail: 'Backend unavailable'},
        {kind: 'network failure', response: undefined, rejection: new TypeError('Failed to fetch'), detail: 'Failed to fetch'},
        {kind: 'unstructured rejection', response: undefined, rejection: null, detail: 'Failed to promote obstacle'},
    ])('rejects $kind with the backend detail and no success', async ({response, rejection, detail}) => {
        if (rejection !== undefined) mocks.callCreate.mockRejectedValue(rejection);
        else mocks.callCreate.mockResolvedValue(response);
        showPanel(inArea);
        fireEvent.click(screen.getByRole('button', {name: 'Promote'}));
        const onOk = mocks.confirm.mock.calls[0][0].onOk;
        if (rejection !== undefined) await expect(onOk()).rejects.toBe(rejection);
        else await expect(onOk()).rejects.toThrow(detail);
        expect(mocks.error).toHaveBeenCalledOnce();
        expect(mocks.error.mock.calls[0][0].description).toBe(detail);
        expect(mocks.success).not.toHaveBeenCalled();
    });
});

describe('real Ant Design confirmation', () => {
    it.each(['Promote', 'Ignore'])('keeps %s open on rejection and permits retry without an unhandled rejection', async action => {
        mocks.realModal = true;
        const failure = {error: {error: 'Temporarily unavailable'}};
        mocks.callCreate.mockRejectedValueOnce(failure);
        const unhandled: unknown[] = [];
        const browserRejection = (event: PromiseRejectionEvent) => {unhandled.push(event.reason);};
        window.addEventListener('unhandledrejection', browserRejection);
        try {
            render(<ConfigProvider theme={{token: {motion: false}}}><App>
                <TrackedObstaclesPanel {...inArea} selectedObstacleId={null} onHoverObstacle={() => {}}/>
            </App></ConfigProvider>);
            fireEvent.click(screen.getByRole('button', {name: action}));
            const dialog = await screen.findByRole('dialog');
            await waitFor(() => expect(dialog).toBeVisible());
            const confirm = within(dialog).getByRole('button', {name: action});
            fireEvent.click(confirm);
            await waitFor(() => expect(mocks.error).toHaveBeenCalledOnce());
            expect(mocks.error.mock.calls[0][0].description).toBe('Temporarily unavailable');
            expect(mocks.success).not.toHaveBeenCalled();
            await waitFor(() => expect(confirm).not.toHaveClass('ant-btn-loading'));
            await waitFor(() => expect(dialog).toBeVisible());
            expect(confirm).toBeEnabled();
            // Flush the event-loop turn in which a detached rejection would surface.
            await act(() => new Promise<void>(resolve => {window.setTimeout(resolve, 0);}));
            expect(unhandled).toEqual([]);
            fireEvent.click(confirm);
            await waitFor(() => expect(mocks.success).toHaveBeenCalledOnce());
            await waitFor(() => expect(screen.queryByRole('dialog')).not.toBeInTheDocument());
            expect(mocks.callCreate).toHaveBeenCalledTimes(2);
            const expected = action === 'Promote' ? ['promote_obstacle', {area_index: 3, obstacle_id: 42, polygon}]
                : ['ignore_obstacle', {obstacle_id: 42}];
            expect(mocks.callCreate.mock.calls).toEqual([expected, expected]);
            await act(() => new Promise<void>(resolve => {window.setTimeout(resolve, 0);}));
            expect(unhandled).toEqual([]);
        } finally {
            window.removeEventListener('unhandledrejection', browserRejection);
        }
    });
});

it.each(['Promote', 'Ignore'])('cancels the real %s confirmation without sending a command', async action => {
    mocks.realModal = true;
    render(<ConfigProvider theme={{token: {motion: false}}}><App>
        <TrackedObstaclesPanel {...inArea} selectedObstacleId={null} onHoverObstacle={() => {}}/>
    </App></ConfigProvider>);
    fireEvent.click(screen.getByRole('button', {name: action}));
    const dialog = await screen.findByRole('dialog');
    await waitFor(() => expect(dialog).toBeVisible());
    fireEvent.click(within(dialog).getByRole('button', {name: 'Cancel'}));
    await waitFor(() => expect(screen.queryByRole('dialog')).not.toBeInTheDocument());
    expect(mocks.callCreate).not.toHaveBeenCalled();
    expect(mocks.error).not.toHaveBeenCalled();
    expect(mocks.success).not.toHaveBeenCalled();
});
