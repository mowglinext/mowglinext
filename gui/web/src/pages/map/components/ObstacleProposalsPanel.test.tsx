import {fireEvent, render, screen} from '@testing-library/react';
import {beforeEach, describe, expect, it, vi} from 'vitest';
import {ObstacleProposalsPanel} from './ObstacleProposalsPanel.tsx';
import type {ObstacleProposal} from '../utils/obstacleProposals.ts';

const mocks = vi.hoisted(() => ({confirm: vi.fn<(options: {content: string; onOk: () => Promise<void>}) => void>(), success: vi.fn(), error: vi.fn(), callCreate: vi.fn()}));
vi.mock('../../../hooks/useApi.ts', () => ({useApi: () => ({mowglinext: {callCreate: mocks.callCreate}})}));
vi.mock('../../../theme/ThemeContext.tsx', () => ({useThemeMode: () => ({colors: {}})}));
vi.mock('antd', async (original) => ({
    ...await original<typeof import('antd')>(),
    App: {useApp: () => ({modal: {confirm: mocks.confirm}, notification: {success: mocks.success, error: mocks.error}})},
}));

const dig: ObstacleProposal = {
    id: 7,
    name: 'Dig at (-3.23, 11.01): wheels 0.45 m vs pose 0.02 m, sigma 0.004 m',
    source: 2,
    areaIndex: 1,
    areaName: 'back lawn',
    polygon: {points: []},
};

const showPanel = (proposals: ObstacleProposal[] = [dig]) => render(<ObstacleProposalsPanel
    proposals={proposals} selectedProposalId={null} onHoverProposal={() => {}}
/>);

describe('obstacle proposals panel', () => {
    beforeEach(() => {
        vi.clearAllMocks();
        mocks.callCreate.mockResolvedValue({data: {message: 'ok'}});
    });

    it('renders nothing without proposals', () => {
        const {container} = showPanel([]);
        expect(container).toBeEmptyDOMElement();
    });

    it('shows the dig evidence and its area so the operator can judge it', () => {
        showPanel();
        expect(screen.getByText('Dig proposal #7')).toBeInTheDocument();
        expect(screen.getByText(/back lawn · Dig at \(-3\.23, 11\.01\)/)).toBeInTheDocument();
    });

    it('accepts through promote_obstacle with the pending id, only after confirmation', async () => {
        showPanel();
        fireEvent.click(screen.getByRole('button', {name: 'Accept'}));
        expect(mocks.callCreate).not.toHaveBeenCalled();
        expect(mocks.confirm.mock.calls[0][0].content).toContain('back lawn');

        await mocks.confirm.mock.calls[0][0].onOk();

        expect(mocks.callCreate).toHaveBeenCalledExactlyOnceWith('promote_obstacle',
            expect.objectContaining({area_index: 1, pending_id: 7, polygon: {points: []}}));
        expect(mocks.success).toHaveBeenCalledOnce();
    });

    it('rejects through discard_obstacle, never the tracker ignore route', async () => {
        showPanel();
        fireEvent.click(screen.getByRole('button', {name: 'Reject'}));
        await mocks.confirm.mock.calls[0][0].onOk();

        expect(mocks.callCreate).toHaveBeenCalledExactlyOnceWith('discard_obstacle', {obstacle_id: 7});
        expect(mocks.success).toHaveBeenCalledOnce();
    });

    it('shows backend failures and leaves the dialog retryable', async () => {
        mocks.callCreate.mockRejectedValue({error: {error: 'no pending obstacle with id 7'}});
        showPanel();
        fireEvent.click(screen.getByRole('button', {name: 'Accept'}));

        await expect(mocks.confirm.mock.calls[0][0].onOk()).rejects.toEqual({error: {error: 'no pending obstacle with id 7'}});

        expect(mocks.error).toHaveBeenCalledWith(expect.objectContaining({description: 'no pending obstacle with id 7'}));
        expect(mocks.success).not.toHaveBeenCalled();
    });
});
