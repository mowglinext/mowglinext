import {cleanup, fireEvent, render, screen, waitFor} from '@testing-library/react';
import {afterEach, describe, expect, it, vi} from 'vitest';
import {PullRequestPicker, type UpdaterPullRequest} from './PullRequestPicker';

const pr = (over: Partial<UpdaterPullRequest>): UpdaterPullRequest => ({
    number: 1, title: 'A fix', branch: 'fix/a', author: 'dev', url: 'https://github.com/x/y/pull/1',
    updated_at: '2026-11-09T18:00:00Z', from_fork: false, labeled: false, installable: false, ...over,
});

const listing = {
    repository: 'mowglinext/mowglinext',
    label: 'deploy-preview',
    pull_requests: [
        pr({number: 1110, title: 'No transit next to the robot', branch: 'fix/near-start', labeled: true, installable: true, deployment: 'deployment-aaaaaaaaaaaa-1-1'}),
        pr({number: 1112, title: 'Blade spin-up', branch: 'fix/blade', labeled: false}),
        pr({number: 1200, title: 'From a fork', branch: 'patch-1', from_fork: true}),
    ],
};

const mockFetch = (status: number, body: unknown) =>
    vi.spyOn(globalThis, 'fetch').mockResolvedValue(new Response(JSON.stringify(body), {status}));

const openPicker = () =>
    fireEvent.mouseDown(screen.getByTestId('pull-request-picker').querySelector('.ant-select-selector') ?? screen.getByTestId('pull-request-picker'));

describe('PullRequestPicker', () => {
    afterEach(() => {cleanup(); vi.restoreAllMocks();});

    it('queries the trusted repository and selects an installable PR by its head branch', async () => {
        const fetchSpy = mockFetch(200, listing);
        const onSelect = vi.fn();
        render(<PullRequestPicker repository="mowglinext/mowglinext" branch="" onSelect={onSelect}/>);

        await waitFor(() => expect(fetchSpy).toHaveBeenCalled());
        expect(fetchSpy.mock.calls[0][0]).toBe('/api/system/updater/pull-requests?repository=mowglinext%2Fmowglinext');

        openPicker();
        fireEvent.click(await screen.findByText('#1110 No transit next to the robot', {exact: false}));
        expect(onSelect).toHaveBeenCalledWith('fix/near-start');
    });

    it('explains why a PR is not installable', async () => {
        mockFetch(200, listing);
        render(<PullRequestPicker repository="mowglinext/mowglinext" branch="" onSelect={vi.fn()}/>);
        openPicker();
        expect(await screen.findByText('not built: add label deploy-preview')).toBeInTheDocument();
        expect(screen.getByText('fork: not installable')).toBeInTheDocument();
        expect(screen.getByText('installable')).toBeInTheDocument();
    });

    it('shows the deployment of the PR matching the current branch', async () => {
        mockFetch(200, listing);
        render(<PullRequestPicker repository="mowglinext/mowglinext" branch="fix/near-start" onSelect={vi.fn()}/>);
        expect(await screen.findByText('#1110', {selector: 'a'})).toHaveAttribute('href', 'https://github.com/x/y/pull/1');
        expect(screen.getByText('deployment-aaaaaaaaaaaa-1-1', {exact: false})).toBeInTheDocument();
    });

    it('reports a GitHub failure without breaking the branch field', async () => {
        mockFetch(502, {error: 'GitHub pull requests unavailable: rate limited'});
        render(<PullRequestPicker repository="mowglinext/mowglinext" branch="" onSelect={vi.fn()}/>);
        expect(await screen.findByText('Pull requests unavailable')).toBeInTheDocument();
        expect(screen.getByText('GitHub pull requests unavailable: rate limited')).toBeInTheDocument();
    });
});
