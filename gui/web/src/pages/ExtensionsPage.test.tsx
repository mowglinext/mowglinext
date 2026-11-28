import {fireEvent, render, screen, waitFor, within} from '@testing-library/react';
import userEvent from '@testing-library/user-event';
import {afterEach, beforeEach, describe, expect, it, vi} from 'vitest';
import ExtensionsPage from './ExtensionsPage';
import type {ExtensionManifest, ExtensionReview, ExtensionServiceSummary, ExtensionState, ExtensionStatus} from './extensions/api';

const manifest: ExtensionManifest = {
    id: 'private-link', version: '1.0.0', source: {repository: 'my-team/private-link', revision: 'a'.repeat(40)},
    runtime: {kind: 'compose'}, profile: 'isolated', grants: ['network:egress'], health: 'process-only',
};
const entry: ExtensionStatus = {
    id: manifest.id, version: manifest.version, manifest, digest: 'installed-digest', managed: true, enabled: false,
    compatible: true, blockers: [], recovery_required: false, observation: {process: 'stopped', application: 'unknown', data: 'unknown'},
};
const state: ExtensionState = {api: 1, configured: true, entries: []};
const serviceSummary: ExtensionServiceSummary[] = [{
    name: 'link', image: `ghcr.io/my-team/link@sha256:${'c'.repeat(64)}`,
    mounts: [
        {type: 'bind', source: '/etc/mowgli-extensions/private-link', target: '/app/config', read_only: true},
        {type: 'volume', source: 'private-link-data', target: '/app/data', read_only: false},
    ],
    networks: [{name: 'private-link-internal', internal: true}, {name: 'private-link-egress', internal: false}],
    environment_keys: ['ACCESS_TOKEN', 'SERVER_URL'],
}];
const reviewed: ExtensionReview = {manifest, digest: 'reviewed-digest', binding: {project: 'extension-private-link', config_dir: '/etc/mowgli-extensions/private-link', data_dir: '/var/lib/mowgli-extensions/private-link/data'}, action: 'install', blockers: [], services: serviceSummary};
const fetchMock = vi.fn<typeof fetch>();
function urlPath(url: RequestInfo | URL) { return typeof url === 'string' ? url : url instanceof URL ? url.href : url.url; }
function bodyJSON(body: BodyInit | null | undefined): unknown { return JSON.parse(typeof body === 'string' ? body : 'null') as unknown; }
function response(value: unknown, status = 200) { return Promise.resolve(new Response(JSON.stringify(value), {status})); }
function setup(initial = state, blockers: string[] = [], reviewOverride: Partial<ExtensionReview> = {}) {
    fetchMock.mockImplementation((url) => {
        if (urlPath(url).endsWith('/state')) return response(initial);
        if (urlPath(url).endsWith('/review')) return response({...reviewed, blockers, ...reviewOverride});
        return response({action: 'install', id: manifest.id, phase: 'running'}, 202);
    });
}
async function importAndReview(uploadedBundle: Record<string, unknown> = {manifest}) {
    const user = userEvent.setup();
    await user.click(await screen.findByRole('button', {name: 'Custom import'}));
    await user.upload(screen.getByLabelText('Bundle file'), new File([JSON.stringify(uploadedBundle)], 'private.json', {type: 'application/json'}));
    await waitFor(() => expect(screen.getByRole('button', {name: 'Review bundle'})).toBeEnabled());
    await user.click(screen.getByRole('button', {name: 'Review bundle'}));
    await screen.findByText('a'.repeat(40));
    await waitFor(() => expect(screen.getByRole('button', {name: /Apply reviewed import/})).not.toHaveClass('ant-btn-loading'));
    return user;
}
function posts(path: string) { return fetchMock.mock.calls.filter(([url, options]) => urlPath(url).endsWith(path) && options?.method === 'POST'); }

describe('ExtensionsPage', () => {
    beforeEach(() => { vi.stubGlobal('fetch', fetchMock); fetchMock.mockReset(); setup(); });
    afterEach(() => { vi.unstubAllGlobals(); });

    it('reviews a local private bundle before applying its exact reviewed identity, without enabling it', async () => {
        render(<ExtensionsPage/>);
        const uploadedBundle = {manifest, compose: {services: {link: {environment: {ACCESS_TOKEN: 'never-render-this-secret'}}}}};
        const user = await importAndReview(uploadedBundle);
        expect(posts('/apply')).toHaveLength(0);
        expect(screen.getByText('network:egress')).toBeInTheDocument();
        expect(screen.getByText(/without starting it/)).toBeInTheDocument();
        const summary = screen.getByRole('region', {name: 'Services in this bundle'});
        expect(within(summary).getByText(serviceSummary[0].image)).toBeInTheDocument();
        expect(within(summary).getByText('/etc/mowgli-extensions/private-link → /app/config')).toBeInTheDocument();
        expect(within(summary).getByText('private-link-data → /app/data')).toBeInTheDocument();
        expect(within(summary).getByText('Host directory · Read only')).toBeInTheDocument();
        expect(within(summary).getByText('Named volume · Read and write')).toBeInTheDocument();
        expect(within(summary).getByText('(Internal only)')).toBeInTheDocument();
        expect(within(summary).getByText('(Egress enabled)')).toBeInTheDocument();
        expect(within(summary).getByText('ACCESS_TOKEN, SERVER_URL')).toBeInTheDocument();
        expect(screen.queryByText(/never-render-this-secret/)).toBeNull();
        await user.click(screen.getByRole('button', {name: /Apply reviewed import/}));
        await waitFor(() => expect(posts('/apply')).toHaveLength(1));
        const options = posts('/apply')[0][1]!;
        expect(bodyJSON(options.body)).toEqual({action: 'install', bundle: JSON.stringify(uploadedBundle), id: manifest.id, digest: 'reviewed-digest', acknowledged: true});
        expect(options.headers).toEqual({'Content-Type': 'application/json', 'X-Mowgli-Update': '1'});
        expect(posts('/action')).toHaveLength(0);
    });

    it.each([{label: 'missing', services: undefined}, {label: 'empty', services: []}])('blocks Compose apply when the host service summary is $label', async ({services}) => {
        setup(state, [], {services});
        render(<ExtensionsPage/>);
        const user = await importAndReview();
        expect(screen.getByText(/host did not provide a service summary/)).toBeInTheDocument();
        const apply = screen.getByRole('button', {name: /Apply reviewed import/});
        expect(apply).toBeDisabled();
        await user.click(apply);
        expect(posts('/apply')).toHaveLength(0);
    });

    it('permits observe-only external registration with an empty Compose service summary', async () => {
        setup(state, [], {manifest: {...manifest, runtime: {kind: 'external-systemd', units: ['link.service']}, profile: 'external'}, services: []});
        render(<ExtensionsPage/>);
        const user = await importAndReview();
        expect(screen.queryByText(/host did not provide a service summary/)).toBeNull();
        expect(screen.getByText(/registers an external service for observation only/)).toBeInTheDocument();
        await user.click(screen.getByRole('button', {name: /Apply reviewed import/}));
        await waitFor(() => expect(posts('/apply')).toHaveLength(1));
    });

    it('shows host trust blockers and prevents applying without silently trusting a custom repository', async () => {
        setup(state, ['source repository is not trusted']);
        render(<ExtensionsPage/>);
        await importAndReview();
        expect(screen.getByText('source repository is not trusted')).toBeInTheDocument();
        expect(screen.getByRole('button', {name: /Apply reviewed import/})).toBeDisabled();
        expect(screen.getByText(/An administrator must approve/)).toBeInTheDocument();
        expect(posts('/apply')).toHaveLength(0);
    });

    it('preserves the selected file and review after an apply error and permits retry', async () => {
        setup();
        const base = fetchMock.getMockImplementation()!;
        let failed = false;
        fetchMock.mockImplementation((url, options) => {
            if (urlPath(url).endsWith('/apply') && !failed) { failed = true; return response({error: 'Host is busy; retry'}, 409); }
            return base(url, options);
        });
        render(<ExtensionsPage/>);
        const user = await importAndReview();
        await user.click(screen.getByRole('button', {name: /Apply reviewed import/}));
        expect(await screen.findByText('Host is busy; retry')).toBeInTheDocument();
        expect(screen.getByText('private.json')).toBeInTheDocument();
        expect(screen.getByText('reviewed-digest')).toBeInTheDocument();
        await waitFor(() => expect(screen.getByRole('button', {name: /Apply reviewed import/})).toBeEnabled());
        await user.click(screen.getByRole('button', {name: /Apply reviewed import/}));
        await waitFor(() => expect(posts('/apply')).toHaveLength(2));
    });

    it('offers only Forget for external services and confirms that the process and data remain untouched', async () => {
        setup({...state, entries: [{...entry, managed: false}]});
        render(<ExtensionsPage/>);
        const row = await screen.findByRole('region', {name: manifest.id});
        expect(within(row).queryByRole('button', {name: 'Enable'})).toBeNull();
        expect(within(row).queryByRole('button', {name: 'Remove'})).toBeNull();
        const user = userEvent.setup();
        await user.click(within(row).getByRole('button', {name: 'Forget registration'}));
        expect(posts('/action')).toHaveLength(0);
        const dialog = screen.getByRole('dialog');
        expect(within(dialog).getByText(/service keeps running/)).toBeInTheDocument();
        await user.click(within(dialog).getByRole('button', {name: 'Confirm'}));
        await waitFor(() => expect(posts('/action')).toHaveLength(1));
        expect(bodyJSON(posts('/action')[0][1]?.body)).toEqual({action: 'forget', id: manifest.id, digest: 'installed-digest', acknowledged: true});
    });

    it('requires confirmation for enable and prevents removal while enabled', async () => {
        setup({...state, entries: [{...entry, enabled: true, previous_version: '0.9.0'}]});
        render(<ExtensionsPage/>);
        expect(await screen.findByRole('button', {name: 'Remove'})).toBeDisabled();
        const user = userEvent.setup();
        await user.click(screen.getByRole('button', {name: 'Disable'}));
        expect(posts('/action')).toHaveLength(0);
        expect(screen.getByText(/registration, configuration and data are retained/)).toBeInTheDocument();
        await user.click(screen.getByRole('button', {name: 'Confirm'}));
        await waitFor(() => expect(posts('/action')).toHaveLength(1));
        expect(bodyJSON(posts('/action')[0][1]?.body)).toMatchObject({action: 'disable', digest: 'installed-digest'});
    });

    it('offers journal recovery after failure and makes unknown health explicit', async () => {
        setup({...state, entries: [entry], operation: {action: 'enable', id: entry.id, phase: 'failed', error: 'Interrupted operation'}});
        render(<ExtensionsPage/>);
        expect(await screen.findByText('Interrupted operation')).toBeInTheDocument();
        expect(screen.getAllByText('Unknown')).toHaveLength(2);
        const user = userEvent.setup();
        await user.click(screen.getByRole('button', {name: 'Recover'}));
        await user.click(screen.getByRole('button', {name: 'Confirm'}));
        await waitFor(() => expect(posts('/action')).toHaveLength(1));
        expect(bodyJSON(posts('/action')[0][1]?.body)).toEqual({action: 'recover', id: '', digest: '', acknowledged: true});
    });

    it('binds a confirmed rollback to the displayed previous bundle identity', async () => {
        setup({...state, entries: [{...entry, previous_version: '0.9.0', previous_digest: 'previous-digest'}]});
        render(<ExtensionsPage/>);
        const user = userEvent.setup();
        await user.click(await screen.findByRole('button', {name: 'Roll back (0.9.0)'}));
        expect(screen.getByText(/previous-digest/)).toBeInTheDocument();
        await user.click(screen.getByRole('button', {name: 'Confirm'}));
        await waitFor(() => expect(posts('/action')).toHaveLength(1));
        expect(bodyJSON(posts('/action')[0][1]?.body)).toEqual({action: 'rollback', id: entry.id, digest: 'installed-digest', previous_digest: 'previous-digest', acknowledged: true});
    });

    it('handles an older host without offering mutation controls', async () => {
        fetchMock.mockImplementation(() => response({error: 'Extension helper unavailable'}, 503));
        render(<ExtensionsPage/>);
        expect(await screen.findByText('Extension Manager unavailable')).toBeInTheDocument();
        expect(screen.getByText(/On an older installation/)).toBeInTheDocument();
        expect(screen.getByRole('button', {name: 'Custom import'})).toBeDisabled();
    });

    it('rejects oversized and malformed local bundles before review', async () => {
        render(<ExtensionsPage/>);
        const user = userEvent.setup();
        await user.click(await screen.findByRole('button', {name: 'Custom import'}));
        fireEvent.change(screen.getByLabelText('Bundle file'), {target: {files: [new File(['x'.repeat(2 * 1024 * 1024 + 1)], 'large.json')]}});
        expect(await screen.findByText(/no larger than 2 MiB/)).toBeInTheDocument();
        fireEvent.change(screen.getByLabelText('Bundle file'), {target: {files: [new File(['not json'], 'invalid.json')]}});
        expect(await screen.findByText(/Could not read this JSON bundle/)).toBeInTheDocument();
        expect(posts('/review')).toHaveLength(0);
    });
});
