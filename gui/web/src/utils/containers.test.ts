import {describe, expect, it, vi} from 'vitest';
import {Api} from '../api/Api';
import {containerAction, restartGui, restartRos2, restartMowgliStack} from './containers';

const urlText = (url: RequestInfo | URL) => typeof url === 'string' ? url : url instanceof URL ? url.href : url.url;
const json = (value: unknown, status = 200) => new Response(JSON.stringify(value), {status});
function harness(containers: unknown) {
    const fetch = vi.fn<typeof globalThis.fetch>().mockImplementation((_url, init) => Promise.resolve(json(init?.method === 'GET' ? {containers} : {})));
    const api = new Api<unknown>({customFetch: fetch});
    api.baseUrl = "";
    return {api, fetch};
}
const ros = {id: 'ros', names: ['/mowgli-ros2'], labels: {app: 'ros2'}};
const gui = {id: 'gui', names: ['/mowgli-gui']};
const posts = (fetch: ReturnType<typeof harness>['fetch']) => fetch.mock.calls.filter(([, init]) => init?.method === 'POST').map(([url]) => urlText(url));

describe('container command boundaries', () => {
    it('matches names or own labels, preserving optional metadata and command choice', async () => {
        const {api, fetch} = harness([{id: null, names: null, labels: null}, ros]);
        await containerAction(api, {label: {key: 'app', value: 'ros2'}}, 'stop');
        expect(posts(fetch)).toEqual(['/containers/ros/stop']);
    });
    it.each([null, {}, {containers: 1}, {containers: [null]}, {containers: [{id: 0}]},
        {containers: [{names: [3]}]}, {containers: [{labels: {app: false}}]}].map(value => [value]))('rejects malformed metadata before sending commands: %j', async raw => {
        const {api, fetch} = harness([]); fetch.mockResolvedValue(json(raw));
        await expect(containerAction(api, {name: 'ros2'}, 'restart')).rejects.toThrow();
        expect(posts(fetch)).toEqual([]);
    });
    it.each([{containers: null}, {containers: []}])('distinguishes absent containers from list failure: %j', async ({containers}) => {
        const {api, fetch} = harness(containers);
        await expect(containerAction(api, {name: 'ros2'}, 'restart')).rejects.toThrow('Container not found');
        expect(posts(fetch)).toEqual([]);
        fetch.mockResolvedValue(json({error: 'Docker unavailable'}, 500));
        await expect(containerAction(api, {name: 'ros2'}, 'restart')).rejects.toMatchObject({status: 500, error: {error: 'Docker unavailable'}});
    });
    it('does not match inherited labels', async () => {
        const {api, fetch} = harness([{...ros, labels: {}}]);
        await expect(containerAction(api, {label: {key: 'toString', value: 'x'}}, 'restart')).rejects.toThrow('Container not found');
        expect(posts(fetch)).toEqual([]);
    });
    it.each([null, [], {ok: false}, {error: 'rejected'}].map(value => [value]))('does not accept a malformed command acknowledgement: %j', async acknowledgement => {
        const {api, fetch} = harness([ros]); fetch.mockResolvedValueOnce(json({containers: [ros]})).mockResolvedValueOnce(json(acknowledgement));
        await expect(containerAction(api, {name: 'ros2'}, 'restart')).rejects.toThrow('Invalid command acknowledgement');
        expect(posts(fetch)).toHaveLength(1);
    });
    it('restarts ROS first and only the exact hardware sidecar afterward', async () => {
        const {api, fetch} = harness([ros, {id: 'bridge', names: ['/mowgli-openmower']}, {id: 'other', names: ['/mowgli-openmower-test']}]);
        await restartRos2(api);
        expect(posts(fetch)).toEqual(['/containers/ros/restart', '/containers/bridge/restart']);
    });
    it('keeps standalone GUI rejection visible without retry', async () => {
        const {api, fetch} = harness([gui]); fetch.mockResolvedValueOnce(json({containers: [gui]})).mockRejectedValueOnce(new TypeError('connection lost'));
        await expect(restartGui(api)).rejects.toThrow('connection lost'); expect(posts(fetch)).toHaveLength(1);
    });
    it('waits for non-GUI acknowledgements, then dispatches GUI without awaiting its shutdown', async () => {
        const {api, fetch} = harness([gui, ros]);
        let complete: ((response: Response) => void) | undefined;
        fetch.mockImplementation((url, init) => {
            if (init?.method === 'GET') return Promise.resolve(json({containers: [gui, ros]}));
            if (urlText(url).includes('/ros/')) return new Promise(resolve => {complete = resolve;});
            return new Promise(() => {});
        });
        const operation = restartMowgliStack(api); await vi.waitFor(() => expect(complete).toBeDefined());
        expect(posts(fetch)).toEqual(['/containers/ros/restart']);
        complete?.(json({})); await operation;
        expect(posts(fetch)).toEqual(['/containers/ros/restart', '/containers/gui/restart']);
    });
    it('does not restart GUI when another container rejects', async () => {
        const {api, fetch} = harness([gui, ros]); fetch.mockResolvedValueOnce(json({containers: [gui, ros]})).mockResolvedValueOnce(json({error: 'denied'}, 500));
        await expect(restartMowgliStack(api)).rejects.toMatchObject({status: 500});
        expect(posts(fetch)).toEqual(['/containers/ros/restart']);
    });
    it.each([new TypeError('lost'), {status: 500, error: {error: 'denied'}}])('observes detached GUI rejection without retry or fictitious success', async error => {
        const {api, fetch} = harness([gui]); fetch.mockResolvedValueOnce(json({containers: [gui]})).mockRejectedValueOnce(error);
        const rejected = vi.fn(); await restartMowgliStack(api, rejected);
        await vi.waitFor(() => expect(rejected).toHaveBeenCalledWith(error)); expect(posts(fetch)).toHaveLength(1);
    });
});
