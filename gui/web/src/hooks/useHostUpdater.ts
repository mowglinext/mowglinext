import {useEffect, useState} from 'react';

export interface UpdateSource {repository: string; track: 'stable' | 'dev' | 'custom'; branch: string}
export interface UpdatePolicy {source: UpdateSource; interval_hours: number; pinned: boolean}
export interface Deployment {component_compatibility?: Record<string, string>; service_choices?: {service: string; image: string; when?: Record<string,string>}[]; images?: Record<string,{repository:string; platforms:Record<string,{manifest:string}>}>; gui_compatibility?: string; layout?: number; data_schema?: number; updater_api?: number; maintenance_api?: number; firmware_protocol?: number; release_tag?: string; id: string; source: UpdateSource; revision: string; published_at: string; updater: Record<string, {version: string; build_id?: string}>}
export interface CustomImage {repository?:string; release_tag?:string; deployment_id?:string; requested:string; reference:string; image_id:string; version?:string; revision?:string; built_at?:string}
export interface FirmwareProtocolChange {from: number; to: number}
export interface HealthIssue {service: string; check: string; message: string}
export interface UpdatePlan {custom_images?:Record<string,CustomImage>; stack?: {changes: {service: string; action: string}[]; selection: {options: Record<string, string>}}; overrides?: Record<string, Deployment>; firmware_protocol_change?: FirmwareProtocolChange; preexisting_health_issues?: HealthIssue[]; id: string; target: Deployment; images: Record<string, string>; previous: Record<string, string>; expires_at: string}
export interface UpdateJob {id: string; kind: string; phase: string; error?: string; recovery_error?: string; recovery_warnings?: string[]; remaining_health_issues?: HealthIssue[]; started_at: string; plan: UpdatePlan}
export interface UpdateNotice {id: string; kind: string; deployment: string; created_at: string; read: boolean; dismissed: boolean}
export interface HostUpdater {
    api: number;
    capabilities?: string[];
    runtime?: {selection?: Record<string,string>; selection_pending?: boolean; identity: string; health: string; checked_at: string; error?: string; components?: Record<string, {name?:string; family?:string; reference?:string; version?:string; revision?:string; built_at?:string; image: string; healthy: boolean; healthcheck: boolean}>};
    agent: {build_id?: string; version: string; revision: string; platform: string; error?: string};
    trusted_repositories: string[];
    state: {custom_images?:Record<string,CustomImage>; active_job_id?: string; policy: UpdatePolicy; installed_policy?: UpdatePolicy; overrides?: Record<string, Deployment>; active?: Deployment; last_check: string; last_success: string; next_check: string; check_error?: string; releases: Deployment[]; notices: UpdateNotice[]; job?: UpdateJob; history: UpdateJob[]};
}
async function readUpdaterResponse<T>(operation: string, response: Response): Promise<T> {
    const text = await response.text();
    const data = text ? JSON.parse(text) as T & {error?: string} : undefined;
    if (!response.ok) throw new Error(data?.error || `HTTP ${response.status}`);
    if (operation === 'state' && (!data || typeof data !== 'object' || !('state' in data) || !('agent' in data))) throw new Error('Host updater unavailable');
    return data as T;
}
export async function updaterRequest<T>(operation: string, body?: unknown): Promise<T> {
    const response = await fetch(`/api/system/updater/${operation}`, body === undefined ? {cache: 'no-store'} : {
        method: 'POST', headers: {'Content-Type': 'application/json', 'X-Mowgli-Update': '1'}, body: JSON.stringify(body),
    });
    return readUpdaterResponse<T>(operation, response);
}
// Shared, ref-counted poller — same shape as multiplexedSocket.ts's WS-topic
// sharing, for the same reason. useHostUpdater() is called from AppShell,
// useNotificationCenter AND HostUpdaterPanel, all mounted at once in normal
// use; each used to run its OWN independent 5s setInterval + fetch, so two
// or three uncoordinated pollers hit `state` (the state payload — releases,
// history, everything — is easily >1 MB) within milliseconds of each other
// on every cycle. Field-reported 2026-09-28: two ~1.34 MB `state` GETs back
// to back, every ~5s, regardless of which page was open (misread at first
// as a Map-page-specific issue — this endpoint has nothing to do with the
// map). Now there is ONE poller total, shared by however many components
// currently want the data; the last one to unmount stops it.
//
// That payload is the whole updater ledger and it hardly ever changes, yet the
// robot's wifi (weak away from the house, exactly where it mows) carried it every
// 5 s. So the poller is also frugal:
//  - background polls are CONDITIONAL (If-None-Match): the server answers 304 with
//    no body when nothing changed, and we keep what we have;
//  - it is slow (POLL_IDLE_MS) unless something is going on: an update job is
//    running, the updater panel is open (it shows live health), or the last poll
//    failed;
//  - it does not run at all while the tab is hidden, and catches up the moment the
//    tab is shown again.
// Polling only reads cached local status. It never schedules a registry check.
export const POLL_FAST_MS = 5_000;
export const POLL_IDLE_MS = 30_000;
const JOB_DONE_PHASES = ['succeeded', 'rolled_back', 'failed'];

let sharedData: HostUpdater | undefined;
let sharedError: string | undefined;
let pollTimer: number | undefined;
let stateEtag: string | undefined;
let liveListeners = 0;
const listeners = new Set<() => void>();

function notifyListeners(): void {
    for (const listener of listeners) listener();
}

// `state.active_job_id` is NOT "a job is running": it names the job behind the installed
// deployment and stays set after every successful update. Only `state.job` in a phase that
// is not final means an update is going on (the updater's Job.Pending()).
function updateInProgress(data: HostUpdater | undefined): boolean {
    const job = data?.state?.job;
    return !!job && !JOB_DONE_PHASES.includes(job.phase);
}

function nextDelay(): number {
    const busy = liveListeners > 0 || sharedData === undefined || sharedError !== undefined || updateInProgress(sharedData);
    return busy ? POLL_FAST_MS : POLL_IDLE_MS;
}

async function refreshState(conditional: boolean): Promise<void> {
    try {
        const headers: Record<string, string> = {};
        if (conditional && stateEtag) headers['If-None-Match'] = stateEtag;
        const response = await fetch('/api/system/updater/state', {cache: 'no-store', headers});
        if (response.status === 304 && sharedData) {
            // Nothing changed since the copy we already hold.
        } else {
            sharedData = await readUpdaterResponse<HostUpdater>('state', response);
            stateEtag = response.headers.get('ETag') ?? undefined;
        }
        sharedError = undefined;
    } catch (e) {
        sharedError = e instanceof Error ? e.message : String(e);
        stateEtag = undefined;
    }
    notifyListeners();
}

function schedulePoll(): void {
    if (pollTimer !== undefined) window.clearTimeout(pollTimer);
    pollTimer = undefined;
    if (listeners.size === 0 || document.hidden) return;
    pollTimer = window.setTimeout(() => {
        pollTimer = undefined;
        void poll();
    }, nextDelay());
}

async function poll(): Promise<void> {
    // The open updater panel shows live runtime health, so it always gets a full copy.
    await refreshState(liveListeners === 0);
    schedulePoll();
}

// An explicit refresh (after the operator did something) is never conditional.
async function refreshNow(): Promise<void> {
    await refreshState(false);
    schedulePoll();
}

function onVisibilityChange(): void {
    if (document.hidden) {
        if (pollTimer !== undefined) window.clearTimeout(pollTimer);
        pollTimer = undefined;
    } else {
        void poll();
    }
}

export function useHostUpdater(options?: {live?: boolean}) {
    const live = options?.live ?? false;
    const [data, setData] = useState<HostUpdater | undefined>(sharedData);
    const [error, setError] = useState<string | undefined>(sharedError);

    useEffect(() => {
        const listener = () => {
            setData(sharedData);
            setError(sharedError);
        };
        const isFirstListener = listeners.size === 0;
        listeners.add(listener);
        if (isFirstListener) {
            document.addEventListener('visibilitychange', onVisibilityChange);
            if (document.hidden) schedulePoll();
            else void refreshNow();
        } else {
            // A poller is already running and may already have fresher data
            // than what this component's initial useState captured at mount.
            listener();
        }
        return () => {
            listeners.delete(listener);
            if (listeners.size === 0) {
                if (pollTimer !== undefined) window.clearTimeout(pollTimer);
                pollTimer = undefined;
                document.removeEventListener('visibilitychange', onVisibilityChange);
            }
        };
    }, []);

    useEffect(() => {
        if (!live) return;
        liveListeners += 1;
        // Entering live mode: show current values at once and poll at the fast rate.
        void refreshNow();
        return () => {
            liveListeners -= 1;
            schedulePoll();
        };
    }, [live]);

    return {data, error, refresh: refreshNow};
}

export function compatibleGUI(base: Deployment, gui: Deployment): boolean {
    return !!base.gui_compatibility && base.gui_compatibility === gui.gui_compatibility &&
        base.source.repository === gui.source.repository &&
        ['layout', 'data_schema', 'updater_api', 'maintenance_api', 'firmware_protocol'].every(key =>
            base[key as keyof Deployment] !== undefined && base[key as keyof Deployment] === gui[key as keyof Deployment]);
}

// Deployment IDs also change for frontend/ROS-only releases. Prefer the
// publisher's executable-input identity; retain old-worker compatibility.
export function sameUpdaterBuild(candidate: {version: string; build_id?: string}, running: {version: string; build_id?: string}): boolean {
    return candidate.build_id && running.build_id ? candidate.build_id === running.build_id : candidate.version === running.version;
}

export function componentCompatibility(base: Deployment, candidate: Deployment, family: string, platform: string): 'source' | 'contract' | 'platform' | undefined {
    if (base.source.repository !== candidate.source.repository || base.source.track !== candidate.source.track || base.source.branch !== candidate.source.branch) return 'source';
    const expected = base.component_compatibility?.[family];
    const actual = candidate.component_compatibility?.[family];
    const legacyGUI = family === 'mowglinext-gui' && !expected && !actual && compatibleGUI(base, candidate);
    if (candidate.id !== base.id && !legacyGUI && (!expected || expected !== actual ||
        !['layout','data_schema','updater_api','maintenance_api','firmware_protocol'].every(key => base[key as keyof Deployment] !== undefined && base[key as keyof Deployment] === candidate[key as keyof Deployment]))) return 'contract';
    if (!candidate.images?.[family]?.platforms[platform]) return 'platform';
}
