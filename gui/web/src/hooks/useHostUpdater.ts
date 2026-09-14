import {useCallback, useEffect, useState} from 'react';

export interface UpdateSource {repository: string; track: 'stable' | 'dev' | 'custom'; branch: string}
export interface UpdatePolicy {source: UpdateSource; interval_hours: number; pinned: boolean}
export interface Deployment {component_compatibility?: Record<string, string>; service_choices?: {service: string; image: string; when?: Record<string,string>}[]; images?: Record<string,{repository:string; platforms:Record<string,{manifest:string}>}>; gui_compatibility?: string; layout?: number; data_schema?: number; updater_api?: number; maintenance_api?: number; firmware_protocol?: number; release_tag?: string; id: string; source: UpdateSource; revision: string; published_at: string; updater: Record<string, {version: string}>}
export interface CustomImage {repository?:string; release_tag?:string; deployment_id?:string; requested:string; reference:string; image_id:string; version?:string; revision?:string; built_at?:string}
export interface UpdatePlan {custom_images?:Record<string,CustomImage>; stack?: {changes: {service: string; action: string}[]; selection: {options: Record<string, string>}}; overrides?: Record<string, Deployment>; id: string; target: Deployment; images: Record<string, string>; previous: Record<string, string>; expires_at: string}
export interface UpdateJob {id: string; kind: string; phase: string; error?: string; started_at: string; plan: UpdatePlan}
export interface UpdateNotice {id: string; kind: string; deployment: string; created_at: string; read: boolean; dismissed: boolean}
export interface HostUpdater {
    api: number;
    capabilities?: string[];
    runtime?: {selection?: Record<string,string>; selection_pending?: boolean; identity: string; health: string; checked_at: string; error?: string; components?: Record<string, {name?:string; family?:string; reference?:string; version?:string; revision?:string; image: string; healthy: boolean; healthcheck: boolean}>};
    agent: {version: string; revision: string; platform: string; error?: string};
    trusted_repositories: string[];
    state: {custom_images?:Record<string,CustomImage>; active_job_id?: string; policy: UpdatePolicy; installed_policy?: UpdatePolicy; overrides?: Record<string, Deployment>; active?: Deployment; last_check: string; last_success: string; next_check: string; check_error?: string; releases: Deployment[]; notices: UpdateNotice[]; job?: UpdateJob; history: UpdateJob[]};
}
export async function updaterRequest<T>(operation: string, body?: unknown): Promise<T> {
    const response = await fetch(`/api/system/updater/${operation}`, body === undefined ? {cache: 'no-store'} : {
        method: 'POST', headers: {'Content-Type': 'application/json', 'X-Mowgli-Update': '1'}, body: JSON.stringify(body),
    });
    const text = await response.text();
    const data = text ? JSON.parse(text) as T & {error?: string} : undefined;
    if (!response.ok) throw new Error(data?.error || `HTTP ${response.status}`);
    if (operation === 'state' && (!data || typeof data !== 'object' || !('state' in data) || !('agent' in data))) throw new Error('Host updater unavailable');
    return data as T;
}
// Polling only reads cached local status. It never schedules a registry check.
export function useHostUpdater() {
    const [data, setData] = useState<HostUpdater>();
    const [error, setError] = useState<string>();
    const refresh = useCallback(async () => {
        try {setData(await updaterRequest<HostUpdater>('state')); setError(undefined);}
        catch (e) {setError(e instanceof Error ? e.message : String(e));}
    }, []);
    useEffect(() => {void refresh(); const timer = window.setInterval(() => void refresh(), 5000); return () => window.clearInterval(timer);}, [refresh]);
    return {data, error, refresh};
}

export function compatibleGUI(base: Deployment, gui: Deployment): boolean {
    return !!base.gui_compatibility && base.gui_compatibility === gui.gui_compatibility &&
        base.source.repository === gui.source.repository &&
        ['layout', 'data_schema', 'updater_api', 'maintenance_api', 'firmware_protocol'].every(key =>
            base[key as keyof Deployment] !== undefined && base[key as keyof Deployment] === gui[key as keyof Deployment]);
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
