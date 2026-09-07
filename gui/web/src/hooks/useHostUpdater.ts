import {useCallback, useEffect, useState} from 'react';

export interface UpdateSource {repository: string; track: 'stable' | 'dev' | 'custom'; branch: string}
export interface UpdatePolicy {source: UpdateSource; interval_hours: number; pinned: boolean}
export interface Deployment {id: string; source: UpdateSource; revision: string; published_at: string; updater: Record<string, {version: string}>}
export interface UpdatePlan {id: string; target: Deployment; images: Record<string, string>; previous: Record<string, string>; expires_at: string}
export interface UpdateJob {id: string; kind: string; phase: string; error?: string; started_at: string; plan: UpdatePlan}
export interface UpdateNotice {id: string; kind: string; deployment: string; created_at: string; read: boolean; dismissed: boolean}
export interface HostUpdater {
    api: number;
    agent: {version: string; revision: string; platform: string; error?: string};
    trusted_repositories: string[];
    state: {policy: UpdatePolicy; installed_policy?: UpdatePolicy; active?: Deployment; last_check: string; last_success: string; next_check: string; check_error?: string; releases: Deployment[]; notices: UpdateNotice[]; job?: UpdateJob; history: UpdateJob[]};
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
// Polling only reads persisted local status. It never schedules a registry check.
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
