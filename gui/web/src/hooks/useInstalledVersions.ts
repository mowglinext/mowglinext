import {useCallback, useEffect, useRef, useState} from 'react';
import type {ApiVersionsResponse} from '../api/Api';
import {httpBase} from '../utils/apiHost';

export const browserBuild = {
    id: import.meta.env.VITE_WEB_BUILD_ID as string | undefined,
    revision: import.meta.env.VITE_BUILD_REVISION as string | undefined,
    version: import.meta.env.VITE_BUILD_VERSION as string | undefined,
    built_at: import.meta.env.VITE_BUILD_TIME as string | undefined,
};

export function useInstalledVersions() {
    const [data, setData] = useState<ApiVersionsResponse | null>(null);
    const [loading, setLoading] = useState(true);
    const [error, setError] = useState(false);
    const controller = useRef<AbortController | null>(null);
    const refresh = useCallback(async () => {
        controller.current?.abort();
        const request = new AbortController();
        controller.current = request;
        setLoading(true);
        try {
            const response = await fetch(`${httpBase()}/api/system/versions`, {signal: request.signal, cache: 'no-store'});
            if (!response.ok) throw new Error('version inventory unavailable');
            const next: ApiVersionsResponse = await response.json();
            if (!Array.isArray(next.components) || typeof next.docker_available !== 'boolean') throw new Error('invalid inventory');
            if (!request.signal.aborted) { setData(next); setError(false); }
        } catch {
            if (!request.signal.aborted) setError(true);
        } finally {
            if (!request.signal.aborted) setLoading(false);
        }
    }, []);
    useEffect(() => {
        void refresh();
        const poll = window.setInterval(() => { if (!document.hidden) void refresh(); }, 30000);
        const focus = () => { void refresh(); };
        window.addEventListener('focus', focus);
        return () => { controller.current?.abort(); window.clearInterval(poll); window.removeEventListener('focus', focus); };
    }, [refresh]);
    return {data, loading, error, refresh};
}


export interface WebBuildInfo {id: string; revision?: string; version?: string; built_at?: string}

// Read the identity packaged beside the currently served assets, never the Go
// binary's identity. Missing manifests on older/manual servers are unknown.
export function useServedWebBuild() {
    const [build, setBuild] = useState<WebBuildInfo>();
    useEffect(() => {
        let request: AbortController | undefined;
        const refresh = async () => {
            request?.abort();
            const current = new AbortController(); request = current;
            try {
                const response = await fetch('/web-build.json', {cache:'no-store', signal:current.signal});
                if (!response.ok) throw new Error('web build unavailable');
                const data: unknown = await response.json();
                if (!data || typeof data !== 'object' || !('id' in data) || typeof data.id !== 'string' || !data.id) throw new Error('invalid web build');
                if (['revision','version','built_at'].some(key => key in data && typeof (data as Record<string,unknown>)[key] !== 'string')) throw new Error('invalid web metadata');
                if (!current.signal.aborted) setBuild(data as WebBuildInfo);
            } catch {
                if (!current.signal.aborted) setBuild(undefined);
            }
        };
        const focus = () => {void refresh();};
        void refresh();
        const timer = window.setInterval(() => {if (!document.hidden) void refresh();}, 30000);
        window.addEventListener('focus', focus);
        return () => {request?.abort(); window.clearInterval(timer); window.removeEventListener('focus', focus);};
    }, []);
    return build;
}
