import {useCallback, useEffect, useRef, useState} from 'react';
import type {ApiUpdateCheck} from '../api/Api';
import {httpBase} from '../utils/apiHost';

export function useUpdateChecks() {
    const [channel, setChannel] = useState<'dev' | 'stable'>(() => {
        try { return localStorage.getItem('updateComparisonChannel') === 'stable' ? 'stable' : 'dev'; } catch { return 'dev'; }
    });
    const [result, setResult] = useState<ApiUpdateCheck | null>(null);
    const [checking, setChecking] = useState(false);
    const [error, setError] = useState(false);
    const controller = useRef<AbortController | null>(null);
    const fetchCheck = useCallback(async (check: boolean) => {
        controller.current?.abort();
        const request = new AbortController();
        controller.current = request;
        setChecking(true);
        try {
            const response = await fetch(`${httpBase()}/api/system/updates?channel=${channel}&check=${String(check)}`, {signal: request.signal, cache: 'no-store'});
            if (!response.ok) throw new Error('update check unavailable');
            const next = await response.json() as ApiUpdateCheck;
            if (next.channel !== channel || !Array.isArray(next.components) || !next.state) throw new Error('invalid update check');
            if (!request.signal.aborted) { setResult(next); setError(false); }
        } catch { if (!request.signal.aborted) setError(true); }
        finally { if (!request.signal.aborted) setChecking(false); }
    }, [channel]);
    useEffect(() => {
        try { localStorage.setItem('updateComparisonChannel', channel); } catch { /* browser storage is optional */ }
        void fetchCheck(false);
        return () => controller.current?.abort();
    }, [channel, fetchCheck]);
    return {channel, setChannel, result: result?.channel === channel ? result : null, checking, error, check: () => fetchCheck(true)};
}
