import {useCallback, useEffect, useRef, useState} from 'react';
import {errorMessage, isAbortError} from '../../utils/errors';
import {extensionRequest, type ExtensionOperation, type ExtensionState} from './api';

/** An idle page does no background work. Only an active operation gets polled. */
export function useExtensions() {
    const [state, setState] = useState<ExtensionState>();
    const [error, setError] = useState('');
    const [loading, setLoading] = useState(true);
    const [visible, setVisible] = useState(document.visibilityState !== 'hidden');
    const request = useRef<AbortController | null>(null);
    const alive = useRef(true);
    const refresh = useCallback(async () => {
        request.current?.abort();
        const controller = new AbortController();
        request.current = controller;
        setLoading(true);
        try {
            const next = await extensionRequest<ExtensionState>('state', undefined, controller.signal);
            if (!alive.current || controller.signal.aborted) return;
            setState(next);
            setError('');
        } catch (e) {
            if (alive.current && !controller.signal.aborted && !isAbortError(e)) setError(errorMessage(e));
        } finally {
            if (alive.current && !controller.signal.aborted) setLoading(false);
        }
    }, []);

    useEffect(() => {
        alive.current = true;
        void refresh();
        const onVisibility = () => setVisible(document.visibilityState !== 'hidden');
        document.addEventListener('visibilitychange', onVisibility);
        return () => {
            alive.current = false;
            request.current?.abort();
            document.removeEventListener('visibilitychange', onVisibility);
        };
    }, [refresh]);
    const running = state?.operation?.phase === 'running';
    useEffect(() => {
        if (!running || !visible || loading) return;
        const timer = window.setTimeout(() => { void refresh(); }, 1000);
        return () => window.clearTimeout(timer);
    }, [running, visible, loading, refresh]);

    const acceptOperation = (operation: ExtensionOperation) => {
        // A previously started read must not overwrite the newly accepted job.
        request.current?.abort();
        setLoading(false);
        setState(previous => ({api: 1, configured: true, entries: previous?.entries ?? [], ...previous, operation}));
    };
    return {state, error, loading, refresh, acceptOperation, running};
}
