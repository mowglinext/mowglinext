import {useCallback, useEffect, useRef, useState} from 'react';
import {useTranslation} from 'react-i18next';
import type {NotificationInstance} from 'antd/es/notification/interface';
import {observeAsync} from '../../../utils/async';
import {requestErrorMessage} from '../../../utils/errors';

export interface MapDisplayPreferenceOptions {
    config: Record<string, string>;
    setConfig: (values: Record<string, string>) => Promise<void>;
    notification: Pick<NotificationInstance, 'error'>;
}
type PreferenceKey = 'gui.map.offset.x' | 'gui.map.offset.y' | 'gui.map.display.bearing';

// These are GUI preferences, not physical measurements. Missing keys use the
// existing display default; malformed values retain the last usable display.
function configNumber(raw: unknown): number | undefined {
    if (raw === undefined) return 0;
    if (typeof raw !== 'string' || !/^[+-]?(?:\d+(?:\.\d*)?|\.\d+)(?:[eE][+-]?\d+)?$/.test(raw.trim())) return undefined;
    const number = Number(raw.trim());
    return Number.isFinite(number) ? number : undefined;
}

/** One display preference owns its debounce and ordered writes. Shared by map
 * offsets and bearing so rerenders and navigation cannot change their lifetime. */
export function useMapDisplayPreference(options: MapDisplayPreferenceOptions, key: PreferenceKey, debounceMs: number, errorKey: string) {
    const {t} = useTranslation();
    const raw = options.config[key];
    const [value, setValue] = useState(() => configNumber(raw) ?? 0);
    const pending = useRef<{value: number; revision: number} | null>(null);
    const revision = useRef(0);
    const draft = useRef(false);
    const mounted = useRef(true);
    const timer = useRef<ReturnType<typeof setTimeout> | null>(null);
    const writes = useRef<Promise<void>>(Promise.resolve());
    const latest = useRef({writer: options.setConfig, notification: options.notification, message: t(errorKey)});
    useEffect(() => {
        latest.current = {writer: options.setConfig, notification: options.notification, message: t(errorKey)};
    }, [options.setConfig, options.notification, t, errorKey]);
    useEffect(() => {
        const decoded = configNumber(raw);
        // A late load/old write echo must not replace an unsaved newer edit.
        if (decoded !== undefined && !draft.current) setValue(decoded);
    }, [raw]);

    const flush = useCallback(() => {
        if (timer.current !== null) clearTimeout(timer.current);
        timer.current = null;
        const change = pending.current;
        if (change === null) return;
        pending.current = null;
        const write = async () => {
            try {
                await latest.current.writer({[key]: String(change.value)});
                if (revision.current === change.revision) draft.current = false;
            } catch (error: unknown) {
                // Failed drafts stay visible, without retry or success indication.
                // Superseded failures cannot overwrite the newer request's result.
                if (mounted.current && revision.current === change.revision) {
                    latest.current.notification.error({message: latest.current.message,
                        description: requestErrorMessage(error) ?? latest.current.message});
                }
            }
        };
        writes.current = writes.current.then(write, write);
        observeAsync(writes.current);
    }, [key]);
    const handleValue = useCallback((next: number) => {
        if (!mounted.current || !Number.isFinite(next)) return;
        draft.current = true;
        pending.current = {value: next, revision: ++revision.current};
        setValue(next);
        if (timer.current !== null) clearTimeout(timer.current);
        timer.current = setTimeout(flush, debounceMs);
    }, [flush, debounceMs]);
    useEffect(() => {
        mounted.current = true;
        return () => {
            mounted.current = false;
            // Persist an already requested display edit once when leaving. This
            // preserves bearing's existing quick-rotate/navigation behavior.
            flush();
        };
    }, [flush]);
    return {value, handleValue};
}
