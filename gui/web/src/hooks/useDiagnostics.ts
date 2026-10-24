import {useCallback, useEffect, useLayoutEffect, useRef, useState} from "react";
import {isDiagnosticStatus, diagnosticPublicationIdentity, advanceDiagnosticProgress, type DiagnosticProgress} from '../utils/diagnosticTelemetry';
import {DiagnosticTransitionMemory, type DiagnosticTransitionCursor} from '../utils/diagnosticTransitions';
import {createDiagnosticEpisodeState, readDiagnosticEpisodes, reduceDiagnosticEpisodes,
    DIAGNOSTIC_EPISODE_IDENTITY_LIMIT, type DiagnosticEpisodeState} from '../utils/diagnosticEpisodes';
import {DIAGNOSTIC_STALE_MS, diagnosticReportFresh, qualifyDiagnosticReport} from '../utils/diagnosticReportPolicy';
import {isJsonObject} from '../utils/json';
import {useWS} from "./useWS.ts";

export interface DiagnosticStatus {
    level: number;
    name: string;
    message: string;
    hardware_id: string;
    values: { key: string; value: string }[];
}

/** Browser receipt metadata, separate from ROS publication/acquisition time. */
export interface TimestampedDiagnosticStatus extends DiagnosticStatus {
    /** Civil date retained for the existing receipt contract; not an elapsed clock. */
    receivedAt: number;
    /** Monotonic instant of that same distinct receipt; replay does not renew it. */
    receivedMonotonic: number;
    /** Opaque publication identity; cached replay does not renew receivedAt. */
    publicationStamp?: string;
    /** Monotonic ordered-publication evidence; not a physical acquisition time. */
    healthProgressAt?: number;
    healthBlocked?: boolean;
}

export interface DiagnosticArray {
    status?: TimestampedDiagnosticStatus[];
}

/** Entries not refreshed within this window should be rendered as stale. */
export {DIAGNOSTIC_STALE_MS} from '../utils/diagnosticReportPolicy';

/** True when an accumulated entry has not been refreshed for DIAGNOSTIC_STALE_MS. */
export const isDiagnosticStale = (
    entry: Pick<TimestampedDiagnosticStatus, "receivedMonotonic">,
    nowMonotonic: number,
): boolean => !Number.isFinite(entry.receivedMonotonic)
    || nowMonotonic - entry.receivedMonotonic > DIAGNOSTIC_STALE_MS;

export interface DiagnosticEpisodeAdmission {
    connected: boolean;
    lidarEnabled?: unknown;
    /** Input availability only, not DDS/process authority. Missing inputs cannot qualify an observation. */
    ready?: boolean;
}

/**
 * Subscribes to the /diagnostics topic and accumulates entries by name.
 * Entries are merged (latest wins per name), stamped with `receivedAt`
 * so consumers can grey out entries that stopped updating, and throttled
 * to avoid flicker from high-frequency diagnostic updates.
 */
export const useDiagnostics = (transitionContext?: string, episodeAdmission?: DiagnosticEpisodeAdmission) => {
    const [diagnostics, setDiagnostics] = useState<DiagnosticArray>({})
    // Local monotonic disconnect boundary, never a civil date or ROS session.
    const [disconnectedAt, setDisconnectedAt] = useState<number | null>(null);
    const healthProgress = useRef(new Map<string, DiagnosticProgress>());
    const sourceForName = useRef(new Map<string, string>());
    const malformedNames = useRef(new Set<string>());
    const receiptsRef = useRef(new Map<string, {identity: string; at: number; monotonic: number}>());
    const accumulatorRef = useRef<Map<string, TimestampedDiagnosticStatus>>(new Map());
    const throttleRef = useRef<ReturnType<typeof setTimeout> | null>(null);
    const [transitionMemory] = useState(() => new DiagnosticTransitionMemory());
    const episodesRef = useRef<DiagnosticEpisodeState | null>(null);
    const episodeAdmissionRef = useRef(episodeAdmission);
    const episodeTransportConnected = useRef(true);
    // A reconnect cannot reconstruct the context's missing history. This flag
    // affects continuity only; an admissible OK still resolves immediately.
    const episodeConnectionGap = useRef(false);
    const episodesEnabled = episodeAdmission !== undefined;
    useLayoutEffect(() => { episodeAdmissionRef.current = episodeAdmission; }, [episodeAdmission]);
    useLayoutEffect(() => {
        episodeConnectionGap.current = false;
        episodesRef.current = episodesEnabled && transitionContext !== undefined && transitionContext.length <= DIAGNOSTIC_EPISODE_IDENTITY_LIMIT
            ? createDiagnosticEpisodeState(Symbol('diagnostic episodes'), transitionContext) : null;
        return () => { episodesRef.current = null; };
    }, [transitionContext, episodesEnabled]);

    // Lazy projection uses existing progress metadata, not a second payload cache
    // or timer. Also run before new progress can hide an unobserved expiry gap.
    const expireEpisodes = useCallback((nowMonotonic: number) => {
        let state = episodesRef.current;
        const options = episodeAdmissionRef.current;
        if (!state || !options) return;
        const admission = {...options, connected: options.connected && episodeTransportConnected.current};
        for (const episode of state.episodes) {
            if (episode.state !== 'active') continue;
            const progress = healthProgress.current.get(JSON.stringify([episode.identity.name, episode.identity.hardware_id]));
            if (admission.ready !== false && progress && diagnosticReportFresh({receivedMonotonic: progress.at, healthProgressAt: progress.at, healthBlocked: progress.blocked},
                {...admission, nowMonotonic})) continue;
            const expired = admission.ready !== false && admission.connected && progress && !progress.blocked && Number.isFinite(progress.at)
                && nowMonotonic >= progress.at && nowMonotonic - progress.at >= DIAGNOSTIC_STALE_MS;
            state = reduceDiagnosticEpisodes(state, {scope: state.scope, context: state.context, order: state.lastOrder + 1,
                identity: episode.identity, continuity: 'unknown', admission: 'control', classification: expired ? 'expired' : 'unverified'});
        }
        episodesRef.current = state;
    }, []);
    const invalidateEpisodes = (name?: string) => {
        let state = episodesRef.current;
        if (!state) return;
        for (const episode of state.episodes) {
            if (episode.state === 'resolved' || name !== undefined && episode.identity.name !== name) continue;
            state = reduceDiagnosticEpisodes(state, {scope: state.scope, context: state.context, order: state.lastOrder + 1,
                identity: episode.identity, continuity: 'unknown', admission: 'control', classification: 'unverified'});
        }
        episodesRef.current = state;
    };
    const readEpisodes = useCallback(() => {
        expireEpisodes(performance.now());
        return episodesRef.current ? readDiagnosticEpisodes(episodesRef.current) : null;
    }, [expireEpisodes]);

    // Opt-in only. Context changes clear observations without changing the
    // current snapshot, its subscriptions, admission or freshness barriers.
    useLayoutEffect(() => {
        if (transitionContext !== undefined) transitionMemory.begin(transitionContext, accumulatorRef.current.values());
        return () => transitionMemory.clear();
    }, [transitionContext, transitionMemory]);
    const readTransitions = useCallback((cursor?: DiagnosticTransitionCursor) =>
        transitionMemory.readAfter(cursor), [transitionMemory]);

    const diagnosticsStream = useWS<unknown>(() => {
        episodeTransportConnected.current = false;
        episodeConnectionGap.current = true;
        invalidateEpisodes();
        setDisconnectedAt(performance.now());
        for (const [key, value] of healthProgress.current) healthProgress.current.set(key, {...value, blocked: true});
        for (const [key, entry] of accumulatorRef.current) {
            const next = {...entry, healthBlocked: true};
            transitionMemory.trackSnapshotCopy(entry, next);
            accumulatorRef.current.set(key, next);
        }
        setDiagnostics({status: Array.from(accumulatorRef.current.values())});
    }, () => { episodeTransportConnected.current = true; },
        (e) => {
            if (!isJsonObject(e) || !Array.isArray(e.status)) return;
            const entries: unknown[] = e.status;
            // Merge incoming entries by name (latest value wins)
            if (entries.length > 0) {
                const receivedAt = Date.now();
                const receivedMonotonic = performance.now();
                expireEpisodes(receivedMonotonic);
                const identity = diagnosticPublicationIdentity(e);
                for (const entry of entries) {
                    if (isDiagnosticStatus(entry)) {
                        const key = JSON.stringify([entry.name, entry.hardware_id]);
                        const known = healthProgress.current.get(key);
                        const wasMalformed = malformedNames.current.has(entry.name);
                        const before = wasMalformed && known ? {...known, blocked: true} : known;
                        let progress = advanceDiagnosticProgress(before, e, receivedMonotonic);
                        const oldSource = sourceForName.current.get(entry.name);
                        if (oldSource !== undefined && oldSource !== key && (!before || before.stamp === progress.stamp)) {
                            progress = {...progress, blocked: true};
                        }
                        healthProgress.current.set(key, progress);
                        sourceForName.current.set(entry.name, key);
                        if (!progress.blocked) malformedNames.current.delete(entry.name);
                        const previous = receiptsRef.current.get(entry.name);
                        const at = identity !== null && previous?.identity === identity ? previous.at : receivedAt;
                        const monotonic = identity !== null && previous?.identity === identity ? previous.monotonic : receivedMonotonic;
                        if (identity !== null) receiptsRef.current.set(entry.name, {identity, at, monotonic});
                        const next = {
                            ...entry, receivedAt: at, receivedMonotonic: monotonic, healthProgressAt: progress.at, healthBlocked: progress.blocked, ...(identity !== null ? {publicationStamp: identity} : {}),
                        };
                        transitionMemory.observe(accumulatorRef.current.get(entry.name), next);
                        const state = episodesRef.current;
                        const admission = episodeAdmissionRef.current;
                        if (state && admission) {
                            const qualification = qualifyDiagnosticReport(next, {...admission,
                                connected: admission.connected && episodeTransportConnected.current, nowMonotonic: receivedMonotonic});
                            episodesRef.current = reduceDiagnosticEpisodes(state, {scope: state.scope, context: state.context,
                                order: state.lastOrder + 1, identity: entry, continuity: qualification.admission === 'accepted'
                                    && !wasMalformed && !before?.blocked && !episodeConnectionGap.current ? 'continuous' : 'unknown',
                                ...qualification});
                        }
                        accumulatorRef.current.set(entry.name, next);
                    } else if (isJsonObject(entry) && typeof entry.name === 'string') {
                        // A named malformed update cannot renew an older valid entry.
                        accumulatorRef.current.delete(entry.name);
                        malformedNames.current.add(entry.name);
                        invalidateEpisodes(entry.name);
                    }
                }
            }
            // Throttle state updates to max 1 per second
            if (!throttleRef.current) {
                throttleRef.current = setTimeout(() => {
                    throttleRef.current = null;
                    setDiagnostics({
                        status: Array.from(accumulatorRef.current.values()),
                    });
                }, 1000);
            }
        })

    const {start, stop} = diagnosticsStream;
    useEffect(() => {
        start("/api/mowglinext/subscribe/diagnostics")
        return () => {
            stop()
            if (throttleRef.current) {
                clearTimeout(throttleRef.current);
                throttleRef.current = null;
            }
        }
    }, [start, stop]);

    return {diagnostics, disconnectedAt, stop: diagnosticsStream.stop, start: diagnosticsStream.start, readTransitions, readEpisodes};
};
