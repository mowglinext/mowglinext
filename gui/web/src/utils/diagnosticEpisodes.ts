/** Local report episodes, not proof of physical acquisition or producer identity. */
export const DIAGNOSTIC_EPISODE_CAPACITY = 64;
export const DIAGNOSTIC_EPISODE_IDENTITY_LIMIT = 4096;

export interface EpisodeIdentity {readonly name: string; readonly hardware_id: string}
export type EpisodeContinuity = 'continuous' | 'unknown';
interface DecisionContext {
    readonly scope: symbol;
    readonly context: string;
    /** Strictly increasing, safe local decision order, not a ROS stamp. Gaps are allowed. */
    readonly order: number;
    readonly identity: EpisodeIdentity;
    readonly continuity: EpisodeContinuity;
    /** Optional opaque input only. Never parsed, retained or used for admission. */
    readonly publicationStamp?: string | null;
}
/** Qualification belongs to the caller; this module has no ROS/health dependency. */
export type DiagnosticEpisodeDecision = DecisionContext & (
    | {readonly admission: 'accepted'; readonly classification: 'fault'; readonly severity: 1 | 2}
    | {readonly admission: 'accepted'; readonly classification: 'explicit_ok'}
    | {readonly admission: 'rejected'; readonly classification: 'fault' | 'explicit_ok'}
    | {readonly admission: 'control'; readonly classification: 'expired' | 'unverified' | 'unavailable' | 'not_applicable'}
);

export interface DiagnosticEpisode {
    /** Opening decision order; unique only within this RAM scope. */
    readonly id: number;
    readonly identity: EpisodeIdentity;
    readonly state: 'active' | 'resolved' | 'expired' | 'unknown';
    readonly continuity: EpisodeContinuity;
    readonly opening: 'first_observed' | 'after_explicit_ok' | 'unknown_history';
    readonly lastFaultSeverity: 1 | 2;
    readonly peakSeverity: 1 | 2;
    /** Changes only for episode content, not identical republications or consultation. */
    readonly version: number;
    readonly resolvedOrder?: number;
    readonly unconsulted: boolean;
}
export interface EpisodeLosses {
    /** Old resolved, consulted episodes removed to admit another opening. */
    readonly evicted: number;
    /** Omitted observations, not a count of distinct material incidents. */
    readonly omitted: number;
    /** Retained for loss accounting; this policy never evicts unread episodes. */
    readonly unconsultedEvicted: number;
    readonly pending: boolean;
    readonly version: number;
}
export interface DiagnosticEpisodeState {
    readonly scope: symbol;
    readonly context: string;
    readonly capacity: number;
    readonly lastOrder: number;
    readonly episodes: readonly DiagnosticEpisode[];
    readonly losses: EpisodeLosses;
    /** Consultation cannot restore history that was not retained. */
    readonly historyIncomplete: boolean;
}
export interface DiagnosticEpisodeReceipt {
    readonly scope: symbol;
    readonly context: string;
    readonly episodes: readonly {readonly id: number; readonly version: number}[];
    readonly lossVersion?: number;
}

/** Caller supplies a fresh scope on mount/context change; no implicit reset on receipt. */
export function createDiagnosticEpisodeState(scope: symbol, context: string, capacity = DIAGNOSTIC_EPISODE_CAPACITY): DiagnosticEpisodeState {
    if (!Number.isInteger(capacity) || capacity < 1 || capacity > DIAGNOSTIC_EPISODE_CAPACITY
        || context.length > DIAGNOSTIC_EPISODE_IDENTITY_LIMIT) throw new RangeError('Invalid episode capacity/context');
    return Object.freeze({scope, context, capacity, lastOrder: 0, episodes: Object.freeze([]),
        losses: Object.freeze({evicted: 0, omitted: 0, unconsultedEvicted: 0, pending: false, version: 0}), historyIncomplete: false});
}

const increment = (value: number) => Math.min(Number.MAX_SAFE_INTEGER, value + 1);
const sameIdentity = (a: EpisodeIdentity, b: EpisodeIdentity) => a.name === b.name && a.hardware_id === b.hardware_id;

/** Pure, bounded reducer. Expiry/invalidity are explicit caller decisions, never inferred here. */
export function reduceDiagnosticEpisodes(state: DiagnosticEpisodeState, decision: DiagnosticEpisodeDecision): DiagnosticEpisodeState {
    if (decision.scope !== state.scope || decision.context !== state.context
        || !Number.isSafeInteger(decision.order) || decision.order <= state.lastOrder) return state;
    const advanced = {...state, lastOrder: decision.order};
    // A rejected observation alone is neither a fault nor proof of invalidity.
    // The caller supplies a separate control decision when continuity is lost.
    if (decision.admission === 'rejected') return Object.freeze(advanced);
    if (decision.identity.name.length + decision.identity.hardware_id.length > DIAGNOSTIC_EPISODE_IDENTITY_LIMIT) {
        // A control cannot match any retained bounded identity or open an episode.
        if (decision.admission === 'control') return Object.freeze(advanced);
        return Object.freeze({...advanced, historyIncomplete: true,
            losses: Object.freeze({...state.losses, omitted: increment(state.losses.omitted), pending: true, version: decision.order})});
    }
    let index = -1;
    for (let i = state.episodes.length - 1; i >= 0; i--) {
        if (sameIdentity(state.episodes[i].identity, decision.identity)) { index = i; break; }
    }
    const previous = index < 0 ? undefined : state.episodes[index];
    if (decision.classification === 'fault' && (!previous || previous.state === 'resolved')) {
        // Insertion order is opening order. Protect every unresolved or unread
        // episode, including expired/unknown evidence; consultation is not OK.
        const evictionIndex = state.episodes.length === state.capacity
            ? state.episodes.findIndex(e => e.state === 'resolved' && !e.unconsulted) : -1;
        if (state.episodes.length === state.capacity && evictionIndex < 0) {
            return Object.freeze({...advanced, historyIncomplete: true,
                losses: Object.freeze({...state.losses, omitted: increment(state.losses.omitted), pending: true, version: decision.order})});
        }
        const losses = evictionIndex >= 0 ? Object.freeze({...state.losses,
            evicted: increment(state.losses.evicted), version: decision.order}) : state.losses;
        // No tombstone map grows with lost sources. Unknown predecessors stay
        // explicitly unknown, even after the loss information was consulted.
        const unknownHistory = decision.continuity === 'unknown' || !previous && state.historyIncomplete;
        const episode: DiagnosticEpisode = Object.freeze({id: decision.order,
            identity: Object.freeze({name: decision.identity.name, hardware_id: decision.identity.hardware_id}),
            state: 'active', continuity: unknownHistory ? 'unknown' : 'continuous',
            opening: unknownHistory ? 'unknown_history' : previous ? 'after_explicit_ok' : 'first_observed',
            lastFaultSeverity: decision.severity, peakSeverity: decision.severity, version: decision.order, unconsulted: true});
        const retained = evictionIndex >= 0 ? state.episodes.filter((_, i) => i !== evictionIndex) : state.episodes;
        return Object.freeze({...advanced, episodes: Object.freeze([...retained, episode]),
            losses, historyIncomplete: state.historyIncomplete || evictionIndex >= 0});
    }
    if (!previous || previous.state === 'resolved') return Object.freeze(advanced);
    const continuity = previous.continuity === 'unknown' || decision.continuity === 'unknown' ? 'unknown' : 'continuous';
    let next: DiagnosticEpisode;
    if (decision.classification === 'fault') {
        if (previous.state === 'active' && previous.lastFaultSeverity === decision.severity
            && previous.continuity === continuity) return Object.freeze(advanced);
        next = Object.freeze({...previous, state: 'active', continuity, lastFaultSeverity: decision.severity,
            peakSeverity: previous.peakSeverity === 2 || decision.severity === 2 ? 2 : 1,
            version: decision.order, unconsulted: previous.unconsulted || decision.severity > previous.peakSeverity});
    } else if (decision.classification === 'explicit_ok') {
        next = Object.freeze({...previous, state: 'resolved', continuity, version: decision.order, resolvedOrder: decision.order});
    } else {
        const nextState = decision.classification === 'expired' ? 'expired' : 'unknown';
        if (previous.state === nextState && previous.continuity === 'unknown') return Object.freeze(advanced);
        next = Object.freeze({...previous, state: nextState, continuity: 'unknown', version: decision.order});
    }
    const episodes = state.episodes.slice();
    episodes[index] = next;
    return Object.freeze({...advanced, episodes: Object.freeze(episodes)});
}

/** Only include complete episodes (peak included)/losses actually presented; reading is not consultation. */
export function readDiagnosticEpisodes(state: DiagnosticEpisodeState, options: {readonly ids?: readonly number[]; readonly includeLosses?: boolean} = {}) {
    if (options.ids && options.ids.length > state.capacity) throw new RangeError('Too many presented episode IDs');
    const episodes = options.ids ? Object.freeze(state.episodes.filter(e => options.ids?.includes(e.id))) : state.episodes;
    const losses = options.includeLosses === false ? undefined : state.losses;
    const receipt: DiagnosticEpisodeReceipt = Object.freeze({scope: state.scope, context: state.context,
        episodes: Object.freeze(episodes.map(e => Object.freeze({id: e.id, version: e.version}))),
        ...(losses ? {lossVersion: losses.version} : {})});
    return Object.freeze({episodes, losses, receipt, hasUnconsulted: state.losses.pending || state.episodes.some(e => e.unconsulted)});
}

/** Not an authentication API: the caller must pass the receipt for content actually presented. */
export function acknowledgeDiagnosticEpisodes(state: DiagnosticEpisodeState, receipt: DiagnosticEpisodeReceipt): DiagnosticEpisodeState {
    if (receipt.scope !== state.scope || receipt.context !== state.context || receipt.episodes.length > state.capacity) return state;
    let changed = false;
    const episodes = state.episodes.map(episode => {
        if (!episode.unconsulted || !receipt.episodes.some(e => e.id === episode.id && e.version === episode.version)) return episode;
        changed = true;
        return Object.freeze({...episode, unconsulted: false});
    });
    const losses = state.losses.pending && receipt.lossVersion === state.losses.version
        ? Object.freeze({...state.losses, pending: false}) : state.losses;
    if (!changed && losses === state.losses) return state;
    return Object.freeze({...state, episodes: changed ? Object.freeze(episodes) : state.episodes, losses});
}
