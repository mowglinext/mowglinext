import type {DiagnosticStatus, TimestampedDiagnosticStatus} from '../hooks/useDiagnostics';

export const DIAGNOSTIC_TRANSITION_CAPACITY = 64;
// Identity is retained verbatim or omitted explicitly, never truncated into an
// ambiguous identifier. This also bounds unexpected long producer strings.
export const DIAGNOSTIC_TRANSITION_IDENTITY_LIMIT = 4096;

/** A change reported by a producer, not a certified physical state/resolution. */
export interface DiagnosticTransition {
    readonly order: number;
    readonly name: string;
    readonly hardware_id: string;
    readonly previousLevel: number | null;
    readonly level: number;
    readonly publicationStamp: string | null;
    readonly healthBlocked: boolean;
}

/** Local RAM scope only: it does not identify or authorize a ROS producer. */
export interface DiagnosticTransitionCursor {
    readonly scope: symbol;
    readonly order: number;
}

export interface DiagnosticTransitionRead {
    readonly active: boolean;
    readonly context: string | null;
    readonly contextChanged: boolean;
    readonly cursor: DiagnosticTransitionCursor;
    readonly transitions: readonly DiagnosticTransition[];
    /** Missing changes after the supplied cursor, within the current scope. */
    readonly missed: number;
    readonly evicted: number;
    readonly omittedOversizedIdentity: number;
}

/**
 * One bounded, nonvisual memory owned by one committed hook context. No timers,
 * subscriptions, notifications, persistence or React state updates. Unchanged
 * levels (including new stamps/measurement values) allocate no transition.
 */
export class DiagnosticTransitionMemory {
    private context: string | null = null;
    private scope = Symbol();
    private order = 0;
    private evicted = 0;
    private omittedOversizedIdentity = 0;
    private readonly records = new Array<DiagnosticTransition | undefined>(DIAGNOSTIC_TRANSITION_CAPACITY);
    private head = 0;
    private count = 0;
    // Snapshot admission must remain unchanged when context changes. Weak
    // references exclude its old entries as predecessors without retaining the
    // DiagnosticStatus payloads or adding a per-message comparison map.
    private previousContextEntries = new WeakSet<DiagnosticStatus>();

    begin(context: string, previousEntries: Iterable<DiagnosticStatus>) {
        this.clear();
        this.context = context;
        this.previousContextEntries = new WeakSet(previousEntries);
    }

    clear() {
        this.context = null;
        this.scope = Symbol();
        this.order = 0;
        this.evicted = 0;
        this.omittedOversizedIdentity = 0;
        this.head = 0;
        this.count = 0;
        this.records.fill(undefined);
        this.previousContextEntries = new WeakSet();
    }

    /** A local disconnect copy is not a new producer observation. */
    trackSnapshotCopy(previous: DiagnosticStatus, next: DiagnosticStatus) {
        if (this.previousContextEntries.has(previous)) this.previousContextEntries.add(next);
    }

    observe(previous: DiagnosticStatus | undefined, next: TimestampedDiagnosticStatus) {
        if (this.context === null) return;
        const sameSource = previous !== undefined && previous.name === next.name
            && previous.hardware_id === next.hardware_id && !this.previousContextEntries.has(previous);
        let previousLevel = sameSource ? previous.level : null;
        // The current snapshot is keyed by name. If a different hardware ID
        // replaced that name, use only a retained observation of this exact
        // pair, never the other producer's level. Bounded by 64 records.
        if (previousLevel === null) {
            for (let i = this.count - 1; i >= 0; i--) {
                const record = this.records[(this.head + i) % this.records.length];
                if (record?.name === next.name && record.hardware_id === next.hardware_id) {
                    previousLevel = record.level;
                    break;
                }
            }
        }
        // First OK initializes the existing snapshot, not an alert episode.
        // Text/value changes at an unchanged level remain in that snapshot.
        if (previousLevel === next.level || previousLevel === null && next.level === 0) return;
        const order = ++this.order;
        if (next.name.length + next.hardware_id.length > DIAGNOSTIC_TRANSITION_IDENTITY_LIMIT) {
            this.omittedOversizedIdentity++;
            return;
        }
        const transition: DiagnosticTransition = Object.freeze({
            order, name: next.name, hardware_id: next.hardware_id,
            previousLevel, level: next.level, publicationStamp: next.publicationStamp ?? null,
            healthBlocked: next.healthBlocked === true,
        });
        if (this.count === this.records.length) {
            this.records[this.head] = transition;
            this.head = (this.head + 1) % this.records.length;
            this.evicted++;
        } else {
            this.records[(this.head + this.count) % this.records.length] = transition;
            this.count++;
        }
    }

    /** Non-destructive read; pass the returned cursor to avoid reading twice. */
    readAfter(cursor?: DiagnosticTransitionCursor): DiagnosticTransitionRead {
        const contextChanged = cursor !== undefined && (cursor.scope !== this.scope
            || !Number.isSafeInteger(cursor.order) || cursor.order < 0 || cursor.order > this.order);
        const after = cursor === undefined || contextChanged ? 0 : cursor.order;
        const transitions: DiagnosticTransition[] = [];
        for (let i = 0; i < this.count; i++) {
            const record = this.records[(this.head + i) % this.records.length];
            if (record && record.order > after) transitions.push(record);
        }
        return {active: this.context !== null, context: this.context, contextChanged,
            cursor: {scope: this.scope, order: this.order}, transitions,
            missed: this.order - after - transitions.length,
            evicted: this.evicted, omittedOversizedIdentity: this.omittedOversizedIdentity};
    }
}
