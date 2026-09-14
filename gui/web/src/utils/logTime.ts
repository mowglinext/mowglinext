/**
 * Log timestamp parsing and rendering.
 *
 * The container log views used to print whatever the producing process baked
 * into the line, and the producers disagree: ROS2 nodes emit the rcutils
 * default `[INFO] [<epoch>.<nanos>] [node]: msg`, the Go GUI emits three
 * different shapes from the same container (gin's access logger, an
 * unconfigured logrus, and stdlib `log`), and none of them carry a zone marker.
 * That is the "some UTC, some epoch" the operator sees.
 *
 * The docker daemon's own RFC3339Nano stamp (enabled by
 * `ContainerLogsOptions.Timestamps` in `gui/pkg/providers/docker.go`) is
 * prefixed to every line and takes precedence: it is always present and always
 * zone-marked, so whenever it is there it SUPERSEDES the
 * `CONTAINER_LOG_ASSUMED_TZ` guess the gin / Go-stdlib branches have to make.
 * The producer's own token is still stripped from the body, so the operator
 * never sees a raw epoch next to a rendered time.
 *
 * Everything here is pure — no React, no date library. `Intl.DateTimeFormat`
 * covers the whole job; neither dayjs nor date-fns is declared in
 * `package.json` (antd pulls dayjs in transitively, but importing an
 * undeclared transitive dependency is a trap).
 */

export type TimeZoneMode = 'local' | 'utc'

/** localStorage key holding the operator's zone choice. Browser-local, never a robot setting. */
export const LOG_TIME_ZONE_STORAGE_KEY = 'mowgli.log-timezone'

export const TIME_ZONE_MODES: readonly TimeZoneMode[] = ['local', 'utc']

export const DEFAULT_TIME_ZONE_MODE: TimeZoneMode = 'local'

export type LogTimestampSource =
    | 'docker'
    | 'ros'
    | 'logrus'
    | 'gin'
    | 'goStdlib'
    | 'received'

export interface ParsedLogTimestamp {
    /** Milliseconds since the epoch. Never NaN. */
    epochMs: number
    /** Which producer's format the timestamp came from, or 'received'. */
    source: LogTimestampSource
    /** The line with its (now redundant) timestamp token removed. */
    body: string
}

/** Rendered in place of a timestamp that cannot be resolved. */
export const TIMESTAMP_PLACEHOLDER = '—'

/**
 * gin and Go's stdlib `log` print a wall-clock time with NO zone marker. No
 * `TZ=` is set anywhere in `docker/` or `install/`, so those container clocks
 * run on UTC.
 *
 * This constant is the LABEL shown on the timezone badge. The parsing side is
 * `utcEpochMs()`, which hardcodes `Date.UTC` — the two encode the same
 * assumption in two places and MUST be changed together. If a `TZ=` is ever
 * added to a container, changing only this constant relabels the badge while
 * every gin / Go-stdlib line keeps being parsed as UTC, i.e. it renders the
 * wrong instant under a right-looking label. Fixing that properly needs a
 * zone-aware parse in `utcEpochMs`, not a new string here.
 *
 * Lines carrying the docker daemon's own RFC3339Nano prefix are unaffected —
 * that stamp is zone-marked and supersedes this guess.
 */
const CONTAINER_LOG_ASSUMED_TZ = 'UTC'

/**
 * Plausibility bounds for a bare ROS epoch, so `[123]` (a BT node id) and
 * `[9999999999999999.0]` are not mistaken for timestamps.
 */
const ROS_EPOCH_MIN_MS = Date.UTC(2020, 0, 1)
const ROS_EPOCH_MAX_MS = Date.UTC(2100, 0, 1)

/** `2026-05-12T22:02:33.123456789Z ` — what `docker logs --timestamps` prefixes. */
const DOCKER_PREFIX_PATTERN =
    /^(\d{4}-\d{2}-\d{2}T\d{2}:\d{2}:\d{2}(?:\.\d+)?(?:Z|[+-]\d{2}:?\d{2}))\s+/

/** `[1747087353.123456789]` — the rcutils default console format. */
const ROS_EPOCH_PATTERN = /\[(\d{10})\.(\d{1,9})\]\s*/

/** `time="2026-05-12T22:02:33Z"` — logrus' default TextFormatter. */
const LOGRUS_PATTERN = /\btime="([^"]+)"\s*/

/** `[GIN] 2026/05/12 - 22:02:33 | ` — gin.Default()'s access logger. */
const GIN_PATTERN = /^\[GIN\]\s+(\d{4})\/(\d{2})\/(\d{2}) - (\d{2}):(\d{2}):(\d{2})\s*\|\s*/

/** `2026/05/12 22:02:33 ` — Go stdlib `log` with default flags. */
const GO_STDLIB_PATTERN = /^(\d{4})\/(\d{2})\/(\d{2}) (\d{2}):(\d{2}):(\d{2})(?:\.\d+)?\s+/

const NANOS_DIGITS = 9
const MILLIS_DIGITS = 3

function isPlausibleEpochMs(epochMs: number): boolean {
    return Number.isFinite(epochMs) && epochMs >= ROS_EPOCH_MIN_MS && epochMs <= ROS_EPOCH_MAX_MS
}

/** Turn a fractional-seconds digit string of any length into whole milliseconds. */
function fractionToMillis(fraction: string): number {
    return Number(fraction.padEnd(NANOS_DIGITS, '0').slice(0, MILLIS_DIGITS))
}

// Parses a zone-less container wall clock as UTC. Paired with
// CONTAINER_LOG_ASSUMED_TZ — see the note there before changing either.
function utcEpochMs(parts: readonly string[]): number {
    const [year, month, day, hour, minute, second] = parts.map(Number)
    return Date.UTC(year, month - 1, day, hour, minute, second)
}

function parseIsoLike(value: string): number | undefined {
    const epochMs = Date.parse(value)
    return Number.isNaN(epochMs) ? undefined : epochMs
}

/**
 * Extract a producer-baked timestamp from one log line, ignoring any docker
 * prefix (which `parseLogTimestamp` peels off first).
 *
 * Precedence runs most-specific first, and the matched token is removed from
 * the body so the operator never sees the time twice.
 */
function parseProducerTimestamp(line: string): ParsedLogTimestamp | undefined {
    const rosMatch = ROS_EPOCH_PATTERN.exec(line)
    if (rosMatch) {
        const epochMs = Number(rosMatch[1]) * 1000 + fractionToMillis(rosMatch[2])
        if (isPlausibleEpochMs(epochMs)) {
            return {epochMs, source: 'ros', body: line.replace(ROS_EPOCH_PATTERN, '')}
        }
    }

    const logrusMatch = LOGRUS_PATTERN.exec(line)
    if (logrusMatch) {
        const epochMs = parseIsoLike(logrusMatch[1])
        if (epochMs !== undefined) {
            return {epochMs, source: 'logrus', body: line.replace(LOGRUS_PATTERN, '')}
        }
    }

    const ginMatch = GIN_PATTERN.exec(line)
    if (ginMatch) {
        return {
            epochMs: utcEpochMs(ginMatch.slice(1, 7)),
            source: 'gin',
            body: `[GIN] ${line.slice(ginMatch[0].length)}`,
        }
    }

    const goMatch = GO_STDLIB_PATTERN.exec(line)
    if (goMatch) {
        return {
            epochMs: utcEpochMs(goMatch.slice(1, 7)),
            source: 'goStdlib',
            body: line.slice(goMatch[0].length),
        }
    }

    return undefined
}

/**
 * Resolve the timestamp of one log line.
 *
 * The docker prefix is peeled off FIRST and, when it parses, supplies the
 * instant — it is the only stamp that is always present and always zone-marked.
 * The producer's own token is still parsed out of the remainder so it is
 * stripped from the body: returning early on the docker match alone would leave
 * a raw `[1747087353.123456789]` epoch rendered next to the formatted column,
 * which is exactly what issue #207 removed.
 *
 * Call this ONCE at ingest and store the result — never inside a render or a
 * filter, which would re-parse the whole 5000-line buffer on every frame.
 *
 * @param line       the raw (already ANSI-stripped) log line
 * @param receivedAtMs when the browser received it, used when the line carries
 *                   no timestamp of its own
 */
export function parseLogTimestamp(line: string, receivedAtMs: number): ParsedLogTimestamp {
    const dockerMatch = DOCKER_PREFIX_PATTERN.exec(line)
    if (dockerMatch) {
        const rest = line.slice(dockerMatch[0].length)
        const inner = parseProducerTimestamp(rest)
        const dockerEpochMs = parseIsoLike(dockerMatch[1])
        if (dockerEpochMs !== undefined) {
            return {epochMs: dockerEpochMs, source: 'docker', body: inner?.body ?? rest}
        }
        if (inner) {
            return inner
        }
    }

    return parseProducerTimestamp(line) ?? {epochMs: receivedAtMs, source: 'received', body: line}
}

export interface FormatLogTimestampOptions {
    /** Append `.mmm`. Off by default — the log column is already wide. */
    withMillis?: boolean
    /** Override the zone. Only used by tests; production passes the mode. */
    timeZone?: string
    /** Drop the date, leaving `HH:mm:ss`. Used on narrow (mobile) layouts. */
    timeOnly?: boolean
}

type DateParts = Record<string, string>

/**
 * Constructing an `Intl.DateTimeFormat` is the expensive half of formatting:
 * ~22 us each, against ~2 us for a `formatToParts` on an existing one. The log
 * column formats every visible line twice (title + body) and the search filter
 * formats the whole buffer, so a fresh formatter per call cost ~110 ms per 5000
 * lines. Keyed on the zone, of which there are two in practice ('UTC' and the
 * browser default), so the map needs no eviction.
 */
const formatterCache = new Map<string, Intl.DateTimeFormat>()

function formatterFor(timeZone: string | undefined): Intl.DateTimeFormat {
    const key = timeZone ?? 'local'
    const cached = formatterCache.get(key)
    if (cached !== undefined) {
        return cached
    }
    const formatter = new Intl.DateTimeFormat('en-GB', {
        ...(timeZone === undefined ? {} : {timeZone}),
        year: 'numeric',
        month: '2-digit',
        day: '2-digit',
        hour: '2-digit',
        minute: '2-digit',
        second: '2-digit',
        hour12: false,
    })
    formatterCache.set(key, formatter)
    return formatter
}

function partsFor(date: Date, timeZone: string | undefined): DateParts {
    const formatter = formatterFor(timeZone)
    const parts: DateParts = {}
    for (const part of formatter.formatToParts(date)) {
        parts[part.type] = part.value
    }
    return parts
}

/**
 * Render an instant as `YYYY-MM-DDTHH:mm:ss` — a fixed, locale-independent
 * shape assembled from `formatToParts`. Deliberately NOT `toLocaleString`,
 * whose output shape changes with the browser locale.
 */
export function formatLogTimestamp(
    epochMs: number,
    mode: TimeZoneMode,
    options: FormatLogTimestampOptions = {},
): string {
    if (!Number.isFinite(epochMs)) {
        return TIMESTAMP_PLACEHOLDER
    }
    const date = new Date(epochMs)
    const timeZone = options.timeZone ?? (mode === 'utc' ? 'UTC' : undefined)
    const parts = partsFor(date, timeZone)
    // `hour12: false` renders midnight as "24" in some ICU builds.
    const hour = parts.hour === '24' ? '00' : parts.hour
    const clock = `${hour}:${parts.minute}:${parts.second}`
    const millis = options.withMillis
        ? `.${String(date.getUTCMilliseconds()).padStart(MILLIS_DIGITS, '0')}`
        : ''
    if (options.timeOnly) {
        return `${clock}${millis}`
    }
    return `${parts.year}-${parts.month}-${parts.day}T${clock}${millis}`
}

/** Noon UTC is the safest anchor for a civil date: no DST transition lands on it. */
const DAY_ANCHOR_HOUR_UTC = 12

/**
 * Collapse an instant to the CIVIL DAY it falls on in the selected zone,
 * returned as the epoch ms of that day at noon UTC.
 *
 * Calendar views (the year-of-lawn grid) bucket sessions by day, and a session
 * logged at 23:30 UTC belongs to a different day depending on the zone the
 * operator is reading in. Bucketing on the browser's zone while the rest of the
 * page renders in UTC puts a row in a bucket its own displayed date contradicts.
 *
 * The anchor is a NUMBER so it doubles as the bucket key, and stepping it by
 * whole days (`+ 86_400_000`) walks consecutive civil dates without ever
 * drifting across a DST boundary. Read the result back with the `getUTC*`
 * accessors, or format it with `timeZone: 'UTC'`.
 */
export function zonedDayAnchorMs(epochMs: number, mode: TimeZoneMode): number {
    if (!Number.isFinite(epochMs)) {
        return NaN
    }
    const parts = partsFor(new Date(epochMs), mode === 'utc' ? 'UTC' : undefined)
    return Date.UTC(
        Number(parts.year),
        Number(parts.month) - 1,
        Number(parts.day),
        DAY_ANCHOR_HOUR_UTC,
    )
}

/**
 * Render one of the backend's structured `*_at` fields. They are RFC3339 UTC
 * strings today (see `gui/pkg/api/*.go`), but epoch milliseconds and `Date`
 * are accepted so every call site can share one helper.
 */
export function formatAbsoluteTimestamp(
    value: string | number | Date | null | undefined,
    mode: TimeZoneMode,
    options: FormatLogTimestampOptions = {},
): string {
    if (value === null || value === undefined || value === '') {
        return TIMESTAMP_PLACEHOLDER
    }
    const epochMs = value instanceof Date
        ? value.getTime()
        : typeof value === 'number'
            ? value
            : Date.parse(value)
    if (!Number.isFinite(epochMs)) {
        return TIMESTAMP_PLACEHOLDER
    }
    return formatLogTimestamp(epochMs, mode, options)
}

/** Short label for the timezone badge: 'UTC', or the browser's IANA zone name. */
export function resolvedZoneLabel(mode: TimeZoneMode): string {
    if (mode === 'utc') {
        return CONTAINER_LOG_ASSUMED_TZ
    }
    return Intl.DateTimeFormat().resolvedOptions().timeZone
}
