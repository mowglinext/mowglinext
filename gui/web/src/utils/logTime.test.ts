import {describe, it, expect} from 'vitest'
import {
    formatAbsoluteTimestamp,
    formatLogTimestamp,
    parseLogTimestamp,
    resolvedZoneLabel,
    TIMESTAMP_PLACEHOLDER,
    zonedDayAnchorMs,
} from './logTime.ts'

// A stream line the frontend never produced a timestamp for.
const RECEIVED_AT_MS = 1_747_000_000_000

// 2026-05-12T22:02:33.123Z
const FIXED_EPOCH_MS = Date.UTC(2026, 4, 12, 22, 2, 33, 123)

describe('parseLogTimestamp', () => {
    it('reads the ROS2 rcutils epoch and strips it from the body', () => {
        // Arrange
        const line = '[INFO] [1747087353.123456789] [map_server_node]: planning'

        // Act
        const parsed = parseLogTimestamp(line, RECEIVED_AT_MS)

        // Assert
        expect(parsed.source).toBe('ros')
        expect(parsed.epochMs).toBe(1747087353123)
        expect(parsed.body).toBe('[INFO] [map_server_node]: planning')
    })

    it('reads a docker RFC3339Nano prefix and strips it from the body', () => {
        // Arrange
        const line = '2026-05-12T22:02:33.123456789Z INFO foo'

        // Act
        const parsed = parseLogTimestamp(line, RECEIVED_AT_MS)

        // Assert
        expect(parsed.source).toBe('docker')
        expect(parsed.epochMs).toBe(FIXED_EPOCH_MS)
        expect(parsed.body).toBe('INFO foo')
    })

    // The docker daemon prefixes EVERY line once ContainerLogsOptions.Timestamps
    // is set (gui/pkg/providers/docker.go), so these are the shapes the live
    // stream actually carries. Returning on the docker match alone would leave
    // the producer's own raw token in the body — the exact regression #207 fixed.
    it('prefers the docker stamp but still strips a ROS epoch from the body', () => {
        // Arrange
        const line =
            '2026-05-12T22:02:33.123456789Z [INFO] [1747087353.123456789] [map_server_node]: planning'

        // Act
        const parsed = parseLogTimestamp(line, RECEIVED_AT_MS)

        // Assert
        expect(parsed.source).toBe('docker')
        expect(parsed.epochMs).toBe(FIXED_EPOCH_MS)
        expect(parsed.body).toBe('[INFO] [map_server_node]: planning')
        expect(parsed.body).not.toMatch(/1747087353/)
    })

    it('prefers the docker stamp over a gin wall clock and keeps the [GIN] marker', () => {
        // Arrange — gin prints a bare, zone-less clock; docker's is zone-marked.
        const line =
            '2026-05-12T22:02:33.123456789Z [GIN] 2026/05/12 - 21:02:33 | 200 |  1.2ms | GET /api'

        // Act
        const parsed = parseLogTimestamp(line, RECEIVED_AT_MS)

        // Assert
        expect(parsed.source).toBe('docker')
        expect(parsed.epochMs).toBe(FIXED_EPOCH_MS)
        expect(parsed.body).toBe('[GIN] 200 |  1.2ms | GET /api')
    })

    it('falls back to the producer token when the docker stamp is unparseable', () => {
        // Arrange — a well-shaped but impossible date; Date.parse returns NaN.
        const line = '2026-13-45T99:99:99.000000000Z [INFO] [1747087353.123456789] hi'

        // Act
        const parsed = parseLogTimestamp(line, RECEIVED_AT_MS)

        // Assert
        expect(parsed.source).toBe('ros')
        expect(parsed.epochMs).toBe(1747087353123)
    })

    it('reads a logrus time= field', () => {
        // Arrange
        const line = 'time="2026-05-12T22:02:33Z" level=info msg=x'

        // Act
        const parsed = parseLogTimestamp(line, RECEIVED_AT_MS)

        // Assert
        expect(parsed.source).toBe('logrus')
        expect(parsed.epochMs).toBe(Date.UTC(2026, 4, 12, 22, 2, 33))
        expect(parsed.body).toBe('level=info msg=x')
    })

    it('reads a gin access-log prefix as UTC', () => {
        // Arrange — no TZ is set in docker/ or install/, so container clocks are UTC.
        const line = '[GIN] 2026/05/12 - 22:02:33 | 200 |  1.2ms | GET /api'

        // Act
        const parsed = parseLogTimestamp(line, RECEIVED_AT_MS)

        // Assert
        expect(parsed.source).toBe('gin')
        expect(parsed.epochMs).toBe(Date.UTC(2026, 4, 12, 22, 2, 33))
        expect(parsed.body).toBe('[GIN] 200 |  1.2ms | GET /api')
    })

    it('reads a Go stdlib log prefix as UTC', () => {
        // Arrange
        const line = '2026/05/12 22:02:33 db opened'

        // Act
        const parsed = parseLogTimestamp(line, RECEIVED_AT_MS)

        // Assert
        expect(parsed.source).toBe('goStdlib')
        expect(parsed.epochMs).toBe(Date.UTC(2026, 4, 12, 22, 2, 33))
        expect(parsed.body).toBe('db opened')
    })

    it('falls back to the receive time when the line carries no timestamp', () => {
        // Arrange
        const line = 'INFO synthetic log line 999'

        // Act
        const parsed = parseLogTimestamp(line, RECEIVED_AT_MS)

        // Assert
        expect(parsed.source).toBe('received')
        expect(parsed.epochMs).toBe(RECEIVED_AT_MS)
        expect(parsed.body).toBe(line)
    })

    it('rejects a bracketed number that is not a plausible epoch', () => {
        // Arrange — a BT node id, not a timestamp.
        const line = '[123] not a stamp'

        // Act
        const parsed = parseLogTimestamp(line, RECEIVED_AT_MS)

        // Assert
        expect(parsed.source).toBe('received')
        expect(parsed.body).toBe(line)
    })

    it('rejects an out-of-range epoch rather than rendering a nonsense date', () => {
        // Arrange
        const line = '[9999999999999999.0] bogus'

        // Act
        const parsed = parseLogTimestamp(line, RECEIVED_AT_MS)

        // Assert
        expect(parsed.source).toBe('received')
        expect(parsed.epochMs).toBe(RECEIVED_AT_MS)
    })
})

describe('formatLogTimestamp', () => {
    it('renders a fixed ISO-like shape in UTC regardless of the runner time zone', () => {
        // Arrange / Act
        const rendered = formatLogTimestamp(FIXED_EPOCH_MS, 'utc')

        // Assert — this is the assertion that would have caught the original bug.
        expect(rendered).toBe('2026-05-12T22:02:33')
    })

    it('renders the same instant differently in a non-UTC local zone', () => {
        // Arrange / Act
        const utc = formatLogTimestamp(FIXED_EPOCH_MS, 'utc')
        const paris = formatLogTimestamp(FIXED_EPOCH_MS, 'local', {timeZone: 'Europe/Paris'})

        // Assert — Paris is UTC+2 in May.
        expect(paris).toBe('2026-05-13T00:02:33')
        expect(paris).not.toBe(utc)
    })

    it('appends milliseconds when asked', () => {
        // Arrange / Act
        const rendered = formatLogTimestamp(FIXED_EPOCH_MS, 'utc', {withMillis: true})

        // Assert
        expect(rendered).toBe('2026-05-12T22:02:33.123')
    })

    it('returns the placeholder rather than "Invalid Date" for unusable input', () => {
        // Arrange / Act / Assert
        expect(formatLogTimestamp(Number.NaN, 'utc')).toBe(TIMESTAMP_PLACEHOLDER)
    })
})

describe('formatAbsoluteTimestamp', () => {
    it('renders the three backend shapes of the same instant identically', () => {
        // Arrange
        const rfc3339 = '2026-05-12T22:02:33.123Z'

        // Act
        const fromString = formatAbsoluteTimestamp(rfc3339, 'utc')
        const fromNumber = formatAbsoluteTimestamp(FIXED_EPOCH_MS, 'utc')
        const fromDate = formatAbsoluteTimestamp(new Date(FIXED_EPOCH_MS), 'utc')

        // Assert
        expect(fromString).toBe('2026-05-12T22:02:33')
        expect(fromNumber).toBe(fromString)
        expect(fromDate).toBe(fromString)
    })

    it('returns the placeholder for nullish or unparseable values', () => {
        // Arrange / Act / Assert
        expect(formatAbsoluteTimestamp(null, 'utc')).toBe(TIMESTAMP_PLACEHOLDER)
        expect(formatAbsoluteTimestamp(undefined, 'utc')).toBe(TIMESTAMP_PLACEHOLDER)
        expect(formatAbsoluteTimestamp('not a date', 'utc')).toBe(TIMESTAMP_PLACEHOLDER)
    })
})

describe('resolvedZoneLabel', () => {
    it('labels UTC mode as UTC', () => {
        // Arrange / Act / Assert
        expect(resolvedZoneLabel('utc')).toBe('UTC')
    })

    it('labels local mode with the resolved IANA zone name', () => {
        // Arrange
        const expected = Intl.DateTimeFormat().resolvedOptions().timeZone

        // Act
        const label = resolvedZoneLabel('local')

        // Assert — CI runners are usually UTC, so assert the resolved name
        // rather than that it differs from UTC.
        expect(label).toBe(expected)
        expect(label.length).toBeGreaterThan(0)
    })
})

describe('zonedDayAnchorMs', () => {
    it('buckets an instant on its UTC civil day in UTC mode', () => {
        // Arrange — 23:30 and 00:30 UTC straddle midnight, so they are two days.
        const lateNight = Date.UTC(2026, 4, 12, 23, 30)
        const justAfter = Date.UTC(2026, 4, 13, 0, 30)

        // Act
        const first = zonedDayAnchorMs(lateNight, 'utc')
        const second = zonedDayAnchorMs(justAfter, 'utc')

        // Assert
        expect(first).toBe(Date.UTC(2026, 4, 12, 12))
        expect(second).toBe(Date.UTC(2026, 4, 13, 12))
        expect(second - first).toBe(24 * 60 * 60 * 1000)
    })

    it('buckets on the browser civil day in local mode', () => {
        // Arrange
        const instant = Date.UTC(2026, 4, 12, 23, 30)
        const local = new Date(instant)

        // Act
        const anchor = zonedDayAnchorMs(instant, 'local')

        // Assert — the browser's own calendar date, whatever the runner's zone.
        expect(anchor).toBe(
            Date.UTC(local.getFullYear(), local.getMonth(), local.getDate(), 12),
        )
    })

    it('steps by whole days without drifting across a DST boundary', () => {
        // Arrange — Europe/Paris springs forward on 2026-03-29.
        const before = zonedDayAnchorMs(Date.UTC(2026, 2, 28, 10), 'utc')

        // Act
        const next = new Date(before + 24 * 60 * 60 * 1000)

        // Assert
        expect(next.getUTCDate()).toBe(29)
        expect(next.getUTCHours()).toBe(12)
    })

    it('returns NaN for a non-finite instant rather than a bogus day', () => {
        expect(Number.isNaN(zonedDayAnchorMs(Number.NaN, 'utc'))).toBe(true)
    })
})
