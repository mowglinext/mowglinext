import {createContext, useCallback, useContext, useEffect, useMemo, useState} from "react";
import {
    DEFAULT_TIME_ZONE_MODE,
    formatAbsoluteTimestamp,
    formatLogTimestamp,
    LOG_TIME_ZONE_STORAGE_KEY,
    resolvedZoneLabel,
    TIME_ZONE_MODES,
    type FormatLogTimestampOptions,
    type TimeZoneMode,
} from "../utils/logTime.ts";

/**
 * How log and `*_at` timestamps are rendered across the app.
 *
 * This is a BROWSER-LOCAL display preference, so it lives in localStorage next
 * to the display-mode preference — deliberately NOT a robot parameter. It is
 * per-operator-device, not per-robot, and adding it to `mowgli_robot.yaml`
 * would trip the sparse-config drift check for no benefit.
 *
 * Kept out of ThemeContext on purpose: that context owns colours and motion.
 */

function isTimeZoneMode(value: string | null): value is TimeZoneMode {
    return value !== null && (TIME_ZONE_MODES as readonly string[]).includes(value);
}

function readTimeZoneMode(): TimeZoneMode {
    try {
        const stored = window.localStorage.getItem(LOG_TIME_ZONE_STORAGE_KEY);
        return isTimeZoneMode(stored) ? stored : DEFAULT_TIME_ZONE_MODE;
    } catch {
        // Private browsing or a locked-down WebView can reject storage.
        return DEFAULT_TIME_ZONE_MODE;
    }
}

interface TimeFormatContextValue {
    timeZoneMode: TimeZoneMode;
    setTimeZoneMode: (mode: TimeZoneMode) => void;
    /** Short label for the badge: 'UTC' or the browser's IANA zone name. */
    zoneLabel: string;
    /** Render a log-line epoch in the current zone. */
    formatLogTime: (epochMs: number, options?: FormatLogTimestampOptions) => string;
    /** Render a backend `*_at` value (RFC3339 / epoch ms / Date) in the current zone. */
    formatAbsolute: (
        value: string | number | Date | null | undefined,
        options?: FormatLogTimestampOptions,
    ) => string;
}

const TimeFormatContext = createContext<TimeFormatContextValue>({
    timeZoneMode: DEFAULT_TIME_ZONE_MODE,
    setTimeZoneMode: () => { /* no-op outside the provider */ },
    zoneLabel: resolvedZoneLabel(DEFAULT_TIME_ZONE_MODE),
    formatLogTime: (epochMs, options) => formatLogTimestamp(epochMs, DEFAULT_TIME_ZONE_MODE, options),
    formatAbsolute: (value, options) => formatAbsoluteTimestamp(value, DEFAULT_TIME_ZONE_MODE, options),
});

export function TimeFormatProvider({children}: {children: React.ReactNode}) {
    const [timeZoneMode, setTimeZoneMode] = useState<TimeZoneMode>(readTimeZoneMode);

    useEffect(() => {
        try {
            window.localStorage.setItem(LOG_TIME_ZONE_STORAGE_KEY, timeZoneMode);
        } catch {
            // Storage rejected — the choice still applies for this session.
        }
    }, [timeZoneMode]);

    const formatLogTime = useCallback(
        (epochMs: number, options?: FormatLogTimestampOptions) =>
            formatLogTimestamp(epochMs, timeZoneMode, options),
        [timeZoneMode],
    );

    const formatAbsolute = useCallback(
        (value: string | number | Date | null | undefined, options?: FormatLogTimestampOptions) =>
            formatAbsoluteTimestamp(value, timeZoneMode, options),
        [timeZoneMode],
    );

    const value = useMemo<TimeFormatContextValue>(() => ({
        timeZoneMode,
        setTimeZoneMode,
        zoneLabel: resolvedZoneLabel(timeZoneMode),
        formatLogTime,
        formatAbsolute,
    }), [timeZoneMode, formatLogTime, formatAbsolute]);

    return (
        <TimeFormatContext.Provider value={value}>
            {children}
        </TimeFormatContext.Provider>
    );
}

export function useTimeFormat(): TimeFormatContextValue {
    return useContext(TimeFormatContext);
}
