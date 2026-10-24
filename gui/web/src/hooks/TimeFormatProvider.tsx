import {useCallback, useEffect, useMemo, useState} from "react";
import {TimeFormatContext, readTimeZoneMode, type TimeFormatContextValue} from './useTimeFormat';
import {formatAbsoluteTimestamp, formatLogTimestamp, LOG_TIME_ZONE_STORAGE_KEY, resolvedZoneLabel, type FormatLogTimestampOptions, type TimeZoneMode} from '../utils/logTime';

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
