import {App, Input, Select, Space} from "antd";
import {useEffect, useMemo, useRef, useState} from "react";
import {useTranslation} from "react-i18next";
import AsyncButton from "../components/AsyncButton.tsx";
import {useWS} from "../hooks/useWS.ts";
import {useApi} from "../hooks/useApi.ts";
import {useThemeMode} from "../theme/ThemeContext.tsx";
import {useIsMobile} from "../hooks/useIsMobile";
import {appendCappedBatch, createLogBatcher, type LogBatcher} from "./logBatcher.ts";
import {useTimeFormat} from "../hooks/useTimeFormat.tsx";
import {parseLogTimestamp, type LogTimestampSource} from "../utils/logTime.ts";

type Severity = 'ERROR' | 'WARN' | 'INFO' | 'DEBUG' | 'OTHER';

const LEVEL_PATTERN = /\b(ERROR|ERR|FATAL|CRITICAL|WARN(?:ING)?|INFO|DEBUG|TRACE)\b/i;
// ESC is a control character by definition -- an ANSI escape matcher cannot
// be written without it. Silenced at the one site that needs it rather than
// globally, so a genuine stray control character elsewhere still reports.
// eslint-disable-next-line no-control-regex
const ANSI_REGEX = /\x1b\[[0-9;]*m/g;

function detectSeverity(line: string): Severity {
    const m = LEVEL_PATTERN.exec(line);
    if (!m) return 'OTHER';
    const tok = m[1].toUpperCase();
    if (tok === 'ERROR' || tok === 'ERR' || tok === 'FATAL' || tok === 'CRITICAL') return 'ERROR';
    if (tok === 'WARN' || tok === 'WARNING') return 'WARN';
    if (tok === 'INFO') return 'INFO';
    if (tok === 'DEBUG' || tok === 'TRACE') return 'DEBUG';
    return 'OTHER';
}

interface ParsedLog {
    id: number;
    /** The line with its producer-baked timestamp removed. */
    plain: string;
    severity: Severity;
    /** Epoch ms, parsed once at ingest — never re-parsed during render. */
    tsMs: number;
    tsSource: LogTimestampSource;
}

const LEVEL_OPTIONS: { value: Severity; label: string }[] = [
    {value: 'ERROR', label: 'logsPage.levelErrors'},
    {value: 'WARN', label: 'logsPage.levelWarnings'},
    {value: 'INFO', label: 'logsPage.levelInfo'},
    {value: 'DEBUG', label: 'logsPage.levelDebug'},
    {value: 'OTHER', label: 'logsPage.levelOther'},
];

const DEFAULT_LEVELS: Severity[] = ['ERROR', 'WARN', 'INFO', 'OTHER'];
const MAX_LINES = 5000;
const LOG_BATCH_INTERVAL_MS = 100;
// Width of the monospace timestamp column: "YYYY-MM-DDTHH:mm:ss" is 19 chars,
// "HH:mm:ss" is 8. Fixed so the bodies line up.
const TIMESTAMP_COLUMN_CH = 19;
const TIMESTAMP_COLUMN_MOBILE_CH = 8;
// A query with no digit in it cannot match a timestamp, so it never needs to
// pay for formatting the buffer. Guards the search-the-time-column path below.
const QUERY_HAS_DIGIT = /\d/;

type ContainerList = { value: string, label: string, status: "started" | "stopped", labels: Record<string, string> };

export const LogsPage = () => {
    const {t} = useTranslation();
    const {colors} = useThemeMode();
    const {timeZoneMode, setTimeZoneMode, zoneLabel, formatLogTime} = useTimeFormat();
    const guiApi = useApi();
    const {notification} = App.useApp();
    const isMobile = useIsMobile();
    const [containers, setContainers] = useState<ContainerList[]>([]);
    const [containerId, setContainerId] = useState<string | undefined>(undefined);
    const [logs, setLogs] = useState<ParsedLog[]>([]);
    const [levels, setLevels] = useState<Severity[]>(DEFAULT_LEVELS);
    const [search, setSearch] = useState('');
    const [autoScroll, setAutoScroll] = useState(true);
    const nextIdRef = useRef(0);
    const listRef = useRef<HTMLDivElement | null>(null);
    const batcherRef = useRef<LogBatcher<ParsedLog> | null>(null);
    if (batcherRef.current === null) {
        batcherRef.current = createLogBatcher<ParsedLog>((batch) => {
            setLogs(prev => appendCappedBatch(prev, batch, MAX_LINES));
        }, LOG_BATCH_INTERVAL_MS);
    }

    const resetLogs = () => {
        batcherRef.current?.reset();
        nextIdRef.current = 0;
        setLogs([]);
    };
    // useWS.onClose fires for BOTH server-side drops and our own stop()/container
    // switches. Flag the deliberate ones so we don't surface an error toast for
    // an intentional stop or a container change.
    const intentionalStopRef = useRef(false);

    const stream = useWS<string>(
        () => {
            if (intentionalStopRef.current) {
                intentionalStopRef.current = false;
                return;
            }
            notification.error({message: t('logsPage.streamClosed')});
        },
        () => { /* connected */ },
        (line, first) => {
            if (first) resetLogs();
            const stripped = line.replace(ANSI_REGEX, '');
            // Parse ONCE here, at ingest. Doing it in the render map or the
            // `filtered` memo would re-parse the whole 5000-line buffer on
            // every frame, on the very path logBatcher.bench.ts exists to
            // protect.
            const {epochMs, source, body} = parseLogTimestamp(stripped, Date.now());
            batcherRef.current?.push({
                id: nextIdRef.current++,
                plain: body,
                severity: detectSeverity(body),
                tsMs: epochMs,
                tsSource: source,
            });
        });

    async function listContainers() {
        try {
            const res = await guiApi.containers.containersList();
            if (res.error) throw new Error(res.error.error);
            const options = res.data.containers?.flatMap<ContainerList>((c) => {
                if (!c.names || !c.id) return [];
                const name = c.names[0].replace("/", "");
                return [{
                    label: c.labels?.app ? `${c.labels.app} (${name})` : name,
                    value: c.id,
                    status: c.state === "running" ? "started" : "stopped",
                    labels: c.labels ?? {},
                }];
            });
            setContainers(options ?? []);
            if (options?.length && !containerId) setContainerId(options[0].value);
        } catch (e: unknown) {
            notification.error({
                message: t('logsPage.failedToListContainers'),
                description: e instanceof Error ? e.message : String(e),
            });
        }
    }

    useEffect(() => { listContainers(); }, []);

    useEffect(() => {
        if (!containerId) return;
        resetLogs();
        stream.start(`/api/containers/${containerId}/logs`);
        // Switching containers (or unmounting) closes the socket on purpose.
        return () => { intentionalStopRef.current = true; stream?.stop(); };
    }, [containerId]);

    useEffect(() => () => batcherRef.current?.reset(), []);

    const commandContainer = (command: "start" | "stop" | "restart") => async () => {
        const messages = {
            start: t('logsPage.containerStarted'),
            stop: t('logsPage.containerStopped'),
            restart: t('logsPage.containerRestarted'),
        };
        try {
            if (!containerId) return;
            const res = await guiApi.containers.containersCreate(containerId, command);
            if (res.error) throw new Error(res.error.error);
            if (command === "start" || command === "restart") {
                stream.start(`/api/containers/${containerId}/logs`);
            } else {
                // Stopping the container intentionally closes the stream.
                intentionalStopRef.current = true;
                stream?.stop();
            }
            await listContainers();
            notification.success({message: messages[command]});
        } catch (e: unknown) {
            notification.error({
                message: t('logsPage.failedToCommandContainer', {command}),
                description: e instanceof Error ? e.message : String(e),
            });
        }
    };

    const selectedContainer = containers.find(c => c.value === containerId);

    // Issue #207 moved the timestamp out of the line body and into its own
    // rendered column, so `plain` no longer contains it and a search for
    // "22:02" — or for an epoch pasted out of a bug report — silently stopped
    // matching. Search the rendered column too, but only for queries that can
    // possibly match one: formatting the whole buffer costs ~11 ms per
    // keystroke at MAX_LINES (with the formatter cache in logTime.ts; ~110 ms
    // without it), and an ordinary word search must not pay that.
    const filtered = useMemo(() => {
        const levelSet = new Set(levels);
        const q = search.trim().toLowerCase();
        const searchesTime = q !== '' && QUERY_HAS_DIGIT.test(q);
        return logs.filter(l => {
            if (!levelSet.has(l.severity)) return false;
            if (!q) return true;
            if (l.plain.toLowerCase().includes(q)) return true;
            if (!searchesTime) return false;
            // Raw epochs, as pasted from a bug report or an older ROS console.
            if (String(Math.floor(l.tsMs / 1000)).includes(q)) return true;
            if (String(l.tsMs).includes(q)) return true;
            // The rendered column, so "22:02" or "2026-05-12" match what is on
            // screen. withMillis is a strict superset of both the desktop
            // (YYYY-MM-DDTHH:mm:ss) and the mobile (HH:mm:ss) column, so one
            // format call covers every layout and also matches ".123".
            return formatLogTime(l.tsMs, {withMillis: true}).toLowerCase().includes(q);
        });
        // formatLogTime is a useCallback keyed on timeZoneMode: without it in
        // the deps, toggling UTC would leave a stale filter result on screen.
    }, [logs, levels, search, formatLogTime]);

    const counts = useMemo(() => {
        const c: Record<Severity, number> = {ERROR: 0, WARN: 0, INFO: 0, DEBUG: 0, OTHER: 0};
        logs.forEach(l => { c[l.severity] += 1; });
        return c;
    }, [logs]);

    // Once the buffer hits MAX_LINES, filtered.length plateaus even as new
    // lines keep arriving, so an effect keyed on it stops firing and autoscroll
    // dies. Key on the last line's monotonic id, which always advances.
    const lastLineId = filtered.length > 0 ? filtered[filtered.length - 1].id : -1;
    useEffect(() => {
        if (!autoScroll) return;
        const el = listRef.current;
        if (el) el.scrollTop = el.scrollHeight;
    }, [lastLineId, autoScroll]);

    const handleScroll = () => {
        const el = listRef.current;
        if (!el) return;
        const atBottom = el.scrollHeight - el.scrollTop - el.clientHeight < 24;
        setAutoScroll(atBottom);
    };

    const severityColor = (s: Severity): string => {
        switch (s) {
            case 'ERROR': return colors.danger;
            case 'WARN': return colors.warning;
            case 'INFO': return colors.info;
            case 'DEBUG': return colors.textMuted;
            default: return colors.textDim;
        }
    };

    return (
        <div style={{display: 'flex', flexDirection: 'column', gap: 12, height: '100%'}}>
            {/* Container picker + lifecycle controls */}
            <div style={{
                display: 'flex', flexDirection: isMobile ? 'column' : 'row', gap: 8,
                alignItems: isMobile ? 'stretch' : 'center',
                background: colors.bgCard, borderRadius: 12, padding: 12, flexShrink: 0,
            }}>
                <Select<string>
                    options={containers}
                    value={containerId}
                    style={{flex: 1, minWidth: isMobile ? undefined : 200}}
                    onSelect={(value) => setContainerId(value)}
                    placeholder={t('logsPage.selectContainer')}
                />
                <Space size={8} style={{flexShrink: 0}}>
                    {selectedContainer?.status === "started" && (
                        <>
                            <AsyncButton onAsyncClick={commandContainer("restart")} size={isMobile ? "middle" : "small"}>{t('logsPage.restart')}</AsyncButton>
                            <AsyncButton
                                disabled={selectedContainer.labels.app === "gui"}
                                onAsyncClick={commandContainer("stop")}
                                size={isMobile ? "middle" : "small"}
                            >{t('logsPage.stop')}</AsyncButton>
                        </>
                    )}
                    {selectedContainer?.status === "stopped" && (
                        <AsyncButton onAsyncClick={commandContainer("start")} size={isMobile ? "middle" : "small"}>{t('logsPage.start')}</AsyncButton>
                    )}
                </Space>
            </div>

            {/* Filter chips + search */}
            <div style={{
                display: 'flex', flexDirection: isMobile ? 'column' : 'row',
                gap: 10, alignItems: isMobile ? 'stretch' : 'center',
                background: colors.bgCard, borderRadius: 12, padding: '10px 12px', flexShrink: 0,
                flexWrap: 'wrap',
            }}>
                <Input.Search
                    placeholder={t('logsPage.searchPlaceholder')}
                    value={search}
                    onChange={(e) => setSearch(e.target.value)}
                    allowClear
                    style={{flex: 1, maxWidth: isMobile ? undefined : 360}}
                />
                <div style={{display: 'flex', gap: 6, flexWrap: 'wrap'}}>
                    {LEVEL_OPTIONS.map(opt => {
                        const active = levels.includes(opt.value);
                        const accent = severityColor(opt.value);
                        return (
                            <button
                                key={opt.value}
                                onClick={() => setLevels(prev =>
                                    prev.includes(opt.value)
                                        ? prev.filter(l => l !== opt.value)
                                        : [...prev, opt.value]
                                )}
                                style={{
                                    padding: '4px 10px', borderRadius: 999, fontSize: 12, fontWeight: 600,
                                    border: `1px solid ${active ? accent : colors.border}`,
                                    background: active ? `${accent}1f` : 'transparent',
                                    color: active ? accent : colors.textDim,
                                    cursor: 'pointer',
                                    transition: 'border-color 0.15s, background-color 0.15s, color 0.15s',
                                }}
                                aria-pressed={active}
                            >
                                {t(opt.label)} <span style={{opacity: 0.7, marginLeft: 4}}>{counts[opt.value]}</span>
                            </button>
                        );
                    })}
                </div>
                <div style={{display: 'flex', gap: 6, marginLeft: isMobile ? 0 : 'auto'}}>
                    <button
                        onClick={() => setTimeZoneMode(timeZoneMode === 'utc' ? 'local' : 'utc')}
                        aria-label={t('logTime.toggleAria')}
                        title={t('logTime.toggleAria')}
                        style={{
                            padding: '4px 10px', borderRadius: 999, fontSize: 12, fontWeight: 600,
                            border: `1px solid ${colors.border}`, background: 'transparent',
                            color: colors.textDim, cursor: 'pointer',
                        }}
                    >
                        {timeZoneMode === 'utc' ? t('logsPage.timeZoneUtc') : t('logsPage.timeZoneLocal')}
                        <span style={{opacity: 0.7, marginLeft: 4}}>{zoneLabel}</span>
                    </button>
                    <button
                        onClick={() => setAutoScroll(a => !a)}
                        style={{
                            padding: '4px 10px', borderRadius: 999, fontSize: 12, fontWeight: 600,
                            border: `1px solid ${autoScroll ? colors.accent : colors.border}`,
                            background: autoScroll ? colors.accentSoft : 'transparent',
                            color: autoScroll ? colors.accent : colors.textDim,
                            cursor: 'pointer',
                        }}
                    >
                        {autoScroll ? `↓ ${t('logsPage.live')}` : `↓ ${t('logsPage.paused')}`}
                    </button>
                    <button
                        onClick={resetLogs}
                        style={{
                            padding: '4px 10px', borderRadius: 999, fontSize: 12, fontWeight: 600,
                            border: `1px solid ${colors.border}`, background: 'transparent',
                            color: colors.textDim, cursor: 'pointer',
                        }}
                    >{t('logsPage.clear')}</button>
                </div>
            </div>

            {/* Log lines */}
            <div
                ref={listRef}
                onScroll={handleScroll}
                style={{
                    flex: 1, minHeight: 0, overflow: 'auto', borderRadius: 12,
                    background: colors.bgCard, padding: '6px 0',
                    fontFamily: '"JetBrains Mono", "SF Mono", ui-monospace, monospace',
                    fontSize: 12, lineHeight: 1.7,
                    border: `1px solid ${colors.borderSubtle}`,
                }}
            >
                {filtered.length === 0 && (
                    <div style={{padding: '40px 16px', textAlign: 'center', color: colors.textMuted}}>
                        {logs.length === 0 ? t('logsPage.waitingForOutput') : t('logsPage.noLinesMatch')}
                    </div>
                )}
                {filtered.map(line => {
                    const accent = severityColor(line.severity);
                    return (
                        <div
                            key={line.id}
                            data-testid="log-line"
                            style={{
                                padding: '3px 14px 3px 12px',
                                borderLeft: `3px solid ${line.severity === 'OTHER' ? 'transparent' : accent}`,
                                background: line.severity === 'ERROR'
                                    ? `${colors.danger}0d`
                                    : line.severity === 'WARN'
                                        ? `${colors.warning}0a`
                                        : 'transparent',
                                color: colors.text,
                                whiteSpace: 'pre-wrap', wordBreak: 'break-all',
                                contentVisibility: 'auto',
                                containIntrinsicSize: 'auto 26px',
                            }}
                        >
                            <span
                                title={line.tsSource === 'received'
                                    ? t('logTime.approximate')
                                    : formatLogTime(line.tsMs, {withMillis: true})}
                                style={{
                                    color: line.tsSource === 'received' ? colors.textMuted : colors.textDim,
                                    marginRight: 10, fontSize: 11,
                                    display: 'inline-block',
                                    minWidth: `${isMobile ? TIMESTAMP_COLUMN_MOBILE_CH : TIMESTAMP_COLUMN_CH}ch`,
                                }}
                            >
                                {formatLogTime(line.tsMs, {timeOnly: isMobile})}
                            </span>
                            {line.severity !== 'OTHER' && (
                                <span style={{
                                    color: accent, fontWeight: 700,
                                    marginRight: 10, fontSize: 10, letterSpacing: '0.06em',
                                    display: 'inline-block', minWidth: 38,
                                }}>
                                    {line.severity}
                                </span>
                            )}
                            <span>{line.plain}</span>
                        </div>
                    );
                })}
            </div>
        </div>
    );
};

export default LogsPage;
