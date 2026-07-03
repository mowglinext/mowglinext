import {App, Input, Select, Space} from "antd";
import {useEffect, useMemo, useRef, useState} from "react";
import {useTranslation} from "react-i18next";
import AsyncButton from "../components/AsyncButton.tsx";
import {useWS} from "../hooks/useWS.ts";
import {useApi} from "../hooks/useApi.ts";
import {useThemeMode} from "../theme/ThemeContext.tsx";
import {useIsMobile} from "../hooks/useIsMobile";

type Severity = 'ERROR' | 'WARN' | 'INFO' | 'DEBUG' | 'OTHER';

const LEVEL_PATTERN = /\b(ERROR|ERR|FATAL|CRITICAL|WARN(?:ING)?|INFO|DEBUG|TRACE)\b/i;
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
    plain: string;
    severity: Severity;
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

type ContainerList = { value: string, label: string, status: "started" | "stopped", labels: Record<string, string> };

export const LogsPage = () => {
    const {t} = useTranslation();
    const {colors} = useThemeMode();
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

    const stream = useWS<string>(
        () => { notification.error({message: t('logsPage.streamClosed')}); },
        () => { /* connected */ },
        (line, first) => {
            setLogs(prev => {
                const plain = line.replace(ANSI_REGEX, '');
                const entry: ParsedLog = {
                    id: nextIdRef.current++,
                    plain,
                    severity: detectSeverity(plain),
                };
                const base = first ? [] : prev;
                const next = [...base, entry];
                return next.length > MAX_LINES ? next.slice(next.length - MAX_LINES) : next;
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
        nextIdRef.current = 0;
        setLogs([]);
        stream.start(`/api/containers/${containerId}/logs`);
        return () => { stream?.stop(); };
    }, [containerId]);

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

    const filtered = useMemo(() => {
        const levelSet = new Set(levels);
        const q = search.trim().toLowerCase();
        return logs.filter(l => {
            if (!levelSet.has(l.severity)) return false;
            if (q && !l.plain.toLowerCase().includes(q)) return false;
            return true;
        });
    }, [logs, levels, search]);

    const counts = useMemo(() => {
        const c: Record<Severity, number> = {ERROR: 0, WARN: 0, INFO: 0, DEBUG: 0, OTHER: 0};
        logs.forEach(l => { c[l.severity] += 1; });
        return c;
    }, [logs]);

    useEffect(() => {
        if (!autoScroll) return;
        const el = listRef.current;
        if (el) el.scrollTop = el.scrollHeight;
    }, [filtered.length, autoScroll]);

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
                                    cursor: 'pointer', transition: 'all 0.15s',
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
                        onClick={() => { setLogs([]); nextIdRef.current = 0; }}
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
                            }}
                        >
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
