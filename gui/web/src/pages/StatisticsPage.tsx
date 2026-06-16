import {useCallback, useEffect, useState} from "react";
import {Table, Tag, Button, Popconfirm, message} from "antd";
import {DeleteOutlined} from "@ant-design/icons";
import {useApi} from "../hooks/useApi.ts";
import {useDiagnosticsSnapshot} from "../hooks/useDiagnosticsSnapshot.ts";
import {useThemeMode} from "../theme/ThemeContext.tsx";
import {useIsMobile} from "../hooks/useIsMobile";
import {DashCard, Bar} from "../components/dashboard";
import {YearOfLawn} from "../components/YearOfLawn.tsx";

interface MowingSession {
  id: string;
  start_time: string;
  end_time: string;
  duration_sec: number;
  area_index: number;
  coverage_percent: number;
  strips_completed: number;
  strips_skipped: number;
  distance_meters: number;
  status: "completed" | "aborted" | "error";
  recharge_pauses?: number;
  errors: string[];
}

interface SessionsResponse {
  sessions: MowingSession[];
  total: number;
}

interface SessionStats {
  total_sessions: number;
  total_duration_sec: number;
  total_distance_m: number;
  total_strips: number;
  completed: number;
  aborted: number;
  errors: number;
  avg_coverage_pct: number;
}

function formatDuration(seconds: number): string {
  if (!seconds || seconds <= 0) return "0m";
  const h = Math.floor(seconds / 3600);
  const m = Math.floor((seconds % 3600) / 60);
  if (h > 0) return `${h}h ${m}m`;
  return `${m}m`;
}

function formatTotalHours(seconds: number): string {
  if (!seconds || seconds <= 0) return "0";
  return Math.round(seconds / 3600).toString();
}

function formatDistance(meters: number): string {
  if (!meters || meters <= 0) return "0";
  if (meters >= 1000) return (meters / 1000).toFixed(1);
  return Math.round(meters).toString();
}

function formatDistanceUnit(meters: number): string {
  return meters >= 1000 ? "km" : "m";
}

function formatDate(timestamp: string): string {
  if (!timestamp) return "--";
  return new Date(timestamp).toLocaleString(undefined, {
    month: "short", day: "numeric", hour: "2-digit", minute: "2-digit",
  });
}

export const StatisticsPage = () => {
  const guiApi = useApi();
  const {snapshot} = useDiagnosticsSnapshot();
  const {colors} = useThemeMode();
  const isMobile = useIsMobile();

  const [sessions, setSessions] = useState<MowingSession[]>([]);
  const [stats, setStats] = useState<SessionStats | null>(null);
  const [loading, setLoading] = useState(false);
  const [hoveredBar, setHoveredBar] = useState<number | null>(null);

  const fetchData = useCallback(async () => {
    setLoading(true);
    try {
      const [sessionsRes, statsRes] = await Promise.all([
        guiApi.request<SessionsResponse>({path: "/diagnostics/sessions", method: "GET", format: "json"}),
        guiApi.request<SessionStats>({path: "/diagnostics/sessions/stats", method: "GET", format: "json"}),
      ]);
      setSessions(sessionsRes.data?.sessions ?? []);
      setStats(statsRes.data ?? null);
    } catch { /* silently degrade */ }
    finally { setLoading(false); }
  }, []);

  const clearStats = useCallback(async () => {
    try {
      await guiApi.request({path: "/diagnostics/sessions", method: "DELETE", format: "json"});
      message.success("Statistics cleared");
      await fetchData();
    } catch {
      message.error("Failed to clear statistics");
    }
  }, [fetchData]);

  useEffect(() => {
    fetchData();
    const interval = setInterval(fetchData, 30000);
    return () => clearInterval(interval);
  }, [fetchData]);

  const completionRate = stats && stats.total_sessions > 0
    ? Math.round((stats.completed / stats.total_sessions) * 100) : 0;

  const coverage = snapshot?.coverage ?? [];

  // Generate fake weekly data from sessions for the bar chart
  const weeklyBars = Array.from({length: 12}, (_, i) => {
    const weekAgo = 12 - i;
    const weekSessions = sessions.filter(s => {
      const d = new Date(s.start_time);
      const now = new Date();
      const diffWeeks = Math.floor((now.getTime() - d.getTime()) / (7 * 24 * 60 * 60 * 1000));
      return diffWeeks === weekAgo;
    });
    return weekSessions.reduce((acc, s) => acc + (s.distance_meters / 1000), 0);
  });
  const maxBar = Math.max(...weeklyBars, 0.01);

  const sessionColumns = [
    {
      title: "Date", dataIndex: "start_time", key: "start_time",
      sorter: (a: MowingSession, b: MowingSession) =>
        new Date(b.start_time).getTime() - new Date(a.start_time).getTime(),
      defaultSortOrder: "ascend" as const,
      render: (v: string) => <span style={{fontSize: 13}}>{formatDate(v)}</span>,
    },
    ...(!isMobile ? [{
      title: "Duration", dataIndex: "duration_sec", key: "duration",
      render: (v: number) => <span style={{fontSize: 13}}>{formatDuration(v)}</span>,
    }] : []),
    {
      title: "Area", dataIndex: "area_index", key: "area_index",
      render: (v: number) => <span style={{fontSize: 13}}>{v != null && v >= 0 ? `#${v}` : "--"}</span>,
    },
    {
      title: "Coverage", dataIndex: "coverage_percent", key: "coverage",
      render: (v: number) => (
        <div style={{display: 'flex', alignItems: 'center', gap: 8, minWidth: isMobile ? 60 : 80}}>
          <Bar value={v ?? 0} color={colors.accent} track={colors.border} height={6}/>
          <span style={{fontSize: 11, color: colors.textDim, whiteSpace: 'nowrap'}}>
            {Math.round(v ?? 0)}%
          </span>
        </div>
      ),
    },
    ...(!isMobile ? [{
      title: "Status", dataIndex: "status", key: "status",
      render: (v: string, record: MowingSession) => {
        const c = v === "completed" ? "success" : v === "aborted" ? "warning" : "error";
        return (
          <span style={{display: 'inline-flex', gap: 6, alignItems: 'center'}}>
            <Tag color={c}>{v ?? "--"}</Tag>
            {record.recharge_pauses ? (
              <Tag color="processing">⏸ {record.recharge_pauses}× recharge</Tag>
            ) : null}
          </span>
        );
      },
    }] : []),
  ];

  const isEmpty = !stats || stats.total_sessions === 0;

  return (
    <div style={{display: 'flex', flexDirection: 'column', gap: 16, paddingBottom: 8}}>
      {/* Editorial page header */}
      <div style={{marginBottom: 2}}>
        <div style={{
          fontSize: 11, color: colors.textMuted, fontWeight: 600,
          letterSpacing: '0.12em', textTransform: 'uppercase' as const,
        }}>
          Statistiques
        </div>
        <div className="mn-display" style={{
          fontSize: isMobile ? 30 : 40, color: colors.text,
          lineHeight: 1.05, marginTop: 4, letterSpacing: '-0.02em',
        }}>
          Une année de tonte
        </div>
      </div>

      {/* Hero stats -- accent watermark per metric, no border to feel lighter */}
      <div style={{display: 'grid', gridTemplateColumns: isMobile ? 'repeat(2, 1fr)' : 'repeat(4, 1fr)', gap: 12}}>
        {[
          {label: 'Total distance', value: formatDistance(stats?.total_distance_m ?? 0), unit: formatDistanceUnit(stats?.total_distance_m ?? 0), hint: 'since install', color: colors.accent},
          {label: 'Hours active', value: formatTotalHours(stats?.total_duration_sec ?? 0), unit: 'h', hint: `${stats?.total_sessions ?? 0} sessions`, color: colors.sky},
          {label: 'Completion rate', value: `${completionRate}`, unit: '%', hint: `${stats?.completed ?? 0} completed`, color: colors.amber},
          {label: 'Runs completed', value: `${stats?.total_sessions ?? 0}`, unit: '', hint: `avg ${Math.round(stats?.avg_coverage_pct ?? 0)}% coverage`, color: colors.accent},
        ].map(s => (
          <DashCard key={s.label} padding={isMobile ? 16 : 20}
                    style={{position: 'relative', overflow: 'hidden'}}>
            <div aria-hidden style={{
              position: 'absolute', top: -28, right: -28, width: 110, height: 110, borderRadius: 110,
              background: `radial-gradient(circle, ${isEmpty ? 'transparent' : `${s.color}24`} 0%, transparent 70%)`,
              pointerEvents: 'none',
            }}/>
            <div style={{
              position: 'relative', fontSize: 11, color: colors.textDim,
              letterSpacing: '0.08em', textTransform: 'uppercase' as const, marginBottom: 10,
              fontWeight: 600,
            }}>
              {s.label}
            </div>
            <div style={{position: 'relative', display: 'flex', alignItems: 'baseline', gap: 4}}>
              <div className="mn-num" style={{
                fontSize: isMobile ? 44 : 60,
                color: isEmpty ? colors.textMuted : s.color,
                lineHeight: 1,
              }}>
                {s.value}
              </div>
              {s.unit && (
                <div style={{
                  fontSize: 14, color: colors.textDim, fontWeight: 500,
                  marginLeft: 4,
                  fontFamily: "'Space Grotesk', 'JetBrains Mono', monospace",
                  textTransform: 'lowercase' as const, letterSpacing: '0.04em',
                }}>{s.unit}</div>
              )}
            </div>
            <div style={{
              position: 'relative', fontSize: 11, color: colors.textMuted, marginTop: 8, fontWeight: 500,
            }}>{s.hint}</div>
          </DashCard>
        ))}
      </div>

      {isEmpty && (
        <DashCard padding={isMobile ? 16 : 20} style={{textAlign: 'center'}}>
          <div style={{fontSize: 14, fontWeight: 600, color: colors.text}}>
            Aucune tonte pour l'instant
          </div>
          <div style={{fontSize: 12, color: colors.textDim, marginTop: 6}}>
            Votre première session apparaîtra ici.
          </div>
        </DashCard>
      )}

      {/* Year of lawn -- contribution-style heatmap of mowing distance */}
      <DashCard>
        <YearOfLawn sessions={sessions}/>
      </DashCard>

      {/* Weekly chart */}
      <DashCard>
        <div style={{display: 'flex', alignItems: 'center', justifyContent: 'space-between', marginBottom: 16}}>
          <div>
            <div style={{fontSize: 14, fontWeight: 600}}>Distance per week</div>
            <div style={{fontSize: 11, color: colors.textMuted}}>Last 12 weeks (km)</div>
          </div>
        </div>
        <div style={{display: 'flex', alignItems: 'flex-end', gap: isMobile ? 4 : 8, height: isMobile ? 120 : 180, paddingBottom: 20, position: 'relative'}}>
          {[0.25, 0.5, 0.75].map(p => {
            const chartH = isMobile ? 100 : 160;
            return <div key={p} style={{position: 'absolute', left: 0, right: 0, bottom: 20 + p * chartH, height: 1, background: colors.border}}/>;
          })}
          {/* max-value gridline label */}
          {maxBar > 0.01 && (
            <div style={{
              position: 'absolute', left: 0, top: -4, fontSize: 9, fontWeight: 600,
              color: colors.textMuted, fontFamily: "'Space Grotesk', monospace",
            }}>
              {maxBar.toFixed(1)} km
            </div>
          )}
          {weeklyBars.map((v, i) => {
            const chartH = isMobile ? 100 : 160;
            const h = maxBar > 0 ? (v / maxBar) * chartH : 0;
            const isLatest = i === weeklyBars.length - 1;
            const showValue = (isLatest || hoveredBar === i) && v > 0;
            return (
              <div
                key={i}
                onMouseEnter={() => setHoveredBar(i)}
                onMouseLeave={() => setHoveredBar(null)}
                onClick={() => setHoveredBar(hoveredBar === i ? null : i)}
                title={`Week ${i + 1}: ${v.toFixed(2)} km`}
                style={{flex: 1, display: 'flex', flexDirection: 'column', alignItems: 'center', gap: 6, position: 'relative', cursor: 'pointer'}}
              >
                <div style={{
                  width: '100%', height: Math.max(2, h),
                  background: (isLatest || hoveredBar === i)
                    ? `linear-gradient(180deg, ${colors.accent}, ${colors.accent}88)`
                    : `linear-gradient(180deg, ${colors.accent}66, ${colors.accent}22)`,
                  borderRadius: '6px 6px 2px 2px',
                  border: (isLatest || hoveredBar === i) ? `1px solid ${colors.accent}` : 'none',
                  transition: 'height .4s',
                }}/>
                <div style={{fontSize: 9, color: colors.textMuted, position: 'absolute', bottom: 0}}>W{i + 1}</div>
                {showValue && (
                  <div style={{position: 'absolute', top: -22, fontSize: 10, fontWeight: 700, color: colors.accent, whiteSpace: 'nowrap'}}>
                    {v.toFixed(1)} km
                  </div>
                )}
              </div>
            );
          })}
        </div>
      </DashCard>

      {/* Coverage + session history */}
      <div style={{display: 'grid', gridTemplateColumns: isMobile ? '1fr' : (coverage.length > 0 ? '1fr 1.4fr' : '1fr'), gap: 14}}>
        {coverage.length > 0 && (
          <DashCard>
            <div style={{fontSize: 14, fontWeight: 600, marginBottom: 14}}>Zone coverage</div>
            {coverage.map(area => (
              <div key={area.area_index} style={{marginBottom: 10}}>
                <div style={{display: 'flex', justifyContent: 'space-between', fontSize: 12, marginBottom: 4}}>
                  <span style={{color: colors.text, fontWeight: 500}}>Area {area.area_index}</span>
                  <span style={{color: colors.textDim}}>
                    {area.mowed_cells}/{area.total_cells} cells
                  </span>
                </div>
                <Bar
                  value={area.coverage_percent} max={100}
                  color={colors.accent} track={colors.border} height={6}
                />
              </div>
            ))}
          </DashCard>
        )}

        <DashCard padding={0}>
          <div style={{padding: '18px 18px 0', display: 'flex', alignItems: 'flex-start', justifyContent: 'space-between', gap: 12}}>
            <div>
              <div style={{fontSize: 14, fontWeight: 600, marginBottom: 4}}>Session history</div>
              <div style={{fontSize: 11, color: colors.textMuted, marginBottom: 14}}>
                {sessions.length} session{sessions.length !== 1 ? 's' : ''} recorded
              </div>
            </div>
            <Popconfirm
              title="Clear all statistics?"
              description="This permanently deletes every recorded session. This cannot be undone."
              okText="Clear"
              okButtonProps={{danger: true}}
              cancelText="Cancel"
              onConfirm={clearStats}
            >
              <Button size="small" danger icon={<DeleteOutlined/>} disabled={sessions.length === 0}>
                Clear
              </Button>
            </Popconfirm>
          </div>
          <Table
            size="small"
            loading={loading}
            dataSource={sessions}
            columns={sessionColumns}
            rowKey="id"
            pagination={{pageSize: 10, showSizeChanger: false}}
            locale={{
              emptyText: (
                <div style={{padding: '24px 0', color: colors.textSecondary}}>
                  No mowing sessions recorded yet.
                </div>
              ),
            }}
          />
        </DashCard>
      </div>
    </div>
  );
};

export default StatisticsPage;
