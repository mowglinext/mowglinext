import {App, Select, Switch, TimePicker} from "antd";
import {useCallback, useEffect, useRef, useState} from "react";
import {useApi} from "../hooks/useApi.ts";
import {useWS} from "../hooks/useWS.ts";
import {Map as MapType} from "../types/ros.ts";
import {useIsMobile} from "../hooks/useIsMobile";
import {useThemeMode} from "../theme/ThemeContext.tsx";
import {DashCard, ActionButton, IconPlus, FONT} from "../components/dashboard";
import dayjs from "dayjs";

interface Schedule {
  id: string;
  area: number;
  time: string;
  daysOfWeek: number[];
  enabled: boolean;
  createdAt: string;
  lastRun?: string;
}

const DAYS = ["Mon", "Tue", "Wed", "Thu", "Fri", "Sat", "Sun"];
const DAY_LETTERS = ["S", "M", "T", "W", "T", "F", "S"];

function areaLabel(index: number, name: string | undefined): string {
  return name ? `${index + 1}. ${name}` : `Area ${index + 1}`;
}

export const SchedulePage = () => {
  const {colors} = useThemeMode();
  const guiApi = useApi();
  const {notification, modal} = App.useApp();
  const isMobile = useIsMobile();
  const [schedules, setSchedules] = useState<Schedule[]>([]);
  const [loading, setLoading] = useState(false);
  const [workingAreas, setWorkingAreas] = useState<Array<string | undefined>>([]);
  const fetchedRef = useRef(false);

  const mapStream = useWS<string>(
    () => {},
    () => {},
    (data) => {
      const parsed = JSON.parse(data) as MapType;
      const names = (parsed.working_area ?? []).map((a) => a.name);
      setWorkingAreas(names);
    },
  );

  useEffect(() => {
    mapStream.start("/api/mowglinext/subscribe/map");
    return () => { mapStream.stop(); };
  }, []);

  const areaOptions = workingAreas.length > 0
    ? workingAreas.map((name, index) => ({label: areaLabel(index, name), value: index}))
    : Array.from({length: Math.max(1, schedules.reduce((max, s) => Math.max(max, s.area + 1), 1))}, (_, i) => ({
        label: areaLabel(i, undefined), value: i,
      }));

  const fetchSchedules = useCallback(async () => {
    try {
      const response = await guiApi.request<{ schedules: Schedule[] }>({
        path: "/schedules", method: "GET", format: "json",
      });
      setSchedules(response.data.schedules ?? []);
    } catch {
      notification.error({message: "Failed to load schedules"});
    }
  }, [guiApi, notification]);

  useEffect(() => {
    if (fetchedRef.current) return;
    fetchedRef.current = true;
    fetchSchedules();
  }, [fetchSchedules]);

  const handleCreate = async (body?: Partial<Schedule>) => {
    setLoading(true);
    try {
      await guiApi.request({
        path: "/schedules", method: "POST",
        body: {area: 0, time: "09:00", daysOfWeek: [1, 2, 3, 4, 5], enabled: false, ...body},
        format: "json",
      });
      await fetchSchedules();
    } catch {
      notification.error({message: "Failed to create schedule"});
    } finally {
      setLoading(false);
    }
  };

  const STARTER_TEMPLATES: { name: string; subtitle: string; body: Partial<Schedule>; icon: string }[] = [
    {
      name: "Weekend warrior",
      subtitle: "Saturdays at 10:00",
      body: {time: "10:00", daysOfWeek: [6]},
      icon: "sun",
    },
    {
      name: "Stealth runs",
      subtitle: "Mon · Wed · Fri at 06:00",
      body: {time: "06:00", daysOfWeek: [1, 3, 5]},
      icon: "moon",
    },
    {
      name: "Daily quick mow",
      subtitle: "Every day at 08:00",
      body: {time: "08:00", daysOfWeek: [0, 1, 2, 3, 4, 5, 6]},
      icon: "calendar",
    },
  ];

  const templateAccents = [colors.accent, colors.sky, colors.amber, colors.pink];
  const templateCards = (
    <div style={{display: 'grid', gridTemplateColumns: isMobile ? '1fr' : 'repeat(3, 1fr)', gap: 10}}>
      {STARTER_TEMPLATES.map((tpl, i) => {
        const color = templateAccents[i % templateAccents.length];
        return (
          <button
            key={tpl.name}
            onClick={() => handleCreate(tpl.body)}
            disabled={loading}
            style={{
              textAlign: 'left',
              background: `${color}10`,
              border: `1px solid ${color}40`,
              borderRadius: 10,
              padding: '12px 14px',
              cursor: loading ? 'wait' : 'pointer',
              transition: 'transform 0.15s, border-color 0.15s',
              fontFamily: FONT,
            }}
            onMouseEnter={(e) => { e.currentTarget.style.transform = 'translateY(-2px)'; e.currentTarget.style.borderColor = color; }}
            onMouseLeave={(e) => { e.currentTarget.style.transform = 'none'; e.currentTarget.style.borderColor = `${color}40`; }}
          >
            <div style={{fontSize: 11, color, fontWeight: 700, letterSpacing: '0.06em', textTransform: 'uppercase'}}>
              {tpl.icon === 'sun' ? 'Weekly' : tpl.icon === 'moon' ? 'Off-hours' : 'Every day'}
            </div>
            <div style={{fontSize: 14, fontWeight: 700, color: colors.text, marginTop: 4}}>
              {tpl.name}
            </div>
            <div style={{fontSize: 12, color: colors.textDim, marginTop: 4}}>
              {tpl.subtitle}
            </div>
          </button>
        );
      })}
    </div>
  );

  const handleUpdate = async (sched: Schedule) => {
    try {
      await guiApi.request({path: `/schedules/${sched.id}`, method: "PUT", body: sched, format: "json"});
      await fetchSchedules();
    } catch {
      notification.error({message: "Failed to update schedule"});
    }
  };

  const handleDelete = async (id: string) => {
    try {
      await guiApi.request({path: `/schedules/${id}`, method: "DELETE", format: "json"});
      await fetchSchedules();
    } catch {
      notification.error({message: "Failed to delete schedule"});
    }
  };

  const confirmDelete = (id: string) => {
    modal.confirm({
      title: "Delete schedule",
      content: "Are you sure you want to delete this schedule?",
      okText: "Delete",
      okType: "danger",
      cancelText: "Cancel",
      onOk: () => handleDelete(id),
    });
  };

  const toggleDay = (sched: Schedule, day: number) => {
    const days = sched.daysOfWeek.includes(day)
      ? sched.daysOfWeek.filter(d => d !== day)
      : [...sched.daysOfWeek, day];
    handleUpdate({...sched, daysOfWeek: days});
  };

  // Color per schedule index
  const schedColors = [colors.accent, colors.sky, colors.amber, colors.pink];

  // Build grid runs from schedules for weekly view
  const hours = [6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19];
  const gridRuns = schedules.flatMap((sched, si) => {
    const startH = parseInt(sched.time.split(':')[0]);
    return sched.daysOfWeek
      .filter(d => d >= 0 && d <= 6)
      .map(dayIndex => ({
        day: dayIndex === 0 ? 6 : dayIndex - 1, // convert Sun=0..Sat=6 to Mon=0..Sun=6
        start: startH,
        end: Math.min(startH + 1, 20),
        zone: areaLabel(sched.area, workingAreas[sched.area]),
        color: schedColors[si % schedColors.length],
      }));
  });

  const activeCount = schedules.filter(s => s.enabled).length;

  // Schedule card for each schedule (mobile + bottom section on desktop)
  const scheduleCard = (sched: Schedule, idx: number) => {
    const optionsWithCurrent = areaOptions.some((o) => o.value === sched.area)
      ? areaOptions
      : [...areaOptions, {label: areaLabel(sched.area, undefined), value: sched.area}];
    const color = schedColors[idx % schedColors.length];
    return (
      <DashCard key={sched.id} style={{display: 'flex', flexDirection: 'column', gap: 12}}>
        <div style={{display: 'flex', alignItems: 'center', gap: 12}}>
          <div style={{width: 4, height: 32, borderRadius: 2, background: color}}/>
          <Switch
            checked={sched.enabled}
            onChange={(checked) => handleUpdate({...sched, enabled: checked})}
          />
          <Select
            value={sched.area}
            size="small"
            style={{flex: 1}}
            options={optionsWithCurrent}
            onChange={(val) => handleUpdate({...sched, area: val})}
          />
        </div>
        <TimePicker
          value={dayjs(sched.time, "HH:mm")}
          format="HH:mm"
          onChange={(val) => { if (val) handleUpdate({...sched, time: val.format("HH:mm")}); }}
          style={{width: '100%'}}
          size="large"
        />
        <div style={{display: 'flex', gap: 6, justifyContent: 'space-between'}}>
          {DAY_LETTERS.map((letter, i) => {
            const isActive = sched.daysOfWeek.includes(i);
            return (
              <button
                key={i}
                onClick={() => toggleDay(sched, i)}
                style={{
                  width: 44, height: 44, borderRadius: '50%',
                  border: `1.5px solid ${isActive ? color : colors.border}`,
                  background: isActive ? `${color}20` : 'transparent',
                  color: isActive ? color : colors.textSecondary,
                  fontSize: 13, fontWeight: 600, cursor: 'pointer',
                  transition: 'all 0.15s', padding: 0, fontFamily: FONT,
                }}
              >
                {letter}
              </button>
            );
          })}
        </div>
        <div style={{
          display: 'flex', justifyContent: 'space-between', alignItems: 'center',
          borderTop: `1px solid ${colors.borderSubtle}`, paddingTop: 8,
        }}>
          <span style={{fontSize: 12, color: colors.textSecondary}}>
            Last: {sched.lastRun ? dayjs(sched.lastRun).format("YYYY-MM-DD HH:mm") : "Never"}
          </span>
          <button
            onClick={() => confirmDelete(sched.id)}
            style={{
              background: colors.dangerBg, color: colors.danger,
              border: 'none', borderRadius: 8, padding: '6px 12px',
              fontSize: 12, fontWeight: 600, cursor: 'pointer', fontFamily: FONT,
            }}
          >
            Delete
          </button>
        </div>
      </DashCard>
    );
  };

  const pageHeader = (
    <div>
      <div style={{
        fontSize: 11, color: colors.textMuted, fontWeight: 600,
        letterSpacing: '0.12em', textTransform: 'uppercase' as const,
      }}>
        Planning
      </div>
      <div className="mn-display" style={{
        fontSize: isMobile ? 30 : 40, color: colors.text,
        lineHeight: 1.05, marginTop: 4, letterSpacing: '-0.02em',
      }}>
        Tontes programmées
      </div>
    </div>
  );

  if (isMobile) {
    return (
      <div style={{display: 'flex', flexDirection: 'column', gap: 12, paddingBottom: 8}}>
        <div style={{display: 'flex', justifyContent: 'space-between', alignItems: 'flex-end', gap: 12}}>
          {pageHeader}
          <ActionButton primary icon={<IconPlus size={14}/>} label="New run" onClick={() => handleCreate()} disabled={loading}/>
        </div>
        {schedules.length === 0 && (
          <>
            <DashCard style={{padding: 18}}>
              <div style={{fontSize: 14, fontWeight: 700, marginBottom: 6}}>
                Pick a starter
              </div>
              <div style={{fontSize: 12, color: colors.textDim, marginBottom: 12}}>
                One tap to create a schedule. Tune the days, time and area afterwards.
              </div>
              {templateCards}
            </DashCard>
          </>
        )}
        {schedules.map((s, i) => scheduleCard(s, i))}
      </div>
    );
  }

  // Desktop: weekly grid + sub-cards
  return (
    <div style={{display: 'flex', flexDirection: 'column', gap: 16}}>
      {pageHeader}
      {/* Weekly grid */}
      <DashCard>
        <div style={{display: 'grid', gridTemplateColumns: '48px repeat(7, 1fr)', gap: 6}}>
          <div/>
          {DAYS.map(d => (
            <div key={d} style={{textAlign: 'center', fontSize: 11, color: colors.textDim, fontWeight: 600, padding: '2px 0 10px'}}>
              <div>{d}</div>
            </div>
          ))}
          {hours.map(h => (
            <div key={h} style={{display: 'contents'}}>
              <div style={{fontSize: 10, color: colors.textMuted, textAlign: 'right', paddingRight: 6, paddingTop: 2}}>
                {h}:00
              </div>
              {DAYS.map((_, di) => {
                const run = gridRuns.find(r => r.day === di && h >= r.start && h < r.end);
                const isStart = run && run.start === h;
                return (
                  <div key={di} style={{
                    minHeight: 32,
                    background: run ? `${run.color}22` : 'rgba(255,255,255,0.02)',
                    borderRadius: isStart ? '8px 8px 0 0' : (run && run.end - 1 === h ? '0 0 8px 8px' : 0),
                    border: run ? `1px solid ${run.color}66` : `1px solid ${colors.border}`,
                    borderBottom: run && run.end - 1 !== h ? 'none' : undefined,
                    borderTop: run && !isStart ? 'none' : undefined,
                    padding: isStart ? '6px 8px' : 0,
                  }}>
                    {isStart && (
                      <>
                        <div style={{fontSize: 11, fontWeight: 700, color: run.color, lineHeight: 1.1}}>{run.zone}</div>
                        <div style={{fontSize: 10, color: colors.textDim, marginTop: 2}}>{run.start}:00 -- {run.end}:00</div>
                      </>
                    )}
                  </div>
                );
              })}
            </div>
          ))}
        </div>
      </DashCard>

      {/* Sub-cards */}
      <div style={{display: 'grid', gridTemplateColumns: '1fr 1fr 1fr', gap: 14}}>
        <DashCard>
          <div style={{
            fontSize: 11, color: colors.textMuted, marginBottom: 14,
            letterSpacing: '0.08em', textTransform: 'uppercase' as const, fontWeight: 600,
          }}>This week</div>
          <div style={{display: 'flex', alignItems: 'baseline', gap: 8}}>
            <div className="mn-num" style={{fontSize: 46, lineHeight: 1, color: colors.text}}>{activeCount}</div>
            <div style={{fontSize: 12, color: colors.textDim}}>active schedules</div>
          </div>
          <div style={{display: 'flex', gap: 4, marginTop: 12}}>
            {DAYS.map((d, i) => {
              const has = gridRuns.some(r => r.day === i);
              return (
                <div key={i} style={{
                  flex: 1, height: 24, borderRadius: 4,
                  background: has ? colors.accent : 'rgba(255,255,255,0.06)',
                  opacity: has ? 1 : 0.6,
                  display: 'flex', alignItems: 'center', justifyContent: 'center',
                  fontSize: 9, fontWeight: 700,
                  color: has ? '#0a1a10' : colors.textMuted,
                }}>
                  {d[0]}
                </div>
              );
            })}
          </div>
        </DashCard>

        <DashCard>
          <div style={{display: 'flex', alignItems: 'center', justifyContent: 'space-between', marginBottom: 12}}>
            <div style={{fontSize: 13, fontWeight: 600}}>Schedules</div>
            <ActionButton primary icon={<IconPlus size={14}/>} label="New run"
                     onClick={() => handleCreate()} disabled={loading}
                     style={{padding: '8px 14px', fontSize: 12}}/>
          </div>
          <div style={{fontSize: 13, color: colors.textDim, lineHeight: 1.6}}>
            {schedules.length === 0
              ? 'No schedules yet. Pick a starter below or create a custom run.'
              : `${schedules.length} schedule${schedules.length > 1 ? 's' : ''} configured.`}
          </div>
        </DashCard>

        <DashCard>
          <div style={{fontSize: 13, fontWeight: 600, marginBottom: 4}}>Rules</div>
          <div style={{fontSize: 11, color: colors.textMuted, marginBottom: 12}}>
            Built-in safety behaviors — always active.
          </div>
          {[
            {k: 'Rain-aware', on: true, hint: 'pause if rain detected'},
            {k: 'Auto-dock low', on: true, hint: 'return at <20% battery'},
          ].map(r => (
            <div key={r.k} style={{display: 'flex', alignItems: 'center', gap: 10, padding: '6px 0'}}>
              <span style={{
                fontSize: 10, fontWeight: 700, letterSpacing: '0.04em',
                color: r.on ? colors.accent : colors.textMuted,
                background: r.on ? colors.accentSoft : 'transparent',
                border: `1px solid ${r.on ? colors.accent : colors.border}`,
                borderRadius: 100, padding: '2px 10px', flexShrink: 0,
                textTransform: 'uppercase' as const,
              }}>
                {r.on ? 'On' : 'Off'}
              </span>
              <div style={{flex: 1}}>
                <div style={{fontSize: 12, fontWeight: 600}}>{r.k}</div>
                <div style={{fontSize: 10, color: colors.textMuted}}>{r.hint}</div>
              </div>
            </div>
          ))}
        </DashCard>
      </div>

      {/* Starter templates (empty state) */}
      {schedules.length === 0 && (
        <DashCard>
          <div style={{display: 'flex', alignItems: 'baseline', justifyContent: 'space-between', marginBottom: 12}}>
            <div>
              <div style={{fontSize: 14, fontWeight: 700}}>Pick a starter</div>
              <div style={{fontSize: 12, color: colors.textDim, marginTop: 2}}>
                One click to create a schedule. Tune the days, time and area afterwards.
              </div>
            </div>
          </div>
          {templateCards}
        </DashCard>
      )}

      {/* Detailed schedule cards */}
      {schedules.length > 0 && (
        <div style={{display: 'grid', gridTemplateColumns: 'repeat(auto-fill, minmax(320px, 1fr))', gap: 14}}>
          {schedules.map((s, i) => scheduleCard(s, i))}
        </div>
      )}
    </div>
  );
};

export default SchedulePage;
