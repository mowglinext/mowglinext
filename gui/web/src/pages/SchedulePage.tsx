import {App, Modal, Radio, Switch, Tag, TimePicker, Tooltip} from "antd";
import {useCallback, useEffect, useRef, useState} from "react";
import {useTranslation} from "react-i18next";
import {useSettings} from "../hooks/useSettings.ts";
import {useApi} from "../hooks/useApi.ts";
import {useIsMobile} from "../hooks/useIsMobile";
import {useMowingMap} from "../hooks/useMowingMap.ts";
import {useThemeMode} from "../theme/ThemeContext.tsx";
import {DashCard, ActionButton, IconPlus, FONT} from "../components/dashboard";
import {IrriSenseStatusChip} from "../components/schedule/IrriSenseStatusChip.tsx";
import dayjs from "dayjs";

interface Schedule {
  id: string;
  /** Stable map area id (MapArea.id); 0 / absent = every area (a plain Start). */
  areaId?: number;
  /** Display snapshot of the area name, kept so the label survives the area's removal. */
  areaName?: string;
  time: string;
  daysOfWeek: number[];
  enabled: boolean;
  createdAt: string;
  lastRun?: string;
  lastSkipReason?: string;
  lastSkippedAt?: string;
}

const DAY_KEYS = ["dayMon", "dayTue", "dayWed", "dayThu", "dayFri", "daySat", "daySun"] as const;
const DAY_LETTER_KEYS = ["letterSun", "letterMon", "letterTue", "letterWed", "letterThu", "letterFri", "letterSat"] as const;

/** HTTP status of a thrown API error, when it carries one. */
function errorStatus(e: unknown): number | undefined {
  if (typeof e === "object" && e !== null) {
    const status = (e as {status?: unknown}).status;
    return typeof status === "number" ? status : undefined;
  }
  return undefined;
}

/** Pull a human-readable message out of an unknown thrown API error. */
function errorMessage(e: unknown): string | undefined {
  if (e instanceof Error && e.message) return e.message;
  if (typeof e === "object" && e !== null) {
    const err = (e as {error?: {error?: string}; message?: string});
    return err.error?.error ?? err.message;
  }
  return undefined;
}

export const SchedulePage = () => {
  const {t} = useTranslation();
  const {colors} = useThemeMode();
  const {settings} = useSettings();
  const batteryLow = settings.battery_low_percent;
  const rainMode = settings.rain_mode;
  const rainLabels = ['rainModeIgnoreLabel', 'rainModeDockLabel', 'rainModeDockUntilDryLabel', 'rainModePauseAutoLabel'];
  const guiApi = useApi();
  const {notification, modal} = App.useApp();
  const isMobile = useIsMobile();
  const [schedules, setSchedules] = useState<Schedule[]>([]);
  const [loading, setLoading] = useState(false);
  const fetchedRef = useRef(false);
  const map = useMowingMap();
  // The schedule whose area dialog is open, and the choice made in it (0 = all areas).
  const [areaDialogId, setAreaDialogId] = useState<string | null>(null);
  const [areaChoice, setAreaChoice] = useState(0);

  const fetchSchedules = useCallback(async () => {
    try {
      const response = await guiApi.request<{ schedules: Schedule[] }>({
        path: "/schedules", method: "GET", format: "json",
      });
      setSchedules(response.data.schedules ?? []);
    } catch {
      notification.error({message: t('schedulePage.failedToLoad')});
    }
  }, [guiApi, notification, t]);

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
        body: {areaId: 0, time: "09:00", daysOfWeek: [1, 2, 3, 4, 5], enabled: false, ...body},
        format: "json",
      });
      await fetchSchedules();
      // New schedules are created disabled for safety; tell the operator so an
      // untouched starter template doesn't silently never run.
      notification.info({
        message: t('schedulePage.createdDisabledTitle'),
        description: t('schedulePage.createdDisabledBody'),
      });
    } catch (e) {
      notification.error({message: t('schedulePage.failedToCreate'), description: errorMessage(e)});
    } finally {
      setLoading(false);
    }
  };

  const STARTER_TEMPLATES: { name: string; subtitle: string; body: Partial<Schedule>; icon: string }[] = [
    {
      name: t('schedulePage.templateWeekendName'),
      subtitle: t('schedulePage.templateWeekendSubtitle'),
      body: {time: "10:00", daysOfWeek: [6]},
      icon: "sun",
    },
    {
      name: t('schedulePage.templateStealthName'),
      subtitle: t('schedulePage.templateStealthSubtitle'),
      body: {time: "06:00", daysOfWeek: [1, 3, 5]},
      icon: "moon",
    },
    {
      name: t('schedulePage.templateDailyName'),
      subtitle: t('schedulePage.templateDailySubtitle'),
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
              {tpl.icon === 'sun' ? t('schedulePage.tagWeekly') : tpl.icon === 'moon' ? t('schedulePage.tagOffHours') : t('schedulePage.tagEveryDay')}
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
    } catch (e) {
      // 409 = the change would make two enabled schedules overlap; the server's
      // message names the clashing start time.
      notification.error({
        message: errorStatus(e) === 409 ? t('schedulePage.overlapTitle') : t('schedulePage.failedToUpdate'),
        description: errorMessage(e),
      });
    }
  };

  // Mowing areas the operator can pick (navigation areas are never mowed).
  const areaOptions = (map.working_area ?? [])
    .filter(a => typeof a.id === "number" && a.id > 0)
    .map(a => ({id: a.id as number, name: a.name || t('schedulePage.areaFallback', {id: a.id})}));
  const mapLoaded = map.working_area !== undefined;

  // What a schedule's area chip says. `missing` = the map is loaded and no longer has the area.
  const areaLabel = (s: Schedule): {text: string; missing: boolean} => {
    if (!s.areaId) return {text: t('schedulePage.appliesToAllAreas'), missing: false};
    const live = areaOptions.find(a => a.id === s.areaId);
    if (live) return {text: live.name, missing: false};
    const name = s.areaName || t('schedulePage.areaFallback', {id: s.areaId});
    return mapLoaded
      ? {text: t('schedulePage.areaRemoved', {name}), missing: true}
      : {text: name, missing: false};
  };

  const openAreaDialog = (sched: Schedule) => {
    setAreaChoice(sched.areaId ?? 0);
    setAreaDialogId(sched.id);
  };

  const saveAreaChoice = () => {
    const sched = schedules.find(s => s.id === areaDialogId);
    setAreaDialogId(null);
    if (!sched || (sched.areaId ?? 0) === areaChoice) return;
    const picked = areaOptions.find(a => a.id === areaChoice);
    void handleUpdate({...sched, areaId: areaChoice, areaName: picked?.name ?? ""});
  };

  const handleDelete = async (id: string) => {
    try {
      await guiApi.request({path: `/schedules/${id}`, method: "DELETE", format: "json"});
      await fetchSchedules();
    } catch (e) {
      notification.error({message: t('schedulePage.failedToDelete'), description: errorMessage(e)});
    }
  };

  const confirmDelete = (id: string) => {
    modal.confirm({
      title: t('schedulePage.deleteScheduleTitle'),
      content: t('schedulePage.deleteScheduleConfirm'),
      okText: t('schedulePage.delete'),
      okType: "danger",
      cancelText: t('schedulePage.cancel'),
      onOk: () => handleDelete(id),
    });
  };

  const toggleDay = (sched: Schedule, day: number) => {
    const isRemoving = sched.daysOfWeek.includes(day);
    // A schedule with zero days is meaningless and the backend rejects it with
    // a generic 400. Block un-toggling the last remaining day client-side.
    if (isRemoving && sched.daysOfWeek.length <= 1) return;
    const days = isRemoving
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
        area: sched.areaId ? areaLabel(sched).text : null,
        day: dayIndex === 0 ? 6 : dayIndex - 1, // convert Sun=0..Sat=6 to Mon=0..Sun=6
        start: startH,
        end: Math.min(startH + 1, 20),
        color: schedColors[si % schedColors.length],
        enabled: sched.enabled,
      }));
  });

  const activeCount = schedules.filter(s => s.enabled).length;

  // Schedule card for each schedule (mobile + bottom section on desktop)
  const scheduleCard = (sched: Schedule, idx: number) => {
    const color = sched.enabled ? schedColors[idx % schedColors.length] : colors.textSecondary;
    return (
      <DashCard key={sched.id} style={{display: 'flex', flexDirection: 'column', gap: 12}}>
        <div style={{display: 'flex', alignItems: 'center', gap: 12}}>
          <div style={{width: 4, height: 32, borderRadius: 2, background: color}}/>
          <Switch
            aria-label={t("schedulePage.enableSchedule", {index: idx + 1})}
            checked={sched.enabled}
            onChange={(checked) => handleUpdate({...sched, enabled: checked})}
          />
          <Tag color={sched.enabled ? "success" : "default"}>{t(sched.enabled ? "schedulePage.on" : "schedulePage.inactive")}</Tag>
          {/* One area or all: "all" is a plain Start, a specific area goes through
              start_in_area (see scheduler.go). Clicking opens the area picker. */}
          <Tag
            role="button"
            tabIndex={0}
            aria-label={t('schedulePage.chooseAreaAria', {index: idx + 1})}
            color={areaLabel(sched).missing ? "warning" : undefined}
            style={{marginLeft: 'auto', cursor: 'pointer', marginInlineEnd: 0}}
            onClick={() => openAreaDialog(sched)}
            onKeyDown={(e) => {
              if (e.key === 'Enter' || e.key === ' ') { e.preventDefault(); openAreaDialog(sched); }
            }}
          >
            {areaLabel(sched).text}
          </Tag>
        </div>
        <div style={{fontSize: 12, color: colors.textSecondary}}>{t("schedulePage.autoSaveHint")}</div>
        <TimePicker
          aria-label={t("schedulePage.startTime")}
          value={dayjs(sched.time, "HH:mm")}
          format="HH:mm"
          onChange={(val) => { if (val) handleUpdate({...sched, time: val.format("HH:mm")}); }}
          style={{width: '100%'}}
          size="large"
        />
        <div style={{display: 'flex', gap: 6, justifyContent: 'space-between'}}>
          {DAY_LETTER_KEYS.map((letterKey, i) => {
            const isActive = sched.daysOfWeek.includes(i);
            const isLastDay = isActive && sched.daysOfWeek.length <= 1;
            const dayButton = (
              <button
                key={i}
                onClick={() => toggleDay(sched, i)}
                aria-label={t(`schedulePage.${DAY_KEYS[(i + 6) % 7]}`)}
                aria-pressed={isActive}
                disabled={isLastDay}
                style={{
                  width: 44, height: 44, borderRadius: '50%',
                  border: `1.5px solid ${isActive ? color : colors.border}`,
                  background: isActive ? colors.bgElevated : 'transparent',
                  color: isActive ? color : colors.textSecondary,
                  fontSize: 13, fontWeight: 600, cursor: isLastDay ? 'not-allowed' : 'pointer',
                  transition: 'all 0.15s', padding: 0, fontFamily: FONT,
                }}
              >
                {t(`schedulePage.${letterKey}`)}
              </button>
            );
            return isLastDay
              ? <Tooltip key={i} title={t('schedulePage.lastDayTooltip')}>{dayButton}</Tooltip>
              : dayButton;
          })}
        </div>
        <div style={{
          display: 'flex', justifyContent: 'space-between', alignItems: 'center',
          borderTop: `1px solid ${colors.borderSubtle}`, paddingTop: 8,
        }}>
          <span style={{fontSize: 12, color: colors.textSecondary, display: 'flex', flexDirection: 'column', gap: 2}}>
            <span>
              {t('schedulePage.lastRun', {value: sched.lastRun ? dayjs(sched.lastRun).format("YYYY-MM-DD HH:mm") : t('schedulePage.never')})}
            </span>
            {sched.lastSkipReason && (
              <span style={{fontSize: 11, color: colors.sky}} data-testid="schedule-last-skip">
                {t('schedulePage.lastSkipped', {
                  value: sched.lastSkippedAt ? dayjs(sched.lastSkippedAt).format("YYYY-MM-DD HH:mm") : '',
                  reason: sched.lastSkipReason,
                })}
              </span>
            )}
          </span>
          <button
            onClick={() => confirmDelete(sched.id)}
            style={{
              background: colors.dangerBg, color: colors.danger,
              border: 'none', borderRadius: 8, padding: '6px 12px',
              fontSize: 12, fontWeight: 600, cursor: 'pointer', fontFamily: FONT,
            }}
          >
            {t('schedulePage.delete')}
          </button>
        </div>
      </DashCard>
    );
  };

  const dialogSchedule = schedules.find(s => s.id === areaDialogId);
  const areaDialog = (
    <Modal
      open={dialogSchedule !== undefined}
      title={t('schedulePage.areaDialogTitle', {index: dialogSchedule ? schedules.indexOf(dialogSchedule) + 1 : 0})}
      okText={t('schedulePage.saveArea')}
      cancelText={t('schedulePage.cancel')}
      onOk={saveAreaChoice}
      onCancel={() => setAreaDialogId(null)}
      destroyOnHidden
    >
      <div style={{fontSize: 12, color: colors.textSecondary, marginBottom: 12}}>{t('schedulePage.areaDialogHint')}</div>
      <Radio.Group
        value={areaChoice}
        onChange={(e) => setAreaChoice(e.target.value as number)}
        style={{display: 'flex', flexDirection: 'column', gap: 10}}
      >
        <Radio value={0}>
          {t('schedulePage.allAreasOption')}
          <div style={{fontSize: 11, color: colors.textSecondary}}>{t('schedulePage.allAreasOptionHint')}</div>
        </Radio>
        {areaOptions.map(a => <Radio key={a.id} value={a.id}>{a.name}</Radio>)}
      </Radio.Group>
      {areaOptions.length === 0 && (
        <div style={{fontSize: 12, color: colors.textSecondary, marginTop: 12}}>{t('schedulePage.noAreasHint')}</div>
      )}
      {dialogSchedule && areaLabel(dialogSchedule).missing && (
        <div style={{fontSize: 12, color: colors.amber, marginTop: 12}}>{t('schedulePage.areaRemovedWarning')}</div>
      )}
    </Modal>
  );

  const pageHeader = (
    <div>
      <div style={{
        fontSize: 11, color: colors.textMuted, fontWeight: 600,
        letterSpacing: '0.12em', textTransform: 'uppercase' as const,
      }}>
        {t('schedulePage.planning')}
      </div>
      <div className="mn-display" style={{
        fontSize: isMobile ? 30 : 40, color: colors.text,
        lineHeight: 1.05, marginTop: 4, letterSpacing: '-0.02em',
      }}>
        {t('schedulePage.scheduledMows')}
      </div>
      <div style={{marginTop: 8}}>
        <IrriSenseStatusChip/>
      </div>
    </div>
  );

  if (isMobile) {
    return (
      <div style={{display: 'flex', flexDirection: 'column', gap: 12, paddingBottom: 8}}>
        <div style={{display: 'flex', justifyContent: 'space-between', alignItems: 'flex-end', gap: 12}}>
          {pageHeader}
          <ActionButton primary icon={<IconPlus size={14}/>} label={t('schedulePage.newRun')} onClick={() => handleCreate()} disabled={loading}/>
        </div>
        {schedules.length === 0 && (
          <>
            <DashCard style={{padding: 18}}>
              <div style={{fontSize: 14, fontWeight: 700, marginBottom: 6}}>
                {t('schedulePage.pickAStarter')}
              </div>
              <div style={{fontSize: 12, color: colors.textDim, marginBottom: 12}}>
                {t('schedulePage.pickAStarterDescTap')}
              </div>
              {templateCards}
            </DashCard>
          </>
        )}
        {schedules.map((s, i) => scheduleCard(s, i))}
        {areaDialog}
      </div>
    );
  }

  // Desktop: weekly grid + sub-cards
  return (
    <div style={{display: 'flex', flexDirection: 'column', gap: 16}}>
      {pageHeader}
      {areaDialog}
      {/* Weekly grid */}
      <DashCard>
        <div style={{display: 'grid', gridTemplateColumns: '48px repeat(7, 1fr)', gap: 6}}>
          <div/>
          {DAY_KEYS.map(dKey => (
            <div key={dKey} style={{textAlign: 'center', fontSize: 11, color: colors.textDim, fontWeight: 600, padding: '2px 0 10px'}}>
              <div>{t(`schedulePage.${dKey}`)}</div>
            </div>
          ))}
          {hours.map(h => (
            <div key={h} style={{display: 'contents'}}>
              <div style={{fontSize: 10, color: colors.textMuted, textAlign: 'right', paddingRight: 6, paddingTop: 2}}>
                {h}:00
              </div>
              {DAY_KEYS.map((_, di) => {
                const run = gridRuns.find(r => r.day === di && h >= r.start && h < r.end);
                const isStart = run && run.start === h;
                return (
                  <div key={di} style={{
                    minHeight: 32,
                    background: run?.enabled ? `${run.color}22` : colors.bgSubtle,
                    borderRadius: isStart ? '8px 8px 0 0' : (run && run.end - 1 === h ? '0 0 8px 8px' : 0),
                    borderStyle: run && !run.enabled ? 'dashed' : 'solid',
                    borderColor: run ? run.enabled ? run.color : colors.muted : colors.border,
                    borderWidth: 1,
                    borderBottomWidth: run && run.end - 1 !== h ? 0 : 1,
                    borderTopWidth: run && !isStart ? 0 : 1,
                    padding: isStart ? '6px 8px' : 0,
                  }}>
                    {isStart && (
                      <>
                        <div style={{fontSize: 11, fontWeight: 700, color: run.enabled ? run.color : colors.textSecondary, lineHeight: 1.1}}>{run.enabled ? (run.area ? t('schedulePage.blockStartArea', {area: run.area}) : t('schedulePage.blockStartHint')) : t('schedulePage.inactive')}</div>
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
          }}>{t('schedulePage.thisWeek')}</div>
          <div style={{display: 'flex', alignItems: 'baseline', gap: 8}}>
            <div className="mn-num" style={{fontSize: 46, lineHeight: 1, color: colors.text}}>{activeCount}</div>
            <div style={{fontSize: 12, color: colors.textDim}}>{t('schedulePage.activeSchedules')}</div>
          </div>
          <div style={{display: 'flex', gap: 4, marginTop: 12}}>
            {DAY_KEYS.map((dKey, i) => {
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
                  {t(`schedulePage.${dKey}`).charAt(0)}
                </div>
              );
            })}
          </div>
        </DashCard>

        <DashCard>
          <div style={{display: 'flex', alignItems: 'center', justifyContent: 'space-between', marginBottom: 12}}>
            <div style={{fontSize: 13, fontWeight: 600}}>{t('schedulePage.schedules')}</div>
            <ActionButton primary icon={<IconPlus size={14}/>} label={t('schedulePage.newRun')}
                     onClick={() => handleCreate()} disabled={loading}
                     style={{padding: '8px 14px', fontSize: 12}}/>
          </div>
          <div style={{fontSize: 13, color: colors.textDim, lineHeight: 1.6}}>
            {schedules.length === 0
              ? t('schedulePage.noSchedulesYet')
              : t('schedulePage.schedulesConfigured', {count: schedules.length})}
          </div>
        </DashCard>

        <DashCard>
          <div style={{fontSize: 13, fontWeight: 600, marginBottom: 4}}>{t('schedulePage.rules')}</div>
          <div style={{fontSize: 11, color: colors.textMuted, marginBottom: 12}}>
            {t('schedulePage.rulesDescription')}
          </div>
          {[
            {k: t('schedulePage.ruleRainAware'), on: Number.isInteger(rainMode) && rainLabels[rainMode] ? rainMode > 0 : null, hint: Number.isInteger(rainMode) && rainLabels[rainMode] ? t(`settingsRain.${rainLabels[rainMode]}`) : t('schedulePage.unknown')},
            {k: t('schedulePage.ruleAutoDockLow'), on: typeof batteryLow === 'number' ? true : null, hint: typeof batteryLow === 'number' ? t('schedulePage.ruleAutoDockLowHint', {percent: batteryLow}) : t('schedulePage.unknown')},
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
                {r.on === null ? t('schedulePage.unknown') : r.on ? t('schedulePage.on') : t('schedulePage.off')}
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
              <div style={{fontSize: 14, fontWeight: 700}}>{t('schedulePage.pickAStarter')}</div>
              <div style={{fontSize: 12, color: colors.textDim, marginTop: 2}}>
                {t('schedulePage.pickAStarterDescClick')}
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
