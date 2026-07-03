import {useMemo} from "react";
import {useTranslation} from "react-i18next";
import {useThemeMode} from "../theme/ThemeContext.tsx";
import {useIsMobile} from "../hooks/useIsMobile";

/**
 * GitHub-style contribution-graph rendered against mowing sessions.
 *
 * Each cell is one day; the cell intensity is the total mowing distance
 * for that day. We render the last 52 weeks ending today, padded so the
 * leftmost column is a Sunday.
 */

export interface SessionLike {
  start_time: string;
  duration_sec: number;
  distance_meters: number;
  coverage_percent: number;
  status: string;
}

interface YearOfLawnProps {
  sessions: SessionLike[];
}

const DAY_MS = 24 * 60 * 60 * 1000;

function startOfDay(date: Date): Date {
  return new Date(date.getFullYear(), date.getMonth(), date.getDate());
}

function dateKey(date: Date): string {
  return `${date.getFullYear()}-${date.getMonth()}-${date.getDate()}`;
}

function intensity(km: number, max: number): number {
  if (km <= 0 || max <= 0) return 0;
  // 4 buckets so the colour ramp has clear stops, like GH's contribution graph.
  const ratio = km / max;
  if (ratio < 0.1) return 1;
  if (ratio < 0.4) return 2;
  if (ratio < 0.75) return 3;
  return 4;
}

export function YearOfLawn({sessions}: YearOfLawnProps) {
  const {colors} = useThemeMode();
  const {t} = useTranslation();
  const isMobile = useIsMobile();
  // The full year SVG (~810px) always horizontal-scroll-clips on phones; on
  // mobile we render only the most recent weeks. Summary stats stay over the
  // full 52-week window.
  const weeksToShow = isMobile ? 20 : 52;

  const {grid, weeks, totalKm, activeDays, streak, monthLabels} = useMemo(() => {
    const today = startOfDay(new Date());
    // 52 weeks ending today, leftmost column starts on a Sunday.
    const endDay = today.getDay(); // 0..6
    const startOffsetDays = 52 * 7 - 1 - endDay;
    const start = new Date(today.getTime() - startOffsetDays * DAY_MS);

    // Bucket distance per day.
    const perDay = new Map<string, number>();
    sessions.forEach(s => {
      const d = startOfDay(new Date(s.start_time));
      if (d < start || d > today) return;
      const km = s.distance_meters / 1000;
      perDay.set(dateKey(d), (perDay.get(dateKey(d)) ?? 0) + km);
    });

    const days = 52 * 7;
    const cells: { km: number; date: Date }[] = [];
    let totalKm = 0;
    let activeDays = 0;
    for (let i = 0; i < days; i++) {
      const d = new Date(start.getTime() + i * DAY_MS);
      const km = perDay.get(dateKey(d)) ?? 0;
      cells.push({km, date: d});
      totalKm += km;
      if (km > 0) activeDays += 1;
    }

    // Streak (consecutive active days ending today)
    let streak = 0;
    for (let i = cells.length - 1; i >= 0; i--) {
      if (cells[i].km > 0) streak += 1;
      else break;
    }

    // 52 columns of 7 days each
    const weeks: { km: number; date: Date }[][] = [];
    for (let c = 0; c < 52; c++) {
      const col: { km: number; date: Date }[] = [];
      for (let r = 0; r < 7; r++) col.push(cells[c * 7 + r]);
      weeks.push(col);
    }

    // Month labels (column index where a new month starts)
    const monthLabels: { col: number; label: string }[] = [];
    let prevMonth = -1;
    weeks.forEach((col, ci) => {
      const m = col[0].date.getMonth();
      if (m !== prevMonth) {
        const isoMonth = col[0].date.toLocaleString(undefined, {month: 'short'});
        monthLabels.push({col: ci, label: isoMonth});
        prevMonth = m;
      }
    });

    return {grid: cells, weeks, totalKm, activeDays, streak, monthLabels};
  }, [sessions]);

  const maxKm = grid.reduce((m, c) => Math.max(m, c.km), 0.001);

  // On mobile, render only the trailing `weeksToShow` columns. Re-index the
  // month labels into the sliced window.
  const colOffset = Math.max(0, weeks.length - weeksToShow);
  const displayWeeks = weeks.slice(colOffset);
  const displayMonthLabels = monthLabels
    .filter(m => m.col >= colOffset)
    .map(m => ({...m, col: m.col - colOffset}));

  const cellSize = 12;
  const cellGap = 3;
  const colWidth = cellSize + cellGap;
  const totalWidth = displayWeeks.length * colWidth + 30;
  const totalHeight = 7 * colWidth + 24;

  const intensityColors = [
    colors.bgElevated,
    `${colors.accent}33`,
    `${colors.accent}66`,
    `${colors.accent}aa`,
    colors.accent,
  ];

  return (
    <div>
      <div style={{display: 'flex', gap: 24, marginBottom: 14, flexWrap: 'wrap'}}>
        <div>
          <div style={{fontSize: 11, color: colors.textMuted, letterSpacing: '0.06em', textTransform: 'uppercase'}}>
            {t('yearOfLawn.yearOfLawn')}
          </div>
          <div style={{fontSize: 22, fontWeight: 700, color: colors.text, marginTop: 2, letterSpacing: '-0.02em'}}>
            {totalKm.toFixed(1)} km <span style={{fontSize: 13, color: colors.textDim, fontWeight: 500}}>{t('yearOfLawn.last52Weeks')}</span>
          </div>
        </div>
        <div>
          <div style={{fontSize: 11, color: colors.textMuted, letterSpacing: '0.06em', textTransform: 'uppercase'}}>
            {t('yearOfLawn.activeDays')}
          </div>
          <div style={{fontSize: 22, fontWeight: 700, color: colors.text, marginTop: 2, letterSpacing: '-0.02em'}}>
            {activeDays}
          </div>
        </div>
        <div>
          <div style={{fontSize: 11, color: colors.textMuted, letterSpacing: '0.06em', textTransform: 'uppercase'}}>
            {t('yearOfLawn.currentStreak')}
          </div>
          <div style={{fontSize: 22, fontWeight: 700, color: streak > 0 ? colors.accent : colors.text, marginTop: 2, letterSpacing: '-0.02em'}}>
            {t('yearOfLawn.dayCount', {count: streak})}
          </div>
        </div>
      </div>

      <div style={{overflowX: 'auto'}}>
        <svg width={totalWidth} height={totalHeight} style={{display: 'block'}}>
          {/* month labels */}
          {displayMonthLabels.map((m, i) => (
            <text key={i}
                  x={m.col * colWidth + 28}
                  y={12}
                  fontSize={9}
                  fill={colors.textMuted}>
              {m.label}
            </text>
          ))}

          {/* day-of-week labels */}
          {(['dayMon', 'dayWed', 'dayFri'] as const).map((dKey, i) => (
            <text key={dKey}
                  x={0}
                  y={18 + (i * 2 + 1) * colWidth + 8}
                  fontSize={9}
                  fill={colors.textMuted}>
              {t(`yearOfLawn.${dKey}`)}
            </text>
          ))}

          {/* cells */}
          {displayWeeks.map((col, ci) =>
            col.map((cell, ri) => {
              const lvl = intensity(cell.km, maxKm);
              return (
                <rect
                  key={`${ci}-${ri}`}
                  x={ci * colWidth + 28}
                  y={ri * colWidth + 18}
                  width={cellSize}
                  height={cellSize}
                  rx={2}
                  fill={intensityColors[lvl]}
                  stroke={lvl === 0 ? colors.borderSubtle : 'none'}
                >
                  <title>{t('yearOfLawn.cellTooltip', {date: cell.date.toLocaleDateString(), km: cell.km.toFixed(2)})}</title>
                </rect>
              );
            })
          )}
        </svg>
      </div>

      {/* legend */}
      <div style={{
        display: 'flex', alignItems: 'center', gap: 6,
        fontSize: 10, color: colors.textMuted, marginTop: 6, justifyContent: 'flex-end',
      }}>
        <span>{t('yearOfLawn.less')}</span>
        {intensityColors.map((c, i) => (
          <span key={i} style={{
            width: 10, height: 10, borderRadius: 2,
            background: c,
            border: i === 0 ? `1px solid ${colors.borderSubtle}` : 'none',
          }}/>
        ))}
        <span>{t('yearOfLawn.more')}</span>
      </div>
    </div>
  );
}
