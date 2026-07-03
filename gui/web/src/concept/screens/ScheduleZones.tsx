import {motion} from "framer-motion";
import {useTranslation} from "react-i18next";
import {Plus, Sun, Moon, Calendar, Cloud, Droplets} from "lucide-react";

import {GlassCard} from "../components/GlassCard";
import {NoiseTexture} from "../components/NoiseTexture";
import {staggerParent, riseFade, springSnap} from "../motion";
import {useViewport} from "../useViewport";

/**
 * Schedule & Zones. Two main blocks:
 *   - Weekly grid (Mon-Sun, hours of the day, runs as colored blocks)
 *   - Zone cards with per-zone scheduling
 *
 * Mobile: stacked, week grid scrolls horizontally.
 * Desktop: 2-column with week grid taking the larger half.
 */

interface ScheduledRun {
  id: string;
  zone: string;
  zoneAccent: "lime" | "cyan" | "amber" | "rose";
  day: number;   // 0..6 mon..sun
  start: number; // hour 0..23
  end: number;
}

const RUNS: ScheduledRun[] = [
  {id: "r1", zone: "scheduleZones.zoneSouthGarden", zoneAccent: "lime",  day: 1, start: 7,  end: 8.5},
  {id: "r2", zone: "scheduleZones.zoneNorthGarden", zoneAccent: "cyan",  day: 1, start: 18, end: 19},
  {id: "r3", zone: "scheduleZones.zoneFrontLawn",   zoneAccent: "amber", day: 2, start: 7,  end: 7.5},
  {id: "r4", zone: "scheduleZones.zoneSouthGarden", zoneAccent: "lime",  day: 4, start: 6,  end: 8},
  {id: "r5", zone: "scheduleZones.zoneSidePath",    zoneAccent: "rose",  day: 5, start: 18, end: 18.5},
  {id: "r6", zone: "scheduleZones.zoneNorthGarden", zoneAccent: "cyan",  day: 6, start: 9,  end: 11},
];

const DAY_KEYS = [
  "scheduleZones.dayMon", "scheduleZones.dayTue", "scheduleZones.dayWed",
  "scheduleZones.dayThu", "scheduleZones.dayFri", "scheduleZones.daySat", "scheduleZones.daySun",
];
const HOURS = [5, 7, 9, 11, 13, 15, 17, 19, 21];

const ACCENT: Record<ScheduledRun["zoneAccent"], string> = {
  lime:  "var(--lime)",
  cyan:  "var(--aurora-cyan)",
  amber: "var(--amber)",
  rose:  "var(--rose)",
};

export function ScheduleZones() {
  const {t} = useTranslation();
  const vp = useViewport();
  return (
    <div style={{
      position: "relative", minHeight: "100dvh",
      padding: vp.isAtLeastTablet ? "36px 36px 56px" : "max(20px, env(safe-area-inset-top)) 18px 110px",
      overflow: "hidden",
    }}>
      <div aria-hidden style={{
        position: "absolute", inset: -100, pointerEvents: "none",
        background: "var(--grad-aurora)",
        filter: "blur(10px)", zIndex: 0,
      }}/>
      <NoiseTexture/>

      <motion.div
        variants={staggerParent(0.06, 0.08)}
        initial="hidden" animate="show"
        style={{
          position: "relative", zIndex: 1,
          maxWidth: vp.isAtLeastTablet ? 1280 : 560, margin: "0 auto",
        }}
      >
        {/* header */}
        <motion.header variants={riseFade} style={{
          display: "flex", alignItems: "flex-end", justifyContent: "space-between",
          marginBottom: vp.isAtLeastTablet ? 26 : 18, gap: 12,
        }}>
          <div>
            <div style={{
              fontSize: 11, color: "var(--ink-3)",
              letterSpacing: "0.08em", textTransform: "uppercase",
              fontWeight: 700,
            }}>
              {t('scheduleZones.title')}
            </div>
            <h1 className="display" style={{
              fontSize: vp.isAtLeastTablet ? 36 : 28, marginTop: 4,
              fontWeight: 700, letterSpacing: "-0.025em", lineHeight: 1.1,
            }}>
              <em style={{fontStyle: "italic", color: "var(--lime)"}}>{t('scheduleZones.headlineCount')}</em> {t('scheduleZones.headlineRest')}
            </h1>
          </div>
          <motion.button
            whileHover={{y: -1}} whileTap={{scale: 0.96}}
            transition={springSnap}
            style={{
              display: "flex", alignItems: "center", gap: 8,
              padding: "11px 16px",
              background: "var(--grad-primary)",
              color: "var(--bg-deep)",
              borderRadius: 12, fontWeight: 700, fontSize: 13,
              boxShadow: "0 12px 24px -8px rgba(124,255,178,0.5), inset 0 1px 0 rgba(255,255,255,0.32)",
            }}
          >
            <Plus size={16} strokeWidth={2.4}/>
            {t('scheduleZones.newMow')}
          </motion.button>
        </motion.header>

        <div style={{
          display: "grid",
          gridTemplateColumns: vp.isAtLeastTablet ? "1.7fr 1fr" : "1fr",
          gap: 22,
        }}>
          {/* ── Week grid ── */}
          <motion.div variants={riseFade}>
            <GlassCard padding={0}>
              <div style={{padding: "20px 22px 12px"}}>
                <div style={{
                  fontSize: 11, color: "var(--ink-3)",
                  letterSpacing: "0.08em", textTransform: "uppercase",
                  fontWeight: 700,
                }}>
                  {t('scheduleZones.thisWeek')}
                </div>
                <div className="display" style={{
                  fontSize: 20, fontWeight: 700, marginTop: 4,
                  letterSpacing: "-0.02em",
                }}>
                  {t('scheduleZones.thisWeekSummary')}
                </div>
              </div>

              <div style={{
                padding: "0 12px 18px", overflowX: "auto",
              }} className="scrollbar-thin">
                <WeekGrid runs={RUNS}/>
              </div>

              {/* Rules row */}
              <div style={{
                display: "flex", gap: 12, padding: "14px 22px 18px",
                borderTop: "1px solid var(--border-soft)",
              }}>
                <RulePill icon={<Cloud size={14}/>}    label={t('scheduleZones.ruleAntiRain')} active/>
                <RulePill icon={<Sun size={14}/>}      label={t('scheduleZones.ruleAntiHeat')} active/>
                <RulePill icon={<Moon size={14}/>}     label={t('scheduleZones.ruleNightMow')} active={false}/>
              </div>
            </GlassCard>
          </motion.div>

          {/* ── Zones column ── */}
          <motion.div variants={riseFade} style={{
            display: "flex", flexDirection: "column", gap: 14,
          }}>
            <GlassCard padding={20}>
              <div style={{
                fontSize: 11, color: "var(--ink-3)",
                letterSpacing: "0.08em", textTransform: "uppercase",
                fontWeight: 700, marginBottom: 14,
              }}>
                {t('scheduleZones.yourZones')}
              </div>
              <div style={{display: "flex", flexDirection: "column", gap: 10}}>
                <ZoneRow name={t('scheduleZones.zoneSouthGarden')}     accent="lime"  area={184} weekly={4} duration="≈ 2h 10min"/>
                <ZoneRow name={t('scheduleZones.zoneNorthGarden')}    accent="cyan"  area={116} weekly={2} duration="≈ 55min"/>
                <ZoneRow name={t('scheduleZones.zoneFrontLawn')} accent="amber" area={92}  weekly={1} duration="≈ 35min"/>
                <ZoneRow name={t('scheduleZones.zoneSidePath')} accent="rose"  area={38}  weekly={1} duration="≈ 15min"/>
              </div>
            </GlassCard>

            <GlassCard padding={18}>
              <div style={{
                display: "flex", alignItems: "center", gap: 10, marginBottom: 12,
              }}>
                <Droplets size={14} color="var(--aurora-cyan)" strokeWidth={2.4}/>
                <span style={{
                  fontSize: 11, color: "var(--ink-2)",
                  letterSpacing: "0.08em", textTransform: "uppercase", fontWeight: 700,
                }}>
                  {t('scheduleZones.weatherThisWeek')}
                </span>
              </div>
              <div style={{
                display: "grid", gridTemplateColumns: "repeat(7, 1fr)", gap: 6,
              }}>
                {DAY_KEYS.map((d, i) => {
                  const temp = [22, 23, 21, 19, 18, 17, 19][i];
                  const rainy = i === 3 || i === 4;
                  return (
                    <div key={d} style={{
                      display: "flex", flexDirection: "column", alignItems: "center", gap: 6,
                      padding: "10px 4px",
                      background: rainy ? "rgba(243,168,92,0.10)" : "rgba(255,255,255,0.03)",
                      border: `1px solid ${rainy ? "rgba(243,168,92,0.32)" : "var(--border-soft)"}`,
                      borderRadius: 10,
                    }}>
                      <div style={{fontSize: 9, color: "var(--ink-3)", fontWeight: 700, letterSpacing: "0.06em"}}>
                        {t(d)}
                      </div>
                      {rainy ? <Cloud size={16} color="var(--amber)"/> : <Sun size={16} color="var(--lime)"/>}
                      <div className="mono" style={{
                        fontSize: 11, fontWeight: 700,
                        color: rainy ? "var(--amber)" : "var(--ink)",
                      }}>
                        {temp}°
                      </div>
                    </div>
                  );
                })}
              </div>
            </GlassCard>
          </motion.div>
        </div>
      </motion.div>
    </div>
  );
}

// ─────────────────────────────────────────────────────────────────────
// Week grid
// ─────────────────────────────────────────────────────────────────────

function WeekGrid({runs}: {runs: ScheduledRun[]}) {
  const {t} = useTranslation();
  const startHour = HOURS[0];
  const endHour   = HOURS[HOURS.length - 1];
  const totalHours = endHour - startHour;
  const dayColWidth = 90;
  const rowHeight = 220;

  return (
    <div style={{
      display: "grid",
      gridTemplateColumns: `48px repeat(7, minmax(${dayColWidth}px, 1fr))`,
      gridTemplateRows: "auto",
    }}>
      <div/>
      {DAY_KEYS.map((d, i) => (
        <div key={d} style={{
          padding: "6px 0 12px", textAlign: "center",
          fontSize: 10, color: "var(--ink-2)",
          letterSpacing: "0.08em", textTransform: "uppercase", fontWeight: 700,
        }}>
          {t(d)}
          <div style={{
            fontSize: 16, fontWeight: 700, color: i === 0 ? "var(--lime)" : "var(--ink)",
            marginTop: 4, fontFamily: "var(--font-display)", letterSpacing: "-0.01em",
          }}>
            {[18, 19, 20, 21, 22, 23, 24][i]}
          </div>
        </div>
      ))}

      {/* hour axis */}
      <div style={{
        position: "relative",
        height: rowHeight, paddingRight: 6, paddingTop: 4,
      }}>
        {HOURS.map((h, i) => (
          <div key={h} style={{
            position: "absolute",
            top: `${(i / (HOURS.length - 1)) * 100}%`,
            right: 6, transform: "translateY(-50%)",
            fontSize: 10, color: "var(--ink-3)", fontFamily: "var(--font-mono)",
          }}>
            {h.toString().padStart(2, "0")}h
          </div>
        ))}
      </div>

      {/* day columns */}
      {Array.from({length: 7}).map((_, dayIdx) => (
        <div key={dayIdx} style={{
          position: "relative",
          height: rowHeight,
          background: "rgba(255,255,255,0.015)",
          borderLeft: "1px solid var(--border-soft)",
        }}>
          {/* hour gridlines */}
          {HOURS.map((_, hi) => (
            <span key={hi} style={{
              position: "absolute", left: 0, right: 0,
              top: `${(hi / (HOURS.length - 1)) * 100}%`,
              height: 1,
              background: "rgba(255,255,255,0.04)",
            }}/>
          ))}

          {/* runs */}
          {runs.filter(r => r.day === dayIdx).map(run => {
            const accent = ACCENT[run.zoneAccent];
            const top    = ((run.start - startHour) / totalHours) * 100;
            const height = ((run.end - run.start) / totalHours) * 100;
            return (
              <motion.div
                key={run.id}
                whileHover={{scale: 1.02, y: -1}}
                whileTap={{scale: 0.98}}
                transition={springSnap}
                style={{
                  position: "absolute",
                  top: `${top}%`, left: 4, right: 4,
                  height: `${height}%`,
                  minHeight: 30,
                  background: `linear-gradient(135deg, ${accent}30, ${accent}0a)`,
                  border: `1px solid ${accent}55`,
                  borderLeft: `3px solid ${accent}`,
                  borderRadius: 10,
                  padding: "6px 8px",
                  cursor: "pointer",
                  overflow: "hidden",
                }}
              >
                <div style={{
                  fontSize: 11, fontWeight: 700, color: "var(--ink)",
                  lineHeight: 1.2,
                }}>
                  {t(run.zone)}
                </div>
                <div className="mono" style={{
                  fontSize: 9, color: accent, marginTop: 3, fontWeight: 600,
                }}>
                  {String(Math.floor(run.start)).padStart(2, "0")}:{(run.start % 1) === 0 ? "00" : "30"} – {String(Math.floor(run.end)).padStart(2, "0")}:{(run.end % 1) === 0 ? "00" : "30"}
                </div>
              </motion.div>
            );
          })}
        </div>
      ))}
    </div>
  );
}

interface ZoneRowProps {
  name: string;
  accent: "lime" | "cyan" | "amber" | "rose";
  area: number;
  weekly: number;
  duration: string;
}

function ZoneRow({name, accent, area, weekly, duration}: ZoneRowProps) {
  const {t} = useTranslation();
  const accentColor = ACCENT[accent];
  return (
    <motion.div
      whileHover={{x: 2}}
      transition={springSnap}
      style={{
        display: "flex", alignItems: "center", gap: 12,
        padding: 12,
        background: "rgba(255,255,255,0.025)",
        border: "1px solid var(--border-soft)",
        borderRadius: 12,
        position: "relative", overflow: "hidden",
      }}
    >
      <span style={{
        position: "absolute", left: 0, top: 0, bottom: 0, width: 3,
        background: accentColor, opacity: 0.7,
      }}/>
      <div style={{
        width: 38, height: 38, borderRadius: 12,
        background: `linear-gradient(135deg, ${accentColor}33, ${accentColor}0a)`,
        border: `1px solid ${accentColor}44`,
        display: "flex", alignItems: "center", justifyContent: "center",
        color: accentColor,
        flexShrink: 0,
      }}>
        <Calendar size={16}/>
      </div>
      <div style={{flex: 1, minWidth: 0}}>
        <div style={{fontSize: 14, fontWeight: 600, color: "var(--ink)"}}>
          {name}
        </div>
        <div className="mono" style={{fontSize: 11, color: "var(--ink-3)", marginTop: 2}}>
          {area} m² · {t('scheduleZones.mowCount', {count: weekly})} · {duration}
        </div>
      </div>
      <div className="display" style={{
        fontSize: 22, fontWeight: 700, color: accentColor, letterSpacing: "-0.02em",
      }}>
        {weekly}
      </div>
    </motion.div>
  );
}

function RulePill({
  icon, label, active,
}: {icon: React.ReactNode; label: string; active: boolean}) {
  return (
    <button style={{
      display: "inline-flex", alignItems: "center", gap: 8,
      padding: "8px 12px",
      background: active ? "rgba(124,255,178,0.10)" : "rgba(255,255,255,0.03)",
      border: `1px solid ${active ? "var(--border-glow)" : "var(--border-soft)"}`,
      borderRadius: 999, fontSize: 12, fontWeight: 600,
      color: active ? "var(--lime)" : "var(--ink-3)",
    }}>
      {icon}
      {label}
    </button>
  );
}
