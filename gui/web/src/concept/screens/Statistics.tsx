import {motion} from "framer-motion";
import {useTranslation} from "react-i18next";
import {TrendingUp, Flame, Sparkles, Award, Activity} from "lucide-react";

import {GlassCard} from "../components/GlassCard";
import {NoiseTexture} from "../components/NoiseTexture";
import {staggerParent, riseFade, springSnap} from "../motion";
import {useViewport} from "../useViewport";

/**
 * Statistics. Hero numbers + year-of-lawn heatmap + weekly distance bars
 * + sessions log. Responsive: stacks on mobile, 2-col on desktop with
 * the heatmap spanning full width on top.
 */

export function Statistics() {
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
        variants={staggerParent(0.05, 0.08)}
        initial="hidden" animate="show"
        style={{
          position: "relative", zIndex: 1,
          maxWidth: vp.isAtLeastTablet ? 1280 : 560, margin: "0 auto",
        }}
      >
        {/* header */}
        <motion.header variants={riseFade} style={{marginBottom: vp.isAtLeastTablet ? 26 : 18}}>
          <div style={{
            fontSize: 11, color: "var(--ink-3)",
            letterSpacing: "0.08em", textTransform: "uppercase", fontWeight: 700,
          }}>
            {t('conceptStatistics.title')}
          </div>
          <h1 className="display" style={{
            fontSize: vp.isAtLeastTablet ? 36 : 28, marginTop: 4,
            fontWeight: 700, letterSpacing: "-0.025em", lineHeight: 1.1,
          }}>
            <em style={{fontStyle: "italic", color: "var(--lime)"}}>{t('conceptStatistics.headlineWeeks')}</em> {t('conceptStatistics.headlineRest')}
          </h1>
        </motion.header>

        {/* Hero stats */}
        <motion.div variants={riseFade} style={{
          display: "grid",
          gridTemplateColumns: vp.isAtLeastTablet ? "repeat(4, 1fr)" : "repeat(2, 1fr)",
          gap: 12,
          marginBottom: 22,
        }}>
          <HeroStat label={t('conceptStatistics.heroAreaLabel')} value="184.3" unit="km²" accent="lime"
                    delta={t('conceptStatistics.heroAreaDelta')} icon={<TrendingUp size={14}/>}/>
          <HeroStat label={t('conceptStatistics.heroSessionsLabel')} value="263" unit="" accent="cyan"
                    delta={t('conceptStatistics.heroSessionsDelta')}     icon={<Activity size={14}/>}/>
          <HeroStat label={t('conceptStatistics.heroStreakLabel')} value="12" unit={t('conceptStatistics.heroStreakUnit')} accent="amber"
                    delta={t('conceptStatistics.heroStreakDelta')}   icon={<Flame size={14}/>}/>
          <HeroStat label={t('conceptStatistics.heroSavingsLabel')} value="46" unit="h" accent="rose"
                    delta={t('conceptStatistics.heroSavingsDelta')}     icon={<Award size={14}/>}/>
        </motion.div>

        {/* Year of lawn heatmap */}
        <motion.div variants={riseFade} style={{marginBottom: 22}}>
          <GlassCard padding={20}>
            <div style={{
              display: "flex", alignItems: "baseline", justifyContent: "space-between",
              marginBottom: 16,
            }}>
              <div>
                <div style={{
                  fontSize: 11, color: "var(--ink-3)",
                  letterSpacing: "0.08em", textTransform: "uppercase", fontWeight: 700,
                }}>
                  Year of Lawn · 2025-2026
                </div>
                <div className="display" style={{
                  fontSize: vp.isAtLeastTablet ? 24 : 18, fontWeight: 700, marginTop: 4,
                  letterSpacing: "-0.02em",
                }}>
                  {t('conceptStatistics.heatmapSubtitle')}
                </div>
              </div>
              <Legend/>
            </div>
            <YearHeatmap/>
          </GlassCard>
        </motion.div>

        {/* Weekly bars + Sessions */}
        <div style={{
          display: "grid",
          gridTemplateColumns: vp.isAtLeastTablet ? "1.4fr 1fr" : "1fr",
          gap: 22,
        }}>
          <motion.div variants={riseFade}>
            <GlassCard padding={20}>
              <div style={{
                display: "flex", alignItems: "center", gap: 10, marginBottom: 16,
              }}>
                <Sparkles size={14} color="var(--lime)" strokeWidth={2.4}/>
                <span style={{
                  fontSize: 11, color: "var(--ink-2)",
                  letterSpacing: "0.08em", textTransform: "uppercase", fontWeight: 700,
                }}>
                  {t('conceptStatistics.distancePerWeek')}
                </span>
              </div>
              <WeeklyBars/>
            </GlassCard>
          </motion.div>

          <motion.div variants={riseFade}>
            <GlassCard padding={20}>
              <div style={{
                fontSize: 11, color: "var(--ink-3)",
                letterSpacing: "0.08em", textTransform: "uppercase",
                fontWeight: 700, marginBottom: 14,
              }}>
                {t('conceptStatistics.recentSessions')}
              </div>
              <div style={{display: "flex", flexDirection: "column", gap: 6}}>
                <SessionRow date={t('conceptStatistics.session1Date')} zone={t('conceptStatistics.zoneSouthGarden')}     duration="1h 47min" status="ongoing"/>
                <SessionRow date={t('conceptStatistics.session2Date')}        zone={t('conceptStatistics.zoneFrontLawn')} duration="35min" status="ok"/>
                <SessionRow date={t('conceptStatistics.session3Date')}        zone={t('conceptStatistics.zoneNorthGarden')}    duration="52min" status="ok"/>
                <SessionRow date={t('conceptStatistics.session4Date')}      zone={t('conceptStatistics.zoneSidePath')} duration="14min" status="ok"/>
                <SessionRow date={t('conceptStatistics.session5Date')}      zone={t('conceptStatistics.zoneSouthGarden')}     duration="2h 04min" status="ok"/>
                <SessionRow date={t('conceptStatistics.session6Date')}      zone={t('conceptStatistics.zoneNorthGarden')}    duration="48min" status="rain"/>
              </div>
            </GlassCard>
          </motion.div>
        </div>
      </motion.div>
    </div>
  );
}

// ─────────────────────────────────────────────────────────────────────
// Hero stat
// ─────────────────────────────────────────────────────────────────────

function HeroStat({
  label, value, unit, accent, delta, icon,
}: {
  label: string;
  value: string;
  unit: string;
  accent: "lime" | "cyan" | "amber" | "rose";
  delta: string;
  icon: React.ReactNode;
}) {
  const colors = {
    lime:  "var(--lime)",
    cyan:  "var(--aurora-cyan)",
    amber: "var(--amber)",
    rose:  "var(--rose)",
  }[accent];

  return (
    <motion.div whileHover={{y: -2}} transition={springSnap}>
      <GlassCard padding={18} style={{position: "relative", overflow: "hidden"}}>
        <span aria-hidden style={{
          position: "absolute", top: -28, right: -28,
          width: 110, height: 110, borderRadius: 110,
          background: `radial-gradient(circle, ${colors}, transparent 70%)`,
          opacity: 0.22, pointerEvents: "none",
        }}/>
        <div style={{
          display: "flex", alignItems: "center", gap: 8,
          fontSize: 10, fontWeight: 700,
          color: "var(--ink-2)", letterSpacing: "0.08em", textTransform: "uppercase",
        }}>
          <span style={{color: colors}}>{icon}</span>
          {label}
        </div>
        <div style={{
          display: "flex", alignItems: "baseline", gap: 4, marginTop: 10,
        }}>
          <div className="display" style={{
            fontSize: 44, fontWeight: 700, color: colors,
            lineHeight: 1, letterSpacing: "-0.03em",
          }}>{value}</div>
          {unit && (
            <div className="mono" style={{fontSize: 14, color: "var(--ink-3)", fontWeight: 600}}>
              {unit}
            </div>
          )}
        </div>
        <div style={{fontSize: 11, color: "var(--ink-3)", marginTop: 8, fontWeight: 500}}>
          {delta}
        </div>
      </GlassCard>
    </motion.div>
  );
}

// ─────────────────────────────────────────────────────────────────────
// Year heatmap
// ─────────────────────────────────────────────────────────────────────

const HEATMAP_WEEKS = 52;
const HEATMAP_DAYS = 7;
const MONTH_KEYS = [
  "conceptStatistics.monthJun", "conceptStatistics.monthJul", "conceptStatistics.monthAug",
  "conceptStatistics.monthSep", "conceptStatistics.monthOct", "conceptStatistics.monthNov",
  "conceptStatistics.monthDec", "conceptStatistics.monthJan", "conceptStatistics.monthFeb",
  "conceptStatistics.monthMar", "conceptStatistics.monthApr", "conceptStatistics.monthMay",
];
const DAY_KEYS = ["conceptStatistics.dayMon", "conceptStatistics.dayWed", "conceptStatistics.dayFri"];

function pseudoIntensity(week: number, day: number): number {
  // deterministic noise that varies by week + day, biases mowing days
  const seed = (week * 53 + day * 11) % 100;
  if (day === 0 || day === 6) return seed > 60 ? 3 : seed > 40 ? 2 : seed > 30 ? 1 : 0;
  return seed > 80 ? 4 : seed > 50 ? 3 : seed > 25 ? 2 : seed > 10 ? 1 : 0;
}

function YearHeatmap() {
  const {t} = useTranslation();
  const cellSize = 12;
  const gap = 3;
  const colors = [
    "rgba(255,255,255,0.04)",
    "rgba(124,255,178,0.20)",
    "rgba(124,255,178,0.4)",
    "rgba(124,255,178,0.65)",
    "var(--lime)",
  ];
  const totalWidth = HEATMAP_WEEKS * (cellSize + gap);
  const totalHeight = HEATMAP_DAYS * (cellSize + gap) + 22;

  return (
    <div style={{overflowX: "auto"}} className="scrollbar-thin">
      <svg width={totalWidth + 30} height={totalHeight} style={{display: "block"}}>
        {/* month labels */}
        {MONTH_KEYS.map((m, i) => (
          <text key={m}
            x={Math.round((i / MONTH_KEYS.length) * totalWidth) + 30}
            y={10}
            fontSize={9}
            fill="rgba(236,255,244,0.45)"
            fontFamily="var(--font-body)">
            {t(m)}
          </text>
        ))}
        {/* day labels */}
        {DAY_KEYS.map((d, i) => (
          <text key={d}
            x={0}
            y={22 + (i * 2 + 1) * (cellSize + gap)}
            fontSize={9}
            fill="rgba(236,255,244,0.45)"
            fontFamily="var(--font-body)">
            {t(d)}
          </text>
        ))}
        {/* cells */}
        {Array.from({length: HEATMAP_WEEKS}).map((_, wi) =>
          Array.from({length: HEATMAP_DAYS}).map((__, di) => {
            const intensity = pseudoIntensity(wi, di);
            return (
              <motion.rect
                key={`${wi}-${di}`}
                x={30 + wi * (cellSize + gap)}
                y={20 + di * (cellSize + gap)}
                width={cellSize} height={cellSize} rx={2}
                fill={colors[intensity]}
                initial={{opacity: 0, scale: 0.4}}
                animate={{opacity: 1, scale: 1}}
                transition={{
                  delay: 0.5 + wi * 0.005 + di * 0.01,
                  duration: 0.4, ease: [0.2, 0.7, 0.2, 1],
                }}
              >
                <title>S{wi + 1} – {intensity * 0.4} km</title>
              </motion.rect>
            );
          })
        )}
      </svg>
    </div>
  );
}

function Legend() {
  const {t} = useTranslation();
  const colors = [
    "rgba(255,255,255,0.04)",
    "rgba(124,255,178,0.20)",
    "rgba(124,255,178,0.4)",
    "rgba(124,255,178,0.65)",
    "var(--lime)",
  ];
  return (
    <div style={{
      display: "inline-flex", alignItems: "center", gap: 6,
      fontSize: 10, color: "var(--ink-3)",
    }}>
      <span>{t('conceptStatistics.legendLess')}</span>
      {colors.map((c, i) => (
        <span key={i} style={{
          width: 10, height: 10, borderRadius: 2, background: c,
        }}/>
      ))}
      <span>{t('conceptStatistics.legendMore')}</span>
    </div>
  );
}

// ─────────────────────────────────────────────────────────────────────
// Weekly bars
// ─────────────────────────────────────────────────────────────────────

const WEEKLY_DATA = [3.4, 2.8, 5.1, 4.6, 6.2, 5.8, 3.9, 4.4, 7.1, 6.4, 5.3, 7.8];
const WEEKLY_MAX = Math.max(...WEEKLY_DATA);

function WeeklyBars() {
  return (
    <div style={{
      display: "flex", alignItems: "flex-end", gap: 8, height: 200, paddingBottom: 24,
      position: "relative",
    }}>
      {/* gridlines */}
      {[0.25, 0.5, 0.75].map((p, i) => (
        <span key={i} style={{
          position: "absolute", left: 0, right: 0,
          bottom: 24 + p * 176,
          height: 1, background: "rgba(255,255,255,0.04)",
        }}/>
      ))}
      {WEEKLY_DATA.map((v, i) => {
        const h = (v / WEEKLY_MAX) * 176;
        const isLatest = i === WEEKLY_DATA.length - 1;
        return (
          <div key={i} style={{
            flex: 1, display: "flex", flexDirection: "column", alignItems: "center", gap: 6,
            position: "relative",
          }}>
            <motion.div
              initial={{height: 0}}
              animate={{height: h}}
              transition={{
                delay: 0.6 + i * 0.04, duration: 0.6, ease: [0.2, 0.7, 0.2, 1],
              }}
              style={{
                width: "100%", borderRadius: "6px 6px 2px 2px",
                background: isLatest
                  ? "linear-gradient(180deg, var(--lime), var(--emerald))"
                  : "linear-gradient(180deg, rgba(124,255,178,0.45), rgba(43,170,102,0.15))",
                border: isLatest ? "1px solid var(--border-glow)" : "none",
                boxShadow: isLatest ? "0 0 16px rgba(124,255,178,0.3)" : "none",
              }}
            />
            <div style={{
              fontSize: 9, color: "var(--ink-3)", position: "absolute", bottom: 4,
              fontFamily: "var(--font-mono)",
            }}>
              S{i + 1}
            </div>
            {isLatest && v > 0 && (
              <motion.div
                initial={{opacity: 0, y: 4}}
                animate={{opacity: 1, y: 0}}
                transition={{delay: 1.0}}
                style={{
                  position: "absolute", top: -22,
                  fontSize: 11, fontWeight: 700, color: "var(--lime)",
                  fontFamily: "var(--font-display)",
                }}
              >
                {v.toFixed(1)} km
              </motion.div>
            )}
          </div>
        );
      })}
    </div>
  );
}

// ─────────────────────────────────────────────────────────────────────
// Session row
// ─────────────────────────────────────────────────────────────────────

function SessionRow({
  date, zone, duration, status,
}: {
  date: string;
  zone: string;
  duration: string;
  status: "ongoing" | "ok" | "rain";
}) {
  const dotColor =
    status === "ongoing" ? "var(--lime)" :
    status === "rain"    ? "var(--amber)" :
    "var(--aurora-cyan)";

  return (
    <div style={{
      display: "flex", alignItems: "center", gap: 10,
      padding: "10px 0",
      borderBottom: "1px solid var(--border-soft)",
    }}>
      <span style={{
        width: 8, height: 8, borderRadius: 4,
        background: dotColor,
        boxShadow: status === "ongoing" ? `0 0 10px ${dotColor}` : "none",
        flexShrink: 0,
      }}/>
      <div style={{flex: 1, minWidth: 0}}>
        <div style={{fontSize: 13, fontWeight: 600, color: "var(--ink)"}}>{zone}</div>
        <div className="mono" style={{fontSize: 11, color: "var(--ink-3)", marginTop: 2}}>
          {date}
        </div>
      </div>
      <div className="mono" style={{
        fontSize: 12, color: dotColor, fontWeight: 700, fontFamily: "var(--font-display)",
      }}>
        {duration}
      </div>
    </div>
  );
}
