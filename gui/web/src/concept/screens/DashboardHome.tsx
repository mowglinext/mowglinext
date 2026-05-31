import {useState} from "react";
import {motion} from "framer-motion";
import {
  Sparkles, ChevronRight, Wifi, Droplets, ThermometerSun, Wind,
} from "lucide-react";

import {GlassCard} from "../components/GlassCard";
import {BatteryRing} from "../components/BatteryRing";
import {StatusOrb} from "../components/StatusOrb";
import {ModeSegment, type Mode} from "../components/ModeSegment";
import {ActionCluster} from "../components/ActionCluster";
import {WeatherChip} from "../components/WeatherChip";
import {LiveMapMini} from "../components/LiveMapMini";
import {ProgressRibbon} from "../components/ProgressRibbon";
import {NoiseTexture} from "../components/NoiseTexture";
import {riseFade, staggerParent, popIn, springSnap} from "../motion";
import {useViewport} from "../useViewport";

/**
 * Dashboard Home -- showpiece.
 *
 * Mobile (single column, max-width 540):
 *   header → hero → modes → live map → tiles → next-up
 *
 * Desktop (≥1024, two-column 1.2fr/1fr):
 *   left:  hero (taller, with the action cluster), modes, next-up
 *   right: live map (large), 3-up tiles, weather card
 */

export function DashboardHome() {
  const [mode, setMode] = useState<Mode>("eco");
  const [phase, setPhase] = useState<"idle" | "playing" | "returning" | "alert">("playing");
  const vp = useViewport();

  const battery = 78;
  const remainingMin = 47;
  const coverage = 0.42;
  const todayMowedM2 = 184;
  const totalArea = 430;
  const robot = {x: 0.58, y: 0.5, heading: 38};

  return (
    <div style={{
      position: "relative",
      minHeight: "100dvh",
      padding: vp.isAtLeastTablet
        ? "36px 36px 64px"
        : "max(20px, env(safe-area-inset-top, 0px)) 18px 110px",
      overflow: "hidden",
    }}>
      {/* Aurora backdrop */}
      <div aria-hidden style={{
        position: "absolute", inset: -100,
        background: "var(--grad-aurora)",
        pointerEvents: "none", filter: "blur(10px)", zIndex: 0,
      }}/>
      <NoiseTexture/>

      <motion.div
        variants={staggerParent(0.06, 0.08)}
        initial="hidden"
        animate="show"
        style={{
          position: "relative", zIndex: 1,
          maxWidth: vp.isWide ? 1280 : vp.isDesktop ? 1180 : 560,
          margin: "0 auto",
        }}
      >
        {/* Greeting strip */}
        <motion.header variants={riseFade} style={{
          display: "flex", alignItems: "center", justifyContent: "space-between",
          marginBottom: vp.isAtLeastTablet ? 26 : 18,
        }}>
          <div>
            <div style={{
              fontSize: 12, color: "var(--ink-3)",
              letterSpacing: "0.06em", textTransform: "uppercase",
              fontWeight: 600,
            }}>
              <Greeting/>
            </div>
            <div className="display" style={{
              fontSize: vp.isAtLeastTablet ? 36 : 26,
              color: "var(--ink)", fontWeight: 700,
              letterSpacing: "-0.025em", lineHeight: 1.1, marginTop: 2,
            }}>
              Cedric
            </div>
          </div>
          <StatusOrb
            tone={phase === "playing" ? "live" : "resting"}
            size={10}
            label={phase === "playing" ? "En tonte" : "Au repos"}
          />
        </motion.header>

        {/* Layout */}
        {vp.isAtLeastTablet ? (
          // ────────── Desktop ──────────
          <div style={{
            display: "grid",
            gridTemplateColumns: vp.isDesktop || vp.isWide ? "1.2fr 1fr" : "1fr",
            gap: 22,
            alignItems: "start",
          }}>
            <div style={{display: "flex", flexDirection: "column", gap: 22}}>
              <motion.div variants={popIn}>
                <HeroCard
                  battery={battery}
                  remainingMin={remainingMin}
                  coverage={coverage}
                  todayMowedM2={todayMowedM2}
                  totalArea={totalArea}
                  phase={phase}
                  setPhase={setPhase}
                  large
                />
              </motion.div>
              <motion.div variants={riseFade}>
                <ModeCard mode={mode} setMode={setMode}/>
              </motion.div>
              <motion.div variants={riseFade}>
                <UpNextCard/>
              </motion.div>
            </div>

            <div style={{display: "flex", flexDirection: "column", gap: 22}}>
              <motion.div variants={riseFade}>
                <LiveGardenCard coverage={coverage} robot={robot} height={300}/>
              </motion.div>
              <motion.div variants={riseFade} style={{
                display: "grid", gridTemplateColumns: "repeat(3, 1fr)", gap: 12,
              }}>
                <StatTile label="Couverture" value={`${Math.round(coverage * 100)}`}
                          unit="%" icon={<Droplets size={14}/>}
                          accent="lime" hint="42 % de la zone"/>
                <StatTile label="Tonte" value="47" unit="min"
                          icon={<Wind size={14}/>}
                          accent="cyan" hint="restantes aujourd'hui"/>
                <StatTile label="Météo" value="22" unit="°C"
                          icon={<ThermometerSun size={14}/>}
                          accent="amber" hint="ensoleillé · sec"/>
              </motion.div>
            </div>
          </div>
        ) : (
          // ────────── Mobile ──────────
          <>
            <motion.div variants={popIn} style={{marginBottom: 16}}>
              <HeroCard
                battery={battery}
                remainingMin={remainingMin}
                coverage={coverage}
                todayMowedM2={todayMowedM2}
                totalArea={totalArea}
                phase={phase}
                setPhase={setPhase}
              />
            </motion.div>
            <motion.div variants={riseFade} style={{marginBottom: 16}}>
              <ModeCard mode={mode} setMode={setMode}/>
            </motion.div>
            <motion.div variants={riseFade} style={{marginBottom: 16}}>
              <LiveGardenCard coverage={coverage} robot={robot}/>
            </motion.div>
            <motion.div variants={riseFade} style={{
              display: "grid", gridTemplateColumns: "repeat(3, 1fr)",
              gap: 10, marginBottom: 16,
            }}>
              <StatTile label="Couverture" value={`${Math.round(coverage * 100)}`}
                        unit="%" icon={<Droplets size={14}/>}
                        accent="lime" hint="42 % de la zone"/>
              <StatTile label="Tonte" value="47" unit="min"
                        icon={<Wind size={14}/>}
                        accent="cyan" hint="restantes aujourd'hui"/>
              <StatTile label="Météo" value="22" unit="°C"
                        icon={<ThermometerSun size={14}/>}
                        accent="amber" hint="ensoleillé · sec"/>
            </motion.div>
            <motion.div variants={riseFade}>
              <UpNextCard/>
            </motion.div>
          </>
        )}
      </motion.div>
    </div>
  );
}

// ─────────────────────────────────────────────────────────────────────
// Composed sub-cards (used on both layouts)
// ─────────────────────────────────────────────────────────────────────

interface HeroCardProps {
  battery: number;
  remainingMin: number;
  coverage: number;
  todayMowedM2: number;
  totalArea: number;
  phase: "idle" | "playing" | "returning" | "alert";
  setPhase: (p: HeroCardProps["phase"]) => void;
  large?: boolean;
}

function HeroCard({
  battery, remainingMin, coverage, todayMowedM2, totalArea, phase, setPhase, large,
}: HeroCardProps) {
  return (
    <GlassCard variant="glow" padding={0}>
      <div style={{
        position: "relative",
        padding: large ? "30px 28px 26px" : "24px 22px 22px",
        background:
          "radial-gradient(circle at 80% -20%, rgba(124,255,178,0.18) 0%, transparent 55%)," +
          "radial-gradient(circle at -20% 110%, rgba(69,214,232,0.16) 0%, transparent 50%)",
      }}>
        <div style={{
          display: "grid",
          gridTemplateColumns: "1fr auto",
          gap: large ? 24 : 18,
          alignItems: "center",
        }}>
          <div style={{minWidth: 0}}>
            <div style={{
              fontSize: 11, color: "var(--lime)", fontWeight: 700,
              letterSpacing: "0.12em", textTransform: "uppercase",
            }}>
              Mowgli · zone Jardin sud
            </div>
            <h1 className="display" style={{
              fontSize: large ? 42 : 32,
              lineHeight: 1.05, marginTop: 8,
              fontWeight: 700, letterSpacing: "-0.028em",
              color: "var(--ink)",
            }}>
              Encore <span style={{
                background: "var(--grad-primary)",
                WebkitBackgroundClip: "text", backgroundClip: "text",
                WebkitTextFillColor: "transparent", color: "transparent",
              }}>{remainingMin} min</span> avant la maison.
            </h1>
            <p style={{
              fontSize: large ? 15 : 14, color: "var(--ink-2)", marginTop: 10,
              lineHeight: 1.55, maxWidth: 460,
            }}>
              Il avance à 0.42 m/s · gazon humide détecté, vitesse réduite de 12 %.
            </p>
          </div>
          <BatteryRing
            percent={battery}
            size={large ? 168 : 132}
            thickness={large ? 12 : 10}
            charging={phase === "returning"}
          >
            <div className="mono" style={{
              fontSize: large ? 42 : 32, fontWeight: 700, lineHeight: 1,
              color: "var(--ink)", letterSpacing: "-0.02em",
            }}>
              {battery}
            </div>
            <div style={{
              fontSize: 10, color: "var(--ink-3)",
              letterSpacing: "0.1em", textTransform: "uppercase",
              fontWeight: 600, marginTop: 2,
            }}>
              batterie
            </div>
          </BatteryRing>
        </div>

        <div style={{marginTop: large ? 26 : 20}}>
          <div style={{
            display: "flex", alignItems: "baseline", justifyContent: "space-between",
            marginBottom: 8,
          }}>
            <span style={{
              fontSize: 11, color: "var(--ink-3)",
              letterSpacing: "0.08em", textTransform: "uppercase",
              fontWeight: 600,
            }}>
              Progression du jour
            </span>
            <span className="mono" style={{
              fontSize: 13, color: "var(--ink-2)", fontWeight: 600,
            }}>
              {todayMowedM2}<span style={{color: "var(--ink-3)"}}>/{totalArea} m²</span>
            </span>
          </div>
          <ProgressRibbon value={coverage}/>
        </div>

        <div style={{marginTop: large ? 30 : 24}}>
          <ActionCluster
            phase={phase}
            onStart={() => setPhase("playing")}
            onPause={() => setPhase("idle")}
            onHome={() => setPhase("returning")}
            onStop={() => setPhase("alert")}
          />
        </div>
      </div>
    </GlassCard>
  );
}

interface ModeCardProps {
  mode: Mode;
  setMode: (m: Mode) => void;
}

function ModeCard({mode, setMode}: ModeCardProps) {
  return (
    <GlassCard padding={20}>
      <div style={{
        display: "flex", alignItems: "center", gap: 10, marginBottom: 14,
      }}>
        <Sparkles size={14} color="var(--lime)" strokeWidth={2.4}/>
        <span style={{
          fontSize: 11, color: "var(--ink-2)",
          letterSpacing: "0.08em", textTransform: "uppercase",
          fontWeight: 600,
        }}>
          Mode de coupe
        </span>
      </div>
      <ModeSegment value={mode} onChange={setMode}/>
    </GlassCard>
  );
}

interface LiveGardenCardProps {
  coverage: number;
  robot: {x: number; y: number; heading: number};
  height?: number;
}

function LiveGardenCard({coverage, robot, height = 200}: LiveGardenCardProps) {
  return (
    <GlassCard padding={0} style={{overflow: "hidden"}}>
      <div style={{
        display: "flex", alignItems: "baseline", justifyContent: "space-between",
        padding: "16px 22px 8px",
      }}>
        <div>
          <div style={{
            fontSize: 11, color: "var(--ink-3)",
            letterSpacing: "0.08em", textTransform: "uppercase",
            fontWeight: 600,
          }}>
            Carte vivante
          </div>
          <div className="display" style={{
            fontSize: 18, fontWeight: 700, marginTop: 2,
            color: "var(--ink)", letterSpacing: "-0.01em",
          }}>
            Jardin sud · trajectoire en direct
          </div>
        </div>
        <button style={{
          display: "inline-flex", alignItems: "center", gap: 4,
          fontSize: 12, fontWeight: 600, color: "var(--lime)",
        }}>
          Voir la carte
          <ChevronRight size={14} strokeWidth={2.4}/>
        </button>
      </div>
      <LiveMapMini coverage={coverage} robot={robot} height={height}/>
    </GlassCard>
  );
}

function UpNextCard() {
  return (
    <GlassCard padding={20}>
      <div style={{
        display: "flex", alignItems: "center", justifyContent: "space-between",
        marginBottom: 14,
      }}>
        <div style={{
          fontSize: 11, color: "var(--ink-3)",
          letterSpacing: "0.08em", textTransform: "uppercase",
          fontWeight: 600,
        }}>
          Programmé ensuite
        </div>
        <WeatherChip condition="partly" tempC={22}/>
      </div>
      <NextRunRow when="Demain · 07:30" area="Jardin nord" duration="≈ 55 min" first/>
      <NextRunRow when="Vendredi · 18:00" area="Pelouse devant" duration="≈ 35 min"/>
    </GlassCard>
  );
}

// ─────────────────────────────────────────────────────────────────────
// Local atoms
// ─────────────────────────────────────────────────────────────────────

function Greeting() {
  const h = new Date().getHours();
  const text =
    h < 6  ? "Bonsoir" :
    h < 12 ? "Bonjour" :
    h < 18 ? "Bel après-midi" :
    "Bonsoir";
  return <>{text}</>;
}

interface StatTileProps {
  label: string;
  value: string;
  unit: string;
  icon: React.ReactNode;
  accent: "lime" | "cyan" | "amber";
  hint: string;
}

function StatTile({label, value, unit, icon, accent, hint}: StatTileProps) {
  const accentColor =
    accent === "lime"  ? "var(--lime)" :
    accent === "cyan"  ? "var(--aurora-cyan)" :
    "var(--amber)";
  return (
    <motion.div
      whileHover={{y: -2}}
      whileTap={{scale: 0.97}}
      transition={springSnap}
    >
      <GlassCard padding={14} style={{position: "relative", overflow: "hidden"}}>
        <span aria-hidden style={{
          position: "absolute", top: -18, right: -18,
          width: 72, height: 72, borderRadius: 72,
          background: `radial-gradient(circle, ${accentColor}, transparent 70%)`,
          opacity: 0.25, pointerEvents: "none",
        }}/>
        <div style={{
          display: "flex", alignItems: "center", gap: 8,
          fontSize: 10, fontWeight: 600,
          color: "var(--ink-2)", letterSpacing: "0.08em",
          textTransform: "uppercase",
        }}>
          <span style={{color: accentColor}}>{icon}</span>
          {label}
        </div>
        <div style={{
          display: "flex", alignItems: "baseline", gap: 4, marginTop: 8,
        }}>
          <div className="display" style={{
            fontSize: 30, fontWeight: 700, lineHeight: 1,
            color: "var(--ink)", letterSpacing: "-0.025em",
          }}>{value}</div>
          <div className="mono" style={{
            fontSize: 12, color: "var(--ink-3)", fontWeight: 600,
          }}>{unit}</div>
        </div>
        <div style={{
          fontSize: 10, color: "var(--ink-3)", marginTop: 6, lineHeight: 1.3,
        }}>
          {hint}
        </div>
      </GlassCard>
    </motion.div>
  );
}

interface NextRunRowProps {
  when: string;
  area: string;
  duration: string;
  first?: boolean;
}

function NextRunRow({when, area, duration, first}: NextRunRowProps) {
  return (
    <div style={{
      display: "flex", alignItems: "center", gap: 12,
      padding: "10px 0",
      borderTop: first ? "none" : "1px solid var(--border-soft)",
    }}>
      <div style={{
        width: 36, height: 36,
        borderRadius: "var(--radius-sm)",
        background: first
          ? "linear-gradient(135deg, rgba(124,255,178,0.18), rgba(43,170,102,0.06))"
          : "rgba(255,255,255,0.04)",
        border: `1px solid ${first ? "var(--border-glow)" : "var(--border-soft)"}`,
        display: "flex", alignItems: "center", justifyContent: "center",
        flexShrink: 0,
      }}>
        <Wifi size={16} color={first ? "var(--lime)" : "var(--ink-2)"}/>
      </div>
      <div style={{flex: 1, minWidth: 0}}>
        <div style={{fontSize: 14, fontWeight: 600, color: "var(--ink)"}}>
          {area}
        </div>
        <div className="mono" style={{
          fontSize: 12, color: "var(--ink-3)", marginTop: 2,
        }}>
          {when} · {duration}
        </div>
      </div>
      <ChevronRight size={16} color="var(--ink-3)"/>
    </div>
  );
}
