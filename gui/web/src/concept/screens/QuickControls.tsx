import {useState} from "react";
import {motion} from "framer-motion";
import {
  ArrowUp, ArrowDown, ArrowLeft, ArrowRight,
  Scissors, ScissorsLineDashed, Volume2, Lightbulb, Lock, Crosshair,
  Pause, Square, Play,
} from "lucide-react";

import {GlassCard} from "../components/GlassCard";
import {ModeSegment, type Mode} from "../components/ModeSegment";
import {StatusOrb} from "../components/StatusOrb";
import {NoiseTexture} from "../components/NoiseTexture";
import {staggerParent, riseFade, springSnap, pressFeedback} from "../motion";
import {useViewport} from "../useViewport";

/**
 * Quick Controls screen.
 *
 * Mobile: stacked — header → big Start hero → mode segment → joystick
 * pad → blade + light + lock + locate quick actions.
 *
 * Desktop: 2-col — left has the hero CTA + joystick, right has mode
 * + telemetry + quick actions in a tidy 2×2.
 */

export function QuickControls() {
  const vp = useViewport();
  const [mode, setMode] = useState<Mode>("eco");
  const [running, setRunning] = useState(true);
  const [blade, setBlade] = useState(true);

  return (
    <div style={{
      position: "relative",
      minHeight: "100dvh",
      padding: vp.isAtLeastTablet ? "36px 36px 56px" : "max(20px, env(safe-area-inset-top)) 18px 110px",
      overflow: "hidden",
    }}>
      <div aria-hidden style={{
        position: "absolute", inset: -100, pointerEvents: "none",
        background: "var(--grad-aurora)",
        filter: "blur(8px)", zIndex: 0,
      }}/>
      <NoiseTexture/>

      <motion.div
        variants={staggerParent(0.06, 0.08)}
        initial="hidden" animate="show"
        style={{
          position: "relative", zIndex: 1,
          maxWidth: vp.isAtLeastTablet ? 1180 : 560,
          margin: "0 auto",
        }}
      >
        {/* header */}
        <motion.header variants={riseFade} style={{
          display: "flex", alignItems: "baseline", justifyContent: "space-between",
          marginBottom: vp.isAtLeastTablet ? 26 : 18,
        }}>
          <div>
            <div style={{
              fontSize: 11, color: "var(--ink-3)",
              letterSpacing: "0.08em", textTransform: "uppercase", fontWeight: 700,
            }}>
              Contrôles
            </div>
            <h1 className="display" style={{
              fontSize: vp.isAtLeastTablet ? 38 : 28, marginTop: 4,
              fontWeight: 700, letterSpacing: "-0.025em", lineHeight: 1.1,
            }}>
              Mowgli, prêt à <em style={{fontStyle: "italic", color: "var(--lime)"}}>partir</em>.
            </h1>
          </div>
          <StatusOrb tone={running ? "live" : "resting"} size={10} label={running ? "Active" : "En pause"}/>
        </motion.header>

        <div style={{
          display: "grid",
          gridTemplateColumns: vp.isAtLeastTablet ? "1fr 1fr" : "1fr",
          gap: 22,
        }}>
          {/* ── Left column ── */}
          <div style={{display: "flex", flexDirection: "column", gap: 18}}>
            <motion.div variants={riseFade}>
              <PrimaryActionHero
                running={running}
                onToggle={() => setRunning(r => !r)}
              />
            </motion.div>
            <motion.div variants={riseFade}>
              <JoystickPadCard/>
            </motion.div>
          </div>

          {/* ── Right column ── */}
          <div style={{display: "flex", flexDirection: "column", gap: 18}}>
            <motion.div variants={riseFade}>
              <GlassCard padding={20}>
                <div style={{
                  fontSize: 11, color: "var(--ink-2)",
                  letterSpacing: "0.08em", textTransform: "uppercase",
                  fontWeight: 700, marginBottom: 14,
                }}>
                  Mode de coupe
                </div>
                <ModeSegment value={mode} onChange={setMode}/>
              </GlassCard>
            </motion.div>

            <motion.div variants={riseFade}>
              <BladeControlCard blade={blade} onToggle={() => setBlade(b => !b)}/>
            </motion.div>

            <motion.div variants={riseFade} style={{
              display: "grid", gridTemplateColumns: "1fr 1fr", gap: 10,
            }}>
              <QuickToggle icon={<Volume2 size={18}/>}     label="Sourdine"  defaultActive={false}/>
              <QuickToggle icon={<Lightbulb size={18}/>}   label="Phares"   defaultActive={true}/>
              <QuickToggle icon={<Lock size={18}/>}        label="Verrou"   defaultActive={false}/>
              <QuickToggle icon={<Crosshair size={18}/>}   label="Locate"   pulse/>
            </motion.div>
          </div>
        </div>
      </motion.div>
    </div>
  );
}

// ─────────────────────────────────────────────────────────────────────
// Sub-components
// ─────────────────────────────────────────────────────────────────────

function PrimaryActionHero({running, onToggle}: {running: boolean; onToggle: () => void}) {
  return (
    <GlassCard variant="glow" padding={0}>
      <div style={{
        position: "relative",
        padding: "28px 24px",
        background:
          "radial-gradient(circle at 50% -20%, rgba(124,255,178,0.18) 0%, transparent 60%)",
      }}>
        <div style={{
          display: "flex", alignItems: "center", justifyContent: "space-between",
          marginBottom: 22,
        }}>
          <div>
            <div style={{
              fontSize: 10, color: "var(--lime)", fontWeight: 700,
              letterSpacing: "0.12em", textTransform: "uppercase",
            }}>
              Action principale
            </div>
            <div className="display" style={{
              fontSize: 24, fontWeight: 700, marginTop: 4,
              letterSpacing: "-0.02em",
            }}>
              {running ? "Tonte en cours" : "Tondeuse en pause"}
            </div>
            <div style={{
              fontSize: 12, color: "var(--ink-2)", marginTop: 6,
            }}>
              {running ? "Tape pour mettre en pause" : "Tape pour reprendre"}
            </div>
          </div>
        </div>

        {/* Big action button */}
        <div style={{display: "flex", alignItems: "center", justifyContent: "center", gap: 14}}>
          <SecondaryActionBig icon={<Square size={20} fill="currentColor"/>} ariaLabel="Stop" tone="danger"/>
          <motion.button
            {...pressFeedback}
            onClick={onToggle}
            aria-label={running ? "Pause" : "Reprendre"}
            style={{
              position: "relative",
              width: 132, height: 132, borderRadius: "50%",
              background: "var(--grad-primary)",
              color: "var(--bg-deep)",
              display: "flex", alignItems: "center", justifyContent: "center",
              boxShadow:
                "0 24px 60px -12px rgba(124,255,178,0.5), inset 0 1px 0 rgba(255,255,255,0.42), inset 0 -14px 26px rgba(43,170,102,0.45)",
              overflow: "hidden",
            }}
          >
            <span aria-hidden style={{
              position: "absolute", inset: 0,
              borderRadius: "inherit",
              background: "linear-gradient(115deg, transparent 25%, rgba(255,255,255,0.45) 50%, transparent 75%)",
              mixBlendMode: "overlay",
              opacity: 0.55,
              transform: "translateX(-100%)",
              animation: "concept-shine 3.6s var(--ease-out) infinite",
              pointerEvents: "none",
            }}/>
            <motion.div
              key={running ? "p" : "r"}
              initial={{scale: 0.65, opacity: 0}}
              animate={{scale: 1, opacity: 1}}
              transition={springSnap}
            >
              {running
                ? <Pause size={48} strokeWidth={2.4} fill="currentColor"/>
                : <Play  size={48} strokeWidth={2.4} fill="currentColor" style={{marginLeft: 4}}/>}
            </motion.div>
          </motion.button>
          <SecondaryActionBig icon={<ArrowDown size={20}/>} ariaLabel="Retour à la base" tone="cyan"/>
        </div>

        <div style={{
          display: "flex", justifyContent: "center", gap: 28, marginTop: 22,
          fontSize: 11, color: "var(--ink-3)",
          letterSpacing: "0.06em", textTransform: "uppercase", fontWeight: 600,
        }}>
          <span>Stop d'urgence</span>
          <span style={{color: "var(--lime)"}}>{running ? "Pause" : "Lancer"}</span>
          <span>Rentrer</span>
        </div>
      </div>
    </GlassCard>
  );
}

function SecondaryActionBig({
  icon, ariaLabel, tone,
}: {icon: React.ReactNode; ariaLabel: string; tone: "default" | "cyan" | "danger"}) {
  const styles = {
    default: {bg: "var(--bg-elevated)",      border: "var(--border-soft)",        color: "var(--ink)"},
    cyan:    {bg: "rgba(69,214,232,0.10)",   border: "rgba(69,214,232,0.45)",     color: "var(--aurora-cyan)"},
    danger:  {bg: "rgba(255,107,122,0.10)",  border: "rgba(255,107,122,0.45)",    color: "var(--rose)"},
  }[tone];
  return (
    <motion.button
      {...pressFeedback}
      aria-label={ariaLabel}
      style={{
        width: 64, height: 64, borderRadius: "50%",
        background: styles.bg,
        border: `1px solid ${styles.border}`,
        color: styles.color,
        display: "flex", alignItems: "center", justifyContent: "center",
        backdropFilter: "blur(20px)",
      }}
    >
      {icon}
    </motion.button>
  );
}

function JoystickPadCard() {
  return (
    <GlassCard padding={22}>
      <div style={{
        display: "flex", alignItems: "center", justifyContent: "space-between",
        marginBottom: 16,
      }}>
        <div style={{
          fontSize: 11, color: "var(--ink-2)",
          letterSpacing: "0.08em", textTransform: "uppercase",
          fontWeight: 700,
        }}>
          Pilotage manuel
        </div>
        <span style={{
          fontSize: 11, color: "var(--ink-3)",
        }}>
          Maintenir pour déplacer
        </span>
      </div>

      <div style={{
        position: "relative",
        width: 220, height: 220,
        margin: "0 auto",
        borderRadius: "50%",
        background:
          "radial-gradient(circle at 50% 50%, rgba(124,255,178,0.12) 0%, rgba(255,255,255,0.02) 60%)",
        border: "1px solid var(--border-soft)",
      }}>
        {/* center dot (would be the joystick handle) */}
        <span style={{
          position: "absolute", top: "50%", left: "50%",
          transform: "translate(-50%,-50%)",
          width: 60, height: 60, borderRadius: "50%",
          background: "var(--grad-primary)",
          boxShadow: "0 8px 24px -6px rgba(124,255,178,0.5), inset 0 1px 0 rgba(255,255,255,0.35)",
        }}/>
        {/* directional arrows */}
        <DirectionalArrow dir="up"/>
        <DirectionalArrow dir="down"/>
        <DirectionalArrow dir="left"/>
        <DirectionalArrow dir="right"/>
      </div>
    </GlassCard>
  );
}

function DirectionalArrow({dir}: {dir: "up" | "down" | "left" | "right"}) {
  const map = {
    up:    {top: 14,   left: "50%",  transform: "translateX(-50%)",        icon: <ArrowUp size={18}/>},
    down:  {bottom: 14, left: "50%", transform: "translateX(-50%)",        icon: <ArrowDown size={18}/>},
    left:  {left: 14,  top: "50%",   transform: "translateY(-50%)",        icon: <ArrowLeft size={18}/>},
    right: {right: 14, top: "50%",   transform: "translateY(-50%)",        icon: <ArrowRight size={18}/>},
  }[dir];
  return (
    <motion.span
      whileTap={{scale: 0.85}}
      transition={springSnap}
      style={{
        position: "absolute", ...map,
        width: 36, height: 36, borderRadius: 10,
        background: "rgba(255,255,255,0.04)",
        border: "1px solid var(--border-soft)",
        display: "flex", alignItems: "center", justifyContent: "center",
        color: "var(--ink-2)",
        cursor: "pointer",
      }}
    >
      {map.icon}
    </motion.span>
  );
}

function BladeControlCard({blade, onToggle}: {blade: boolean; onToggle: () => void}) {
  return (
    <GlassCard padding={20}>
      <div style={{
        display: "flex", alignItems: "center", justifyContent: "space-between",
      }}>
        <div style={{display: "flex", alignItems: "center", gap: 12}}>
          <div style={{
            width: 46, height: 46, borderRadius: 14,
            background: blade
              ? "linear-gradient(135deg, var(--lime), var(--mint))"
              : "rgba(255,255,255,0.04)",
            border: `1px solid ${blade ? "rgba(124,255,178,0.4)" : "var(--border-soft)"}`,
            color: blade ? "var(--bg-deep)" : "var(--ink-2)",
            display: "flex", alignItems: "center", justifyContent: "center",
          }}>
            {blade
              ? <Scissors size={22} strokeWidth={2.4}/>
              : <ScissorsLineDashed size={22} strokeWidth={2.4}/>}
          </div>
          <div>
            <div className="display" style={{fontSize: 18, fontWeight: 700, color: "var(--ink)"}}>
              Lame {blade ? "en marche" : "à l'arrêt"}
            </div>
            <div style={{fontSize: 12, color: "var(--ink-3)", marginTop: 2}}>
              {blade ? "2580 tr/min · 0.4 A" : "Coupée pour la sécurité"}
            </div>
          </div>
        </div>

        {/* Apple-style switch */}
        <motion.button
          onClick={onToggle}
          aria-label="Toggle blade"
          aria-pressed={blade}
          {...pressFeedback}
          style={{
            position: "relative",
            width: 56, height: 32, borderRadius: 999,
            background: blade
              ? "var(--grad-primary)"
              : "rgba(255,255,255,0.08)",
            transition: "background 0.2s",
            border: `1px solid ${blade ? "rgba(124,255,178,0.4)" : "var(--border-soft)"}`,
          }}
        >
          <motion.span
            animate={{x: blade ? 24 : 0}}
            transition={springSnap}
            style={{
              position: "absolute", top: 3, left: 3,
              width: 26, height: 26, borderRadius: "50%",
              background: "var(--bg-deep)",
              boxShadow: "0 4px 10px rgba(0,0,0,0.4)",
            }}
          />
        </motion.button>
      </div>
    </GlassCard>
  );
}

function QuickToggle({
  icon, label, defaultActive = false, pulse,
}: {icon: React.ReactNode; label: string; defaultActive?: boolean; pulse?: boolean}) {
  const [active, setActive] = useState(defaultActive);
  return (
    <motion.button
      onClick={() => setActive(a => !a)}
      whileHover={{y: -1}}
      whileTap={{scale: 0.97}}
      transition={springSnap}
      style={{
        position: "relative",
        display: "flex", flexDirection: "column", alignItems: "flex-start", gap: 8,
        padding: 14,
        borderRadius: 14,
        background: active
          ? "linear-gradient(135deg, rgba(124,255,178,0.12), rgba(43,170,102,0.04))"
          : "rgba(255,255,255,0.03)",
        border: `1px solid ${active ? "var(--border-glow)" : "var(--border-soft)"}`,
        color: active ? "var(--lime)" : "var(--ink-2)",
        overflow: "hidden",
        textAlign: "left",
      }}
    >
      {pulse && (
        <motion.span
          animate={{scale: [1, 1.6], opacity: [0.5, 0]}}
          transition={{duration: 1.8, ease: "easeOut", repeat: Infinity}}
          style={{
            position: "absolute", top: 12, right: 12,
            width: 8, height: 8, borderRadius: 4,
            background: "var(--lime)",
          }}
        />
      )}
      <span style={{
        width: 32, height: 32, borderRadius: 10,
        background: active ? "rgba(124,255,178,0.16)" : "rgba(255,255,255,0.04)",
        border: `1px solid ${active ? "rgba(124,255,178,0.3)" : "var(--border-soft)"}`,
        display: "flex", alignItems: "center", justifyContent: "center",
      }}>
        {icon}
      </span>
      <div style={{
        fontSize: 12, fontWeight: 600,
        color: active ? "var(--ink)" : "var(--ink-2)",
        letterSpacing: "0.02em",
      }}>
        {label}
      </div>
    </motion.button>
  );
}
