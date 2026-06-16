import {motion} from "framer-motion";
import {Play, Square, Home, AlertTriangle} from "lucide-react";
import {pressFeedback, springSnap} from "../motion";

/**
 * Primary action cluster -- big Play (lime gradient w/ inner shine), with a
 * Home + emergency-Stop as glass secondaries. State drives the primary:
 * idle shows Play (start mowing); "playing" morphs it to a Square that
 * stops the run and returns to the dock (there is NO true pause — the
 * "playing" primary maps to the same HOME command as the Home secondary, so
 * the glyph + label say "stop & return", not "pause", to match reality).
 */

type Phase = "idle" | "playing" | "returning" | "alert";

interface ActionClusterProps {
  phase: Phase;
  onStart: () => void;
  onPause: () => void;
  onHome: () => void;
  onStop: () => void;
}

export function ActionCluster({phase, onStart, onPause, onHome, onStop}: ActionClusterProps) {
  const primaryPlaying = phase === "playing";

  return (
    <div style={{
      display: "flex", alignItems: "center", justifyContent: "center", gap: 14,
    }}>
      {/* secondary: stop */}
      <SecondaryButton
        ariaLabel="Stop d'urgence"
        onClick={onStop}
        tone="danger"
      >
        <AlertTriangle size={20} strokeWidth={2.2}/>
      </SecondaryButton>

      {/* primary: play / pause */}
      <motion.button
        {...pressFeedback}
        onClick={primaryPlaying ? onPause : onStart}
        aria-label={primaryPlaying ? "Arrêter et rentrer à la base" : "Démarrer la tonte"}
        style={{
          position: "relative",
          width: 84, height: 84, borderRadius: "50%",
          background: "var(--grad-primary)",
          color: "#02110D",
          display: "flex", alignItems: "center", justifyContent: "center",
          boxShadow:
            "0 18px 40px -10px rgba(124,255,178,0.45), inset 0 1px 0 rgba(255,255,255,0.4), inset 0 -12px 24px rgba(43,170,102,0.4)",
          overflow: "hidden",
        }}
      >
        {/* inner shine sweep -- clip to the circle by inheriting the parent's
            border-radius (overflow:hidden on the framer-motion button is
            unreliable because the press transform creates a new stacking
            context). */}
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
          key={primaryPlaying ? "pause" : "play"}
          initial={{scale: 0.6, opacity: 0}}
          animate={{scale: 1, opacity: 1}}
          transition={springSnap}
          style={{position: "relative"}}
        >
          {primaryPlaying
            ? <Square size={28} strokeWidth={2.4} fill="currentColor"/>
            : <Play  size={32} strokeWidth={2.4} fill="currentColor" style={{marginLeft: 3}}/>}
        </motion.div>
      </motion.button>

      {/* secondary: home */}
      <SecondaryButton
        ariaLabel="Retour à la base"
        onClick={onHome}
        tone={phase === "returning" ? "active" : "default"}
      >
        <Home size={20} strokeWidth={2.2}/>
      </SecondaryButton>
    </div>
  );
}

interface SecondaryProps {
  children: React.ReactNode;
  ariaLabel: string;
  onClick: () => void;
  tone?: "default" | "active" | "danger";
}

function SecondaryButton({children, ariaLabel, onClick, tone = "default"}: SecondaryProps) {
  const colors = {
    default: {bg: "var(--bg-elevated)",       border: "var(--border-soft)",  color: "var(--ink)"},
    active:  {bg: "rgba(69,214,232,0.14)",    border: "rgba(69,214,232,0.5)", color: "var(--aurora-cyan)"},
    danger:  {bg: "rgba(255,107,122,0.12)",   border: "rgba(255,107,122,0.5)", color: "var(--rose)"},
  }[tone];
  return (
    <motion.button
      {...pressFeedback}
      onClick={onClick}
      aria-label={ariaLabel}
      style={{
        width: 56, height: 56, borderRadius: "50%",
        background: colors.bg,
        border: `1px solid ${colors.border}`,
        color: colors.color,
        display: "flex", alignItems: "center", justifyContent: "center",
        backdropFilter: "blur(20px)",
      }}
    >
      {children}
    </motion.button>
  );
}
