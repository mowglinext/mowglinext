import {motion} from "framer-motion";
import {Play, Pause, Home, AlertTriangle, RotateCcw} from "lucide-react";
import {useTranslation} from "react-i18next";
import {pressFeedback, springSnap} from "../motion";

/**
 * Primary action cluster -- big Play (lime gradient w/ inner shine), with a
 * Home + emergency-Stop as glass secondaries. State drives the primary:
 * idle shows Play (start mowing); "playing" morphs it to a Pause glyph that
 * issues a true stop-in-place (COMMAND_STOP=8 → StopHoldSequence: mower off,
 * halt in place, Nav2 left up so the mission can resume, no dock drive). The
 * separate Home secondary still maps to the HOME command (return to dock). In
 * "alert" (latched emergency) it becomes a Re-arm button that
 * clears the emergency, otherwise the operator is stuck (Play is inert while
 * the EmergencyGuard halts the tree).
 */

type Phase = "idle" | "playing" | "returning" | "alert";

interface ActionClusterProps {
  phase: Phase;
  onStart: () => void;
  onPause: () => void;
  onHome: () => void;
  onStop: () => void;
  onRearm: () => void;
}

export function ActionCluster({phase, onStart, onPause, onHome, onStop, onRearm}: ActionClusterProps) {
  const {t} = useTranslation();
  const primaryPlaying = phase === "playing";
  const primaryAlert = phase === "alert";

  return (
    <div style={{
      display: "flex", alignItems: "center", justifyContent: "center", gap: 14,
    }}>
      {/* secondary: stop */}
      <SecondaryButton
        ariaLabel={t('actionCluster.emergencyStop')}
        onClick={onStop}
        tone="danger"
      >
        <AlertTriangle size={20} strokeWidth={2.2}/>
      </SecondaryButton>

      {/* primary: re-arm (latched emergency) / pause-in-place (playing) / play */}
      <motion.button
        {...pressFeedback}
        onClick={primaryAlert ? onRearm : primaryPlaying ? onPause : onStart}
        aria-label={primaryAlert
          ? t('actionCluster.rearm')
          : primaryPlaying
            ? t('actionCluster.pause')
            : t('actionCluster.startMowing')}
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
          key={primaryAlert ? "rearm" : primaryPlaying ? "pause" : "play"}
          initial={{scale: 0.6, opacity: 0}}
          animate={{scale: 1, opacity: 1}}
          transition={springSnap}
          style={{position: "relative"}}
        >
          {primaryAlert
            ? <RotateCcw size={28} strokeWidth={2.4}/>
            : primaryPlaying
              ? <Pause size={28} strokeWidth={2.4} fill="currentColor"/>
              : <Play size={32} strokeWidth={2.4} fill="currentColor" style={{marginLeft: 3}}/>}
        </motion.div>
      </motion.button>

      {/* secondary: home */}
      <SecondaryButton
        ariaLabel={t('actionCluster.returnToBase')}
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
