import {motion} from "framer-motion";
import {Play, Pause, Home, AlertTriangle, RotateCcw, Square} from "lucide-react";
import {useTranslation} from "react-i18next";
import {pressFeedback, springSnap} from "../motion";
import {useThemeMode} from "../../theme/ThemeContext.tsx";
import type {ChargeHold} from "../../utils/chargeHold.ts";

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

export type {ChargeHold} from "../../utils/chargeHold.ts";

interface ActionClusterProps {
  phase: Phase;
  onStart: () => void;
  onPause: () => void;
  onHome: () => void;
  onStop: () => void;
  onRearm: () => void;
  chargeHold?: ChargeHold;
  onResume?: () => void;
  onCancelMowing?: () => void;
}

export function ActionCluster({phase, onStart, onPause, onHome, onStop, onRearm, chargeHold, onResume, onCancelMowing}: ActionClusterProps) {
  const {t} = useTranslation();
  const {displayMode} = useThemeMode();
  const primaryPlaying = phase === "playing";
  const primaryAlert = phase === "alert";
  const manualResumeEligible = !!chargeHold?.autoResume && chargeHold.batteryPercent >= chargeHold.manualResumePercent;
  const chargeHoldMessage = chargeHold?.autoResume
    ? t("actionCluster.chargeHold", {
        message: chargeHold.stateName === "CRITICAL_BATTERY_CHARGING"
          ? t("actionCluster.criticalChargeHold")
          : t("actionCluster.lowChargeHold"),
        full: chargeHold.batteryFullPercent,
        resumeAt: chargeHold.manualResumePercent,
      })
    : t("actionCluster.manualChargeHold");

  // A charge hold is an active mowing session parked by the battery guards,
  // unlike an ordinary idle mower that happens to be charging on the dock.
  if (chargeHold) {
    return (
      <div style={{display: "flex", flexDirection: "column", alignItems: "center", gap: 10}}>
        <div style={{fontSize: 12, lineHeight: 1.45, color: "var(--ink-2)", textAlign: "center", maxWidth: 380}}>
          {chargeHoldMessage}
        </div>
        <div style={{display: "flex", alignItems: "center", justifyContent: "center", gap: 14}}>
          <SecondaryButton ariaLabel={t("actionCluster.emergencyStop")} onClick={onStop} tone="danger" displayMode={displayMode}>
            <AlertTriangle size={20} strokeWidth={2.2}/>
          </SecondaryButton>
          {chargeHold.autoResume && <div style={{display: "flex", flexDirection: "column", alignItems: "center", gap: 5}}>
              <PrimaryButton ariaLabel={t("actionCluster.resumeNow")} onClick={onResume ?? onStart} disabled={!manualResumeEligible} displayMode={displayMode}>
                <Play size={32} strokeWidth={2.4} fill="currentColor" style={{marginLeft: 3}}/>
              </PrimaryButton>
              <span style={{fontSize: 11, fontWeight: 600, color: "var(--ink-2)"}}>{t("actionCluster.resumeNow")}</span>
            </div>}
          <div style={{display: "flex", flexDirection: "column", alignItems: "center", gap: 5}}>
            <SecondaryButton ariaLabel={t("actionCluster.cancelMowing")} onClick={onCancelMowing ?? onPause} tone="default" displayMode={displayMode}>
              <Square size={18} strokeWidth={2.2} fill="currentColor"/>
            </SecondaryButton>
            <span style={{fontSize: 11, fontWeight: 600, color: "var(--ink-2)"}}>{t("actionCluster.cancelMowing")}</span>
          </div>
        </div>
      </div>
    );
  }

  return (
    <div style={{
      display: "flex", alignItems: "center", justifyContent: "center", gap: 14,
    }}>
      {/* secondary: stop */}
      <SecondaryButton
        ariaLabel={t('actionCluster.emergencyStop')}
        onClick={onStop}
        tone="danger"
        displayMode={displayMode}
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
        {/* Visual mode restores the moving sheen; the other modes preserve
            the same action hierarchy without its continuous compositing. */}
        <span aria-hidden style={{
          position: "absolute", inset: 0,
          borderRadius: "inherit",
          background: "linear-gradient(115deg, transparent 25%, rgba(255,255,255,0.45) 50%, transparent 75%)",
          mixBlendMode: "overlay",
          opacity: 0.55,
          transform: displayMode === "visual" ? "translateX(-100%)" : undefined,
          animation: displayMode === "visual" ? "concept-shine 3.6s var(--ease-out) infinite" : undefined,
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
        displayMode={displayMode}
      >
        <Home size={20} strokeWidth={2.2}/>
      </SecondaryButton>
    </div>
  );
}

interface PrimaryProps {
  children: React.ReactNode;
  ariaLabel: string;
  onClick: () => void;
  displayMode: "visual" | "balanced" | "efficient";
  disabled?: boolean;
}

function PrimaryButton({children, ariaLabel, onClick, displayMode, disabled = false}: PrimaryProps) {
  return (
    <motion.button
      {...pressFeedback}
      onClick={onClick}
      aria-label={ariaLabel}
      disabled={disabled}
      style={{
        position: "relative", width: 84, height: 84, borderRadius: "50%",
        background: "var(--grad-primary)", color: "#02110D",
        display: "flex", alignItems: "center", justifyContent: "center",
        boxShadow: "0 18px 40px -10px rgba(124,255,178,0.45), inset 0 1px 0 rgba(255,255,255,0.4), inset 0 -12px 24px rgba(43,170,102,0.4)",
        overflow: "hidden", opacity: disabled ? 0.45 : 1, cursor: disabled ? "not-allowed" : undefined,
      }}
    >
      <span aria-hidden style={{
        position: "absolute", inset: 0, borderRadius: "inherit",
        background: "linear-gradient(115deg, transparent 25%, rgba(255,255,255,0.45) 50%, transparent 75%)",
        mixBlendMode: "overlay", opacity: 0.55,
        transform: displayMode === "visual" ? "translateX(-100%)" : undefined,
        animation: displayMode === "visual" ? "concept-shine 3.6s var(--ease-out) infinite" : undefined,
        pointerEvents: "none",
      }}/>
      <span style={{position: "relative", display: "flex"}}>{children}</span>
    </motion.button>
  );
}

interface SecondaryProps {
  children: React.ReactNode;
  ariaLabel: string;
  onClick: () => void;
  tone?: "default" | "active" | "danger";
  displayMode: "visual" | "balanced" | "efficient";
}

function SecondaryButton({children, ariaLabel, onClick, tone = "default", displayMode}: SecondaryProps) {
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
        backdropFilter: displayMode === "visual" ? "blur(20px)" : undefined,
      }}
    >
      {children}
    </motion.button>
  );
}
