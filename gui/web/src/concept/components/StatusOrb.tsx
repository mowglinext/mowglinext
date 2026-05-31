import type {ReactNode} from "react";
import {motion} from "framer-motion";

/**
 * Pulsing status orb used in the dashboard header. Three accent colors
 * map to three lifecycles (live / resting / alert). The pulse rings are
 * pure SVG so they scale + animate without DOM cost.
 */

type Tone = "live" | "resting" | "alert" | "charging";

const TONE_MAP: Record<Tone, {color: string; glow: string; pulse: boolean}> = {
  live:     {color: "var(--lime)",       glow: "rgba(124, 255, 178, 0.65)", pulse: true},
  resting:  {color: "var(--aurora-cyan)", glow: "rgba(69, 214, 232, 0.55)",  pulse: false},
  alert:    {color: "var(--rose)",       glow: "rgba(255, 107, 122, 0.65)", pulse: true},
  charging: {color: "var(--lime)",       glow: "rgba(124, 255, 178, 0.5)",  pulse: false},
};

interface StatusOrbProps {
  tone?: Tone;
  size?: number;
  label?: ReactNode;
}

export function StatusOrb({tone = "live", size = 10, label}: StatusOrbProps) {
  const t = TONE_MAP[tone];
  return (
    <div style={{
      display: "inline-flex", alignItems: "center", gap: 8,
    }}>
      <div style={{
        position: "relative", width: size, height: size,
      }}>
        {/* halo */}
        <span style={{
          position: "absolute", inset: -8,
          borderRadius: "50%",
          background: t.glow,
          filter: "blur(8px)",
          opacity: 0.7,
        }}/>
        {/* pulse ring */}
        {t.pulse && (
          <motion.span
            style={{
              position: "absolute", inset: 0,
              borderRadius: "50%",
              boxShadow: `0 0 0 2px ${t.color}`,
            }}
            animate={{scale: [1, 2.4], opacity: [0.6, 0]}}
            transition={{duration: 1.8, ease: "easeOut", repeat: Infinity}}
          />
        )}
        {/* core */}
        <span style={{
          position: "absolute", inset: 0,
          borderRadius: "50%",
          background: t.color,
          boxShadow: `0 0 12px ${t.glow}`,
        }}/>
      </div>
      {label && (
        <span style={{
          fontSize: 12, color: "var(--ink-2)",
          letterSpacing: "0.06em", textTransform: "uppercase", fontWeight: 600,
        }}>
          {label}
        </span>
      )}
    </div>
  );
}
