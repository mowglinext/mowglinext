import {motion, useMotionValue, useTransform, animate} from "framer-motion";
import {useEffect} from "react";

/**
 * Animated radial battery ring.
 *
 * Two concentric strokes (track + fill), the fill animates with a spring
 * from 0 to `percent` on mount, the gradient ID changes per accent so
 * multiple rings on a page don't collide.
 */

interface BatteryRingProps {
  percent: number;
  size?: number;
  thickness?: number;
  /** Lime by default. Override for charging / low-battery variants. */
  accent?: string;
  accentEnd?: string;
  gradientId?: string;
  charging?: boolean;
  children?: React.ReactNode;
}

export function BatteryRing({
  percent,
  size = 220,
  thickness = 12,
  accent = "#7CFFB2",
  accentEnd = "#2BAA66",
  gradientId = "concept-batt",
  charging = false,
  children,
}: BatteryRingProps) {
  const r = (size - thickness) / 2;
  const circ = 2 * Math.PI * r;
  const offset = useMotionValue(circ);
  const dashOffset = useTransform(offset, (v) => v);

  useEffect(() => {
    const target = circ * (1 - Math.max(0, Math.min(1, percent / 100)));
    const controls = animate(offset, target, {
      duration: 1.2,
      ease: [0.2, 0.7, 0.2, 1],
      delay: 0.15,
    });
    return controls.stop;
  }, [percent, circ, offset]);

  return (
    <div style={{position: "relative", width: size, height: size}}>
      <svg width={size} height={size} style={{display: "block", transform: "rotate(-90deg)"}}>
        <defs>
          <linearGradient id={gradientId} x1="0" y1="0" x2="1" y2="1">
            <stop offset="0%"  stopColor={accent}/>
            <stop offset="100%" stopColor={accentEnd}/>
          </linearGradient>
          <filter id={`${gradientId}-glow`}>
            <feGaussianBlur stdDeviation={4}/>
          </filter>
        </defs>

        {/* track */}
        <circle
          cx={size / 2} cy={size / 2} r={r}
          stroke="rgba(255,255,255,0.06)"
          strokeWidth={thickness}
          fill="none"
        />

        {/* glow halo (blurred copy of the fill) */}
        <motion.circle
          cx={size / 2} cy={size / 2} r={r}
          stroke={`url(#${gradientId})`}
          strokeWidth={thickness + 4}
          fill="none"
          strokeLinecap="round"
          strokeDasharray={circ}
          style={{strokeDashoffset: dashOffset, opacity: 0.4}}
          filter={`url(#${gradientId}-glow)`}
        />

        {/* main fill */}
        <motion.circle
          cx={size / 2} cy={size / 2} r={r}
          stroke={`url(#${gradientId})`}
          strokeWidth={thickness}
          fill="none"
          strokeLinecap="round"
          strokeDasharray={circ}
          style={{strokeDashoffset: dashOffset}}
        />
      </svg>

      {/* tick marker at 100% -- subtle line marker */}
      <div style={{
        position: "absolute", top: thickness / 2 - 1, left: "50%",
        width: 2, height: 6, borderRadius: 2,
        background: "rgba(255,255,255,0.35)",
        transform: "translateX(-50%)",
      }}/>

      {/* charging bolt overlay */}
      {charging && (
        <motion.div
          initial={{opacity: 0, scale: 0.8}}
          animate={{opacity: 1, scale: 1}}
          transition={{delay: 0.4}}
          style={{
            position: "absolute", inset: 0,
            display: "flex", alignItems: "center", justifyContent: "center",
            pointerEvents: "none",
          }}
        >
          <div style={{
            position: "absolute", top: -2, right: size * 0.18,
            display: "flex", alignItems: "center", gap: 4,
            padding: "3px 8px 3px 6px",
            background: "var(--bg-deep)",
            border: "1px solid var(--lime)",
            borderRadius: "var(--radius-pill)",
            fontSize: 10, fontWeight: 700, color: "var(--lime)",
            letterSpacing: "0.08em", textTransform: "uppercase",
          }}>
            <svg width="9" height="11" viewBox="0 0 9 11" fill="currentColor">
              <path d="M5 0L0 6h3l-1 5 5-6H4l1-5z"/>
            </svg>
            charging
          </div>
        </motion.div>
      )}

      {/* center content */}
      <div style={{
        position: "absolute", inset: 0,
        display: "flex", flexDirection: "column",
        alignItems: "center", justifyContent: "center",
      }}>
        {children}
      </div>
    </div>
  );
}
