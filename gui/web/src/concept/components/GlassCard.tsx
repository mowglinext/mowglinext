import type {CSSProperties, ReactNode} from "react";
import {motion, type Variants} from "framer-motion";
import {riseFade} from "../motion";

/**
 * Glass surface with the luminous edge gradient handled by `.glass` in
 * concept.css. Pass `glow` for the lime-halo variant used on the hero.
 */

type Variant = "default" | "elevated" | "glow";

interface GlassCardProps {
  children: ReactNode;
  variant?: Variant;
  padding?: number | string;
  className?: string;
  style?: CSSProperties;
  /** Wrap in a motion.div with rise-fade entrance. */
  animate?: boolean;
  /** Custom variants override -- e.g. for staggered children. */
  motionVariants?: Variants;
  onClick?: () => void;
}

export function GlassCard({
  children,
  variant = "default",
  padding = 20,
  className = "",
  style,
  animate = false,
  motionVariants,
  onClick,
}: GlassCardProps) {
  const inner = (
    <div
      onClick={onClick}
      className={`glass ${className}`}
      style={{
        padding,
        background: variant === "elevated"
          ? "var(--bg-elevated)"
          : variant === "glow"
            ? "var(--bg-card)"
            : "var(--bg-card)",
        boxShadow: variant === "glow"
          ? "var(--shadow-card), var(--shadow-glow-lime)"
          : "var(--shadow-card)",
        cursor: onClick ? "pointer" : undefined,
        ...style,
      }}
    >
      {children}
    </div>
  );

  if (!animate) return inner;
  return (
    <motion.div variants={motionVariants ?? riseFade}>
      {inner}
    </motion.div>
  );
}
