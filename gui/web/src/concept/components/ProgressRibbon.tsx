import {motion} from "framer-motion";

/**
 * Horizontal segment-strip showing today's progress.
 *
 *   ▰▰▰▰▰▰▰▰▱▱▱▱▱▱▱▱▱▱   42 %
 *
 * Filled lime segments animate in left-to-right; the un-filled segments
 * stay quiet so the gain reads clearly. Used in the hero card.
 */

interface ProgressRibbonProps {
  /** 0..1 */
  value: number;
  segments?: number;
  height?: number;
}

export function ProgressRibbon({
  value,
  segments = 24,
  height = 6,
}: ProgressRibbonProps) {
  const filled = Math.round(value * segments);
  return (
    <div
      role="progressbar"
      aria-valuenow={Math.round(value * 100)}
      style={{
        display: "flex", gap: 3, width: "100%", height,
      }}
    >
      {Array.from({length: segments}).map((_, i) => {
        const isFilled = i < filled;
        return (
          <motion.div
            key={i}
            initial={{opacity: 0, scaleY: 0.4}}
            animate={{
              opacity: isFilled ? 1 : 0.18,
              scaleY: 1,
              background: isFilled
                ? "linear-gradient(180deg, #7CFFB2, #2BAA66)"
                : "rgba(255, 255, 255, 0.12)",
            }}
            transition={{
              duration: 0.5,
              delay: 0.04 * i + 0.2,
              ease: [0.2, 0.7, 0.2, 1],
            }}
            style={{
              flex: 1,
              borderRadius: 2,
              boxShadow: isFilled ? "0 0 8px rgba(124,255,178,0.4)" : "none",
              transformOrigin: "bottom",
            }}
          />
        );
      })}
    </div>
  );
}
