import {motion, LayoutGroup} from "framer-motion";
import {Leaf, Zap, Moon} from "lucide-react";
import {springSnap} from "../motion";

/**
 * Eco / Sport / Silencieux segmented selector with a sliding lime pill.
 * Uses Framer's layoutId to morph the active background between options.
 */

export type Mode = "eco" | "sport" | "silent";

const MODES: {key: Mode; label: string; icon: typeof Leaf; tagline: string}[] = [
  {key: "eco",    label: "Eco",        icon: Leaf, tagline: "Plus long, plus silencieux"},
  {key: "sport",  label: "Sport",      icon: Zap,  tagline: "Vitesse + couverture rapide"},
  {key: "silent", label: "Silencieux", icon: Moon, tagline: "Vitesse réduite, sons feutrés"},
];

interface ModeSegmentProps {
  value: Mode;
  onChange: (next: Mode) => void;
}

export function ModeSegment({value, onChange}: ModeSegmentProps) {
  const active = MODES.find(m => m.key === value)!;
  return (
    <div>
      <LayoutGroup>
        <div
          role="tablist"
          style={{
            display: "grid",
            gridTemplateColumns: "repeat(3, 1fr)",
            gap: 4,
            padding: 4,
            background: "rgba(255, 255, 255, 0.04)",
            borderRadius: "var(--radius-pill)",
            border: "1px solid var(--border-soft)",
          }}
        >
          {MODES.map(({key, label, icon: Icon}) => {
            const isActive = key === value;
            return (
              <button
                key={key}
                role="tab"
                aria-selected={isActive}
                onClick={() => onChange(key)}
                style={{
                  position: "relative",
                  display: "flex", alignItems: "center", justifyContent: "center", gap: 8,
                  padding: "10px 4px",
                  borderRadius: "var(--radius-pill)",
                  fontSize: 13, fontWeight: 600,
                  color: isActive ? "#02110D" : "var(--ink-2)",
                  zIndex: 1,
                  transition: "color 0.15s",
                }}
              >
                {isActive && (
                  <motion.span
                    layoutId="mode-pill"
                    style={{
                      position: "absolute", inset: 0,
                      background: "var(--grad-primary)",
                      borderRadius: "var(--radius-pill)",
                      boxShadow:
                        "0 8px 24px -8px rgba(124,255,178,0.5), inset 0 1px 0 rgba(255,255,255,0.35)",
                      zIndex: -1,
                    }}
                    transition={springSnap}
                  />
                )}
                <Icon size={14} strokeWidth={2.3}/>
                <span>{label}</span>
              </button>
            );
          })}
        </div>
      </LayoutGroup>
      <div style={{
        fontSize: 11, color: "var(--ink-3)", marginTop: 8,
        textAlign: "center", letterSpacing: "0.02em",
      }}>
        {active.tagline}
      </div>
    </div>
  );
}
