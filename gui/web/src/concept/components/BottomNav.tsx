import {motion, LayoutGroup} from "framer-motion";
import {Home, Map as MapIcon, Gamepad2, Calendar, BarChart3} from "lucide-react";
import {springSnap} from "../motion";

export type Screen = "home" | "map" | "controls" | "schedule" | "stats";

const ITEMS: {key: Screen; label: string; icon: typeof Home}[] = [
  {key: "home",     label: "Accueil",     icon: Home},
  {key: "map",      label: "Carte",       icon: MapIcon},
  {key: "controls", label: "Contrôle",    icon: Gamepad2},
  {key: "schedule", label: "Planning",    icon: Calendar},
  {key: "stats",    label: "Stats",       icon: BarChart3},
];

interface BottomNavProps {
  active: Screen;
  onChange: (next: Screen) => void;
}

export function BottomNav({active, onChange}: BottomNavProps) {
  return (
    <nav style={{
      position: "fixed", left: 0, right: 0, bottom: 0,
      paddingBottom: "calc(env(safe-area-inset-bottom, 0px) + 10px)",
      paddingTop: 10,
      paddingLeft: 14, paddingRight: 14,
      background:
        "linear-gradient(180deg, rgba(2,17,13,0) 0%, rgba(2,17,13,0.85) 30%, rgba(2,17,13,0.95) 100%)",
      backdropFilter: "blur(22px) saturate(140%)",
      zIndex: 50,
    }}>
      <LayoutGroup>
        <div style={{
          display: "grid",
          gridTemplateColumns: `repeat(${ITEMS.length}, 1fr)`,
          gap: 2,
          padding: 6,
          background: "rgba(255,255,255,0.04)",
          border: "1px solid var(--border-soft)",
          borderRadius: "var(--radius-pill)",
          backdropFilter: "blur(28px)",
        }}>
          {ITEMS.map(({key, label, icon: Icon}) => {
            const isActive = key === active;
            return (
              <button
                key={key}
                onClick={() => onChange(key)}
                aria-current={isActive ? "page" : undefined}
                style={{
                  position: "relative",
                  display: "flex", flexDirection: "column",
                  alignItems: "center", justifyContent: "center", gap: 2,
                  padding: "10px 4px 8px",
                  borderRadius: "var(--radius-pill)",
                  color: isActive ? "var(--bg-deep)" : "var(--ink-2)",
                  fontSize: 10, fontWeight: 600,
                  letterSpacing: "0.02em",
                  zIndex: 1,
                  transition: "color 0.15s",
                }}
              >
                {isActive && (
                  <motion.span
                    layoutId="nav-pill"
                    style={{
                      position: "absolute", inset: 0,
                      background: "var(--grad-primary)",
                      borderRadius: "var(--radius-pill)",
                      boxShadow:
                        "0 6px 20px -6px rgba(124,255,178,0.55), inset 0 1px 0 rgba(255,255,255,0.3)",
                      zIndex: -1,
                    }}
                    transition={springSnap}
                  />
                )}
                <Icon size={18} strokeWidth={isActive ? 2.4 : 2}/>
                <span>{label}</span>
              </button>
            );
          })}
        </div>
      </LayoutGroup>
    </nav>
  );
}
