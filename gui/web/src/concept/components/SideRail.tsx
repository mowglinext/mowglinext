import {motion, LayoutGroup} from "framer-motion";
import {Home, Map as MapIcon, Gamepad2, Calendar, BarChart3} from "lucide-react";
import {useTranslation} from "react-i18next";
import {springSnap} from "../motion";
import type {Screen} from "./BottomNav";

/**
 * Desktop counterpart of BottomNav. A glass rail pinned to the left
 * edge, 88px wide, with the brand mark at top and 5 vertical icon
 * buttons. Active item shares the same `layoutId` as the mobile pill
 * so route changes feel continuous if the viewport flips.
 */

const ITEMS: {key: Screen; labelKey: string; icon: typeof Home}[] = [
  {key: "home",     labelKey: "conceptSideRail.home",     icon: Home},
  {key: "map",      labelKey: "conceptSideRail.map",      icon: MapIcon},
  {key: "controls", labelKey: "conceptSideRail.controls", icon: Gamepad2},
  {key: "schedule", labelKey: "conceptSideRail.schedule", icon: Calendar},
  {key: "stats",    labelKey: "conceptSideRail.stats",    icon: BarChart3},
];

interface SideRailProps {
  active: Screen;
  onChange: (next: Screen) => void;
}

export function SideRail({active, onChange}: SideRailProps) {
  const {t} = useTranslation();
  return (
    <aside style={{
      position: "fixed", top: 0, bottom: 0, left: 0,
      width: 88,
      display: "flex", flexDirection: "column",
      paddingTop: "max(24px, env(safe-area-inset-top, 0px))",
      paddingBottom: "max(24px, env(safe-area-inset-bottom, 0px))",
      background:
        "linear-gradient(180deg, rgba(2,17,13,0.92) 0%, rgba(2,17,13,0.85) 100%)",
      borderRight: "1px solid var(--border-soft)",
      backdropFilter: "blur(22px) saturate(140%)",
      zIndex: 50,
    }}>
      {/* Brand mark */}
      <div style={{
        display: "flex", flexDirection: "column", alignItems: "center", gap: 6,
        marginBottom: 28,
      }}>
        <div style={{
          width: 44, height: 44, borderRadius: 14,
          background: "var(--grad-primary)",
          display: "flex", alignItems: "center", justifyContent: "center",
          color: "var(--bg-deep)",
          boxShadow: "0 10px 24px -8px rgba(124,255,178,0.5)",
          fontFamily: "var(--font-display)",
          fontWeight: 900, fontSize: 22, lineHeight: 1,
        }}>
          m
        </div>
        <div style={{
          fontSize: 9, color: "var(--ink-3)",
          letterSpacing: "0.18em", textTransform: "uppercase",
          fontWeight: 700,
        }}>
          Mowgli
        </div>
      </div>

      {/* Vertical nav */}
      <LayoutGroup>
        <nav style={{
          display: "flex", flexDirection: "column", gap: 4,
          padding: "0 12px", flex: 1,
        }}>
          {ITEMS.map(({key, labelKey, icon: Icon}) => {
            const isActive = key === active;
            return (
              <button
                key={key}
                onClick={() => onChange(key)}
                aria-current={isActive ? "page" : undefined}
                style={{
                  position: "relative",
                  display: "flex", flexDirection: "column",
                  alignItems: "center", justifyContent: "center", gap: 4,
                  padding: "14px 4px 12px",
                  borderRadius: 14,
                  color: isActive ? "var(--bg-deep)" : "var(--ink-2)",
                  fontSize: 9, fontWeight: 700,
                  letterSpacing: "0.06em", textTransform: "uppercase",
                  zIndex: 1,
                  transition: "color 0.15s",
                }}
              >
                {isActive && (
                  <motion.span
                    layoutId="nav-rail"
                    style={{
                      position: "absolute", inset: 0,
                      background: "var(--grad-primary)",
                      borderRadius: 14,
                      boxShadow:
                        "0 12px 26px -6px rgba(124,255,178,0.5), inset 0 1px 0 rgba(255,255,255,0.32)",
                      zIndex: -1,
                    }}
                    transition={springSnap}
                  />
                )}
                <Icon size={20} strokeWidth={isActive ? 2.4 : 2}/>
                <span>{t(labelKey)}</span>
              </button>
            );
          })}
        </nav>
      </LayoutGroup>
    </aside>
  );
}
