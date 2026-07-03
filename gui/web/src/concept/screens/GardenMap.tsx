import {useState} from "react";
import {motion, AnimatePresence} from "framer-motion";
import {useTranslation} from "react-i18next";
import {
  Layers, Maximize2, Plus, Edit3, Trash2, Navigation,
  ChevronUp, MapPin,
} from "lucide-react";

import {GlassCard} from "../components/GlassCard";
import {LiveMapMini} from "../components/LiveMapMini";
import {NoiseTexture} from "../components/NoiseTexture";
import {staggerParent, riseFade, springSnap, springSoft} from "../motion";
import {useViewport} from "../useViewport";

/**
 * Garden Map screen. Full-bleed map + floating glass panels.
 *
 * Mobile: bottom sheet that drags up to reveal zones + dock status.
 * Desktop: side panels left (zones) + right (live robot + actions).
 */

interface Zone {
  id: string;
  name: string;
  area: number;     // m²
  coverage: number; // 0..1
  active: boolean;
  accent: "lime" | "cyan" | "amber" | "rose";
}

const ZONES: Zone[] = [
  {id: "z1", name: "gardenMap.zoneSouthGarden", area: 184, coverage: 0.42, active: true,  accent: "lime"},
  {id: "z2", name: "gardenMap.zoneNorthGarden", area: 116, coverage: 0,    active: false, accent: "cyan"},
  {id: "z3", name: "gardenMap.zoneFrontLawn",   area: 92,  coverage: 1.0,  active: false, accent: "amber"},
  {id: "z4", name: "gardenMap.zoneSidePath",    area: 38,  coverage: 0,    active: false, accent: "rose"},
];

export function GardenMap() {
  const {t} = useTranslation();
  const vp = useViewport();
  const [, setLayer] = useState<"satellite" | "vector">("vector");
  const [selectedZone, setSelectedZone] = useState<string>("z1");
  const [sheetOpen, setSheetOpen] = useState(false);

  return (
    <div style={{
      position: "relative", width: "100%", height: "100dvh",
      overflow: "hidden",
    }}>
      {/* Full-bleed canvas */}
      <div style={{
        position: "absolute", inset: 0,
        background:
          "radial-gradient(circle at 30% 25%, rgba(69,214,232,0.16) 0%, transparent 55%)," +
          "radial-gradient(circle at 75% 80%, rgba(124,255,178,0.18) 0%, transparent 60%)," +
          "var(--bg-deep)",
      }}/>
      <NoiseTexture/>

      {/* Map -- larger version of LiveMapMini at full height */}
      <div style={{
        position: "absolute", inset: 0,
        display: "flex", alignItems: "center", justifyContent: "center",
      }}>
        <div style={{
          width: vp.isAtLeastTablet ? "78%" : "92%",
          maxWidth: 1100,
        }}>
          <LiveMapMini
            coverage={0.42}
            robot={{x: 0.58, y: 0.5, heading: 38}}
            height={vp.isAtLeastTablet ? 560 : 420}
          />
        </div>
      </div>

      {/* Top status pill (always visible) */}
      <motion.div
        initial={{opacity: 0, y: -10}}
        animate={{opacity: 1, y: 0}}
        transition={{...springSoft, delay: 0.1}}
        style={{
          position: "absolute",
          top: vp.isAtLeastTablet ? 28 : "max(16px, env(safe-area-inset-top, 0px) + 8px)",
          left: vp.isAtLeastTablet ? 28 : 16,
          right: vp.isAtLeastTablet ? 28 : 16,
          display: "flex", justifyContent: "space-between", gap: 12,
          zIndex: 10,
        }}
      >
        <GlassCard padding="10px 14px" style={{display: "inline-flex", alignItems: "center", gap: 10}}>
          <span style={{
            width: 8, height: 8, borderRadius: 4,
            background: "var(--lime)",
            boxShadow: "0 0 10px rgba(124,255,178,0.7)",
          }}/>
          <div className="display" style={{
            fontSize: 14, fontWeight: 700, color: "var(--ink)",
            letterSpacing: "-0.01em",
          }}>
            {t('gardenMap.zoneSouthGarden')}
          </div>
          <span style={{
            fontSize: 11, color: "var(--ink-3)",
            paddingLeft: 8, marginLeft: 4, borderLeft: "1px solid var(--border-soft)",
          }}>
            {t('gardenMap.statusMowed')}
          </span>
        </GlassCard>

        {/* Layer toggle */}
        <div style={{display: "flex", gap: 8}}>
          <MapToolButton onClick={() => setLayer(l => l === "satellite" ? "vector" : "satellite")}>
            <Layers size={16} strokeWidth={2.2}/>
          </MapToolButton>
          <MapToolButton>
            <Maximize2 size={16} strokeWidth={2.2}/>
          </MapToolButton>
        </div>
      </motion.div>

      {/* Desktop side panels */}
      {vp.isAtLeastTablet && (
        <>
          <motion.div
            variants={staggerParent(0.06, 0.15)}
            initial="hidden" animate="show"
            style={{
              position: "absolute", left: 28, top: 96, bottom: 28,
              width: 320,
              display: "flex", flexDirection: "column", gap: 16,
              zIndex: 5,
            }}
          >
            <motion.div variants={riseFade}>
              <GlassCard padding={20}>
                <div style={{
                  display: "flex", alignItems: "center", justifyContent: "space-between",
                  marginBottom: 16,
                }}>
                  <div>
                    <div style={{
                      fontSize: 10, color: "var(--ink-3)",
                      letterSpacing: "0.08em", textTransform: "uppercase",
                      fontWeight: 700,
                    }}>
                      {t('gardenMap.gardenZones')}
                    </div>
                    <div className="display" style={{
                      fontSize: 22, fontWeight: 700, marginTop: 4,
                      letterSpacing: "-0.02em",
                    }}>
                      {t('gardenMap.zonesSummary')}
                    </div>
                  </div>
                  <button style={{
                    width: 36, height: 36, borderRadius: 10,
                    background: "var(--grad-primary)",
                    color: "var(--bg-deep)",
                    display: "flex", alignItems: "center", justifyContent: "center",
                    boxShadow: "0 8px 20px -6px rgba(124,255,178,0.5)",
                  }}>
                    <Plus size={18} strokeWidth={2.4}/>
                  </button>
                </div>

                <div style={{display: "flex", flexDirection: "column", gap: 8}}>
                  {ZONES.map(z => (
                    <ZoneRow
                      key={z.id}
                      zone={z}
                      isSelected={z.id === selectedZone}
                      onClick={() => setSelectedZone(z.id)}
                    />
                  ))}
                </div>
              </GlassCard>
            </motion.div>
          </motion.div>

          {/* Right floating: live robot + actions */}
          <motion.div
            variants={staggerParent(0.06, 0.2)}
            initial="hidden" animate="show"
            style={{
              position: "absolute", right: 28, top: 96, bottom: 28,
              width: 300,
              display: "flex", flexDirection: "column", gap: 16,
              zIndex: 5,
            }}
          >
            <motion.div variants={riseFade}>
              <RobotTelemetryCard/>
            </motion.div>
            <motion.div variants={riseFade}>
              <ZoneActionsCard/>
            </motion.div>
          </motion.div>
        </>
      )}

      {/* Mobile bottom sheet */}
      {!vp.isAtLeastTablet && (
        <BottomSheet open={sheetOpen} onToggle={() => setSheetOpen(o => !o)}>
          <div style={{padding: "8px 18px 18px"}}>
            {/* Robot status mini */}
            <RobotTelemetryCard compact/>
            <div style={{marginTop: 14, display: "flex", flexDirection: "column", gap: 8}}>
              {ZONES.map(z => (
                <ZoneRow
                  key={z.id}
                  zone={z}
                  isSelected={z.id === selectedZone}
                  onClick={() => setSelectedZone(z.id)}
                />
              ))}
            </div>
          </div>
        </BottomSheet>
      )}
    </div>
  );
}

// ─────────────────────────────────────────────────────────────────────
// Sub-components
// ─────────────────────────────────────────────────────────────────────

function MapToolButton({children, onClick}: {children: React.ReactNode; onClick?: () => void}) {
  return (
    <motion.button
      whileTap={{scale: 0.94}}
      whileHover={{y: -1}}
      transition={springSnap}
      onClick={onClick}
      className="glass"
      style={{
        width: 40, height: 40, borderRadius: 12,
        display: "flex", alignItems: "center", justifyContent: "center",
        color: "var(--ink)",
      }}
    >
      {children}
    </motion.button>
  );
}

interface ZoneRowProps {
  zone: Zone;
  isSelected: boolean;
  onClick: () => void;
}

function ZoneRow({zone, isSelected, onClick}: ZoneRowProps) {
  const {t} = useTranslation();
  const accentVar = {
    lime: "var(--lime)",
    cyan: "var(--aurora-cyan)",
    amber: "var(--amber)",
    rose: "var(--rose)",
  }[zone.accent];

  return (
    <motion.button
      onClick={onClick}
      whileHover={{x: 2}}
      whileTap={{scale: 0.98}}
      transition={springSnap}
      style={{
        position: "relative",
        display: "flex", alignItems: "center", gap: 12,
        padding: 12,
        borderRadius: 14,
        background: isSelected ? "rgba(124,255,178,0.07)" : "rgba(255,255,255,0.025)",
        border: `1px solid ${isSelected ? "var(--border-glow)" : "var(--border-soft)"}`,
        textAlign: "left",
        overflow: "hidden",
      }}
    >
      {/* zone accent stripe */}
      <span style={{
        position: "absolute", left: 0, top: 0, bottom: 0, width: 3,
        background: accentVar,
        opacity: isSelected ? 1 : 0.45,
      }}/>
      <div style={{
        width: 34, height: 34, borderRadius: 10,
        background: `linear-gradient(135deg, ${accentVar}33, ${accentVar}0a)`,
        border: `1px solid ${accentVar}44`,
        display: "flex", alignItems: "center", justifyContent: "center",
        color: accentVar,
        flexShrink: 0,
      }}>
        <MapPin size={16} strokeWidth={2.2}/>
      </div>
      <div style={{flex: 1, minWidth: 0}}>
        <div style={{
          display: "flex", alignItems: "baseline", gap: 8,
          fontSize: 14, fontWeight: 600, color: "var(--ink)",
        }}>
          {t(zone.name)}
          {zone.active && (
            <span style={{
              fontSize: 9, color: "var(--lime)",
              letterSpacing: "0.08em", textTransform: "uppercase",
              fontWeight: 700,
            }}>
              {t('gardenMap.zoneMowingTag')}
            </span>
          )}
        </div>
        <div className="mono" style={{
          fontSize: 11, color: "var(--ink-3)", marginTop: 2,
        }}>
          {zone.area} m² · {t('gardenMap.coveredPercent', {percent: Math.round(zone.coverage * 100)})}
        </div>
        {/* progress bar */}
        <div style={{
          marginTop: 6, height: 3, borderRadius: 2,
          background: "rgba(255,255,255,0.06)", overflow: "hidden",
        }}>
          <motion.div
            initial={{width: 0}}
            animate={{width: `${zone.coverage * 100}%`}}
            transition={{duration: 0.6, delay: 0.2, ease: [0.2, 0.7, 0.2, 1]}}
            style={{
              height: "100%",
              background: `linear-gradient(90deg, ${accentVar}, ${accentVar}aa)`,
            }}
          />
        </div>
      </div>
    </motion.button>
  );
}

function RobotTelemetryCard({compact}: {compact?: boolean} = {}) {
  const {t} = useTranslation();
  return (
    <GlassCard padding={16}>
      <div style={{
        display: "flex", alignItems: "center", gap: 10, marginBottom: 12,
      }}>
        <Navigation size={14} color="var(--lime)" strokeWidth={2.4}/>
        <span style={{
          fontSize: 10, color: "var(--ink-2)",
          letterSpacing: "0.08em", textTransform: "uppercase",
          fontWeight: 700,
        }}>
          {t('gardenMap.livePosition')}
        </span>
      </div>

      <div style={{
        display: "grid",
        gridTemplateColumns: compact ? "1fr 1fr 1fr" : "1fr 1fr",
        gap: 10,
      }}>
        <Telemetry label="X" value="+12.4" unit="m"/>
        <Telemetry label="Y" value="-4.7" unit="m"/>
        <Telemetry label={t('gardenMap.telemetryHeading')} value="38" unit="°"/>
        <Telemetry label={t('gardenMap.telemetrySpeed')} value="0.42" unit="m/s"/>
        <Telemetry label="GPS σ" value="3" unit="cm" tone="ok"/>
        <Telemetry label="Strips" value="11" unit="/ 24"/>
      </div>
    </GlassCard>
  );
}

function Telemetry({
  label, value, unit, tone,
}: {label: string; value: string; unit: string; tone?: "ok" | "warn"}) {
  const color = tone === "ok" ? "var(--lime)" : tone === "warn" ? "var(--amber)" : "var(--ink)";
  return (
    <div>
      <div style={{
        fontSize: 9, color: "var(--ink-3)",
        letterSpacing: "0.08em", textTransform: "uppercase",
        fontWeight: 700,
      }}>{label}</div>
      <div style={{display: "flex", alignItems: "baseline", gap: 3, marginTop: 2}}>
        <div className="display" style={{
          fontSize: 22, fontWeight: 700, color,
          letterSpacing: "-0.02em", lineHeight: 1,
        }}>
          {value}
        </div>
        <div className="mono" style={{fontSize: 10, color: "var(--ink-3)"}}>{unit}</div>
      </div>
    </div>
  );
}

function ZoneActionsCard() {
  const {t} = useTranslation();
  return (
    <GlassCard padding={16}>
      <div style={{
        fontSize: 10, color: "var(--ink-2)",
        letterSpacing: "0.08em", textTransform: "uppercase",
        fontWeight: 700, marginBottom: 12,
      }}>
        {t('gardenMap.zoneAction')}
      </div>

      <button style={{
        display: "flex", alignItems: "center", justifyContent: "center", gap: 8,
        width: "100%", padding: "14px 16px",
        background: "var(--grad-primary)",
        color: "var(--bg-deep)",
        borderRadius: 14, fontWeight: 700, fontSize: 14,
        boxShadow: "0 12px 28px -8px rgba(124,255,178,0.5), inset 0 1px 0 rgba(255,255,255,0.35)",
        marginBottom: 8,
      }}>
        <Navigation size={16} strokeWidth={2.4}/>
        {t('gardenMap.mowThisZone')}
      </button>

      <div style={{display: "grid", gridTemplateColumns: "1fr 1fr", gap: 8}}>
        <button style={{
          display: "flex", alignItems: "center", justifyContent: "center", gap: 6,
          padding: "10px 12px",
          background: "rgba(255,255,255,0.04)",
          border: "1px solid var(--border-soft)",
          borderRadius: 12, fontWeight: 600, fontSize: 12,
          color: "var(--ink)",
        }}>
          <Edit3 size={14} strokeWidth={2.2}/>
          {t('gardenMap.edit')}
        </button>
        <button style={{
          display: "flex", alignItems: "center", justifyContent: "center", gap: 6,
          padding: "10px 12px",
          background: "rgba(255,107,122,0.08)",
          border: "1px solid rgba(255,107,122,0.3)",
          borderRadius: 12, fontWeight: 600, fontSize: 12,
          color: "var(--rose)",
        }}>
          <Trash2 size={14} strokeWidth={2.2}/>
          {t('gardenMap.delete')}
        </button>
      </div>
    </GlassCard>
  );
}

// ─────────────────────────────────────────────────────────────────────
// Mobile bottom sheet
// ─────────────────────────────────────────────────────────────────────

interface BottomSheetProps {
  open: boolean;
  onToggle: () => void;
  children: React.ReactNode;
}

function BottomSheet({open, onToggle, children}: BottomSheetProps) {
  const {t} = useTranslation();
  return (
    <>
      <AnimatePresence>
        {open && (
          <motion.div
            initial={{opacity: 0}}
            animate={{opacity: 1}}
            exit={{opacity: 0}}
            onClick={onToggle}
            style={{
              position: "absolute", inset: 0,
              background: "rgba(0,0,0,0.4)",
              backdropFilter: "blur(6px)",
              zIndex: 20,
            }}
          />
        )}
      </AnimatePresence>
      <motion.div
        initial={false}
        animate={{y: open ? 0 : "calc(100% - 92px)"}}
        transition={springSoft}
        drag="y"
        dragConstraints={{top: 0, bottom: 0}}
        dragElastic={0.2}
        onDragEnd={(_, info) => {
          if (info.velocity.y < -200 || info.offset.y < -100) onToggle();
          if (info.velocity.y >  200 || info.offset.y >  100) onToggle();
        }}
        style={{
          position: "absolute", left: 0, right: 0, bottom: 80, /* leave room for bottom-nav */
          maxHeight: "70vh",
          background: "rgba(11,21,17,0.86)",
          backdropFilter: "blur(28px) saturate(140%)",
          borderTop: "1px solid var(--border-soft)",
          borderTopLeftRadius: 28,
          borderTopRightRadius: 28,
          zIndex: 25,
          overflow: "hidden",
          touchAction: "none",
        }}
      >
        {/* handle */}
        <button
          onClick={onToggle}
          style={{
            display: "flex", flexDirection: "column", alignItems: "center", justifyContent: "center",
            gap: 6, width: "100%", padding: "12px 0 8px",
          }}
        >
          <span style={{
            width: 40, height: 4, borderRadius: 2,
            background: "rgba(255,255,255,0.18)",
          }}/>
          <div style={{
            display: "flex", alignItems: "center", gap: 6,
            fontSize: 11, color: "var(--ink-2)",
            letterSpacing: "0.06em", textTransform: "uppercase", fontWeight: 600,
          }}>
            <ChevronUp size={12} strokeWidth={2.6}/>
            {open ? t('gardenMap.collapse') : t('gardenMap.viewZones')}
          </div>
        </button>
        <div style={{overflowY: "auto", maxHeight: "calc(70vh - 60px)"}} className="scrollbar-thin">
          {children}
        </div>
      </motion.div>
    </>
  );
}
