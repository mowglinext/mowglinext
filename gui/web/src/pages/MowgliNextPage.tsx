import {motion} from "framer-motion";
import {Sparkles, ChevronRight, Wifi, Droplets, Thermometer} from "lucide-react";

import {useIsMobile} from "../hooks/useIsMobile";
import {useHighLevelStatus} from "../hooks/useHighLevelStatus.ts";
import {usePower} from "../hooks/usePower.ts";
import {useStatus} from "../hooks/useStatus.ts";
import {useGnssStatus} from "../hooks/useGnssStatus.ts";
import {useEmergency} from "../hooks/useEmergency.ts";
import {useSettings} from "../hooks/useSettings.ts";
import {useDiagnosticsSnapshot} from "../hooks/useDiagnosticsSnapshot.ts";
import {useMowingMap} from "../hooks/useMowingMap.ts";
import {useFusionOdom} from "../hooks/useFusionOdom.ts";
import {useMowerAction} from "../components/MowerActions.tsx";
import {computeBatteryPercent} from "../utils/battery.ts";
import {deriveGpsStatus} from "../utils/gpsStatus.ts";

import {GlassCard} from "../concept/components/GlassCard.tsx";
import {BatteryRing} from "../concept/components/BatteryRing.tsx";
import {StatusOrb} from "../concept/components/StatusOrb.tsx";
import {ActionCluster} from "../concept/components/ActionCluster.tsx";
import {LiveMapMini} from "../concept/components/LiveMapMini.tsx";
import {ProgressRibbon} from "../concept/components/ProgressRibbon.tsx";
import {WeatherChip} from "../concept/components/WeatherChip.tsx";
import {NoiseTexture} from "../concept/components/NoiseTexture.tsx";
import {staggerParent, riseFade, popIn, springSnap} from "../concept/motion.ts";

/**
 * Real-data Dashboard, rebuilt on top of the /concept components.
 *
 * Mobile -> single column; Desktop -> 1.2fr / 1fr layout (hero left,
 * live map + stats right). The page itself sits inside AppShell which
 * provides the side-rail + header chrome.
 */

const MOTION_STATES = new Set([
  "MOWING", "TRANSIT", "UNDOCKING", "RETURNING_HOME", "MANUAL_MOWING",
  "RESUMING_AFTER_RAIN", "RESUMING_UNDOCKING", "BOUNDARY_RECOVERY",
  "LOW_BATTERY_DOCKING", "CRITICAL_BATTERY_DOCKING",
  "COVERAGE_FAILED_DOCKING", "SKIP_STRIP", "PREFLIGHT_CHECK",
  "CALIBRATING_HEADING", "RECORDING",
]);

function useMowerData() {
  const {highLevelStatus} = useHighLevelStatus();
  const power = usePower();
  const status = useStatus();
  const gnss = useGnssStatus();
  const emergency = useEmergency();
  const {settings} = useSettings();

  const isCharging = highLevelStatus.is_charging ?? status.is_charging ?? false;
  const isEmergency = highLevelStatus.emergency ?? emergency.active_emergency ?? false;
  const batteryPercent = computeBatteryPercent(
    highLevelStatus.battery_percent, power.v_battery, settings,
  );
  const gpsStatus = deriveGpsStatus(gnss);

  const stateName = highLevelStatus.state_name ?? (
    isEmergency ? "EMERGENCY" : isCharging ? "CHARGING" : "IDLE"
  );

  const areaPct = (() => {
    if (highLevelStatus.current_path != null && highLevelStatus.current_path > 0 &&
        highLevelStatus.current_path_index != null) {
      return (highLevelStatus.current_path_index / highLevelStatus.current_path) * 100;
    }
    return 0;
  })();

  const isMoving = stateName ? MOTION_STATES.has(stateName) : false;

  return {
    state: stateName,
    battery: batteryPercent,
    charging: isCharging,
    emergency: isEmergency,
    gps: gpsStatus.percent,
    gpsLabel: gpsStatus.label,
    vBattery: power.v_battery ?? 0,
    current: power.charge_current ?? 0,
    rpm: status.mower_motor_rpm ?? 0,
    escTemp: status.mower_esc_temperature ?? 0,
    motorTemp: status.mower_motor_temperature ?? 0,
    rain: status.rain_detected ?? false,
    areaPct,
    isMoving,
    currentArea: highLevelStatus.current_area != null
      ? `Area ${highLevelStatus.current_area + 1}`
      : undefined,
  };
}

export const MowgliNextPage = () => {
  const isMobile = useIsMobile();
  const mowerAction = useMowerAction();
  const data = useMowerData();
  const {settings} = useSettings();
  const {snapshot} = useDiagnosticsSnapshot();
  const map = useMowingMap();
  const odom = useFusionOdom();

  const coverage = snapshot?.coverage ?? [];
  const activeArea = coverage.find(c => c.area_index === 0);
  const todayMowedM2 = activeArea ? activeArea.mowed_cells : 0;
  const totalArea = activeArea ? activeArea.total_cells : 0;
  const coveragePct = totalArea > 0 ? todayMowedM2 / totalArea : 0;

  // Pose normalised to map bbox -- LiveMapMini handles default polygons; we
  // pass an approximated robot dot when we have data.
  const polygonAreas = map.working_area ?? [];
  const polyPoints = polygonAreas[0]?.area?.points ?? [];
  const polygonNormalised = polyPoints.length >= 3
    ? polyPoints.map(p => ({x: p.x ?? 0, y: p.y ?? 0}))
    : undefined;

  const pose = odom?.pose?.pose?.position;
  const ori = odom?.pose?.pose?.orientation;
  const robotYawDeg = ori
    ? (Math.atan2(2 * (ori.w * ori.z + ori.x * ori.y),
                  1 - 2 * (ori.y * ori.y + ori.z * ori.z)) * 180) / Math.PI
    : 0;
  // normalise within bbox
  const bbox = (() => {
    if (!polygonNormalised) return null;
    let x0 = Infinity, y0 = Infinity, x1 = -Infinity, y1 = -Infinity;
    polygonNormalised.forEach(p => {
      if (p.x < x0) x0 = p.x; if (p.y < y0) y0 = p.y;
      if (p.x > x1) x1 = p.x; if (p.y > y1) y1 = p.y;
    });
    return {x0: x0 - 1, y0: y0 - 1, x1: x1 + 1, y1: y1 + 1};
  })();
  const robotNormalised = (pose && bbox)
    ? {
        x: (pose.x - bbox.x0) / (bbox.x1 - bbox.x0),
        y: 1 - (pose.y - bbox.y0) / (bbox.y1 - bbox.y0),
        heading: robotYawDeg,
      }
    : undefined;

  const phase: "idle" | "playing" | "returning" | "alert" =
    data.emergency ? "alert" :
    data.state === "RETURNING_HOME" ? "returning" :
    data.isMoving ? "playing" : "idle";

  // ── ETA estimate ──
  //
  // remaining_cells × cell_size² = remaining m². Mowing rate = tool_width
  // × forward_speed [m²/s]. Use live linear velocity when available, else
  // fall back to the nominal cruise speed.
  const cellResolutionM = 0.05;            // map_server publishes the grid at 5 cm
  const remainingCells = Math.max(0, totalArea - todayMowedM2);
  const remainingM2 = remainingCells * cellResolutionM * cellResolutionM;
  const toolWidthM = (settings?.tool_width as number | undefined) ?? 0.18;
  const liveVel = Math.abs(odom?.twist?.twist?.linear?.x ?? 0);
  const nominalSpeed = 0.35;               // typical OpenMower cruise
  const speedMs = liveVel > 0.05 ? liveVel : nominalSpeed;
  const rateM2PerSec = toolWidthM * speedMs;
  const remainingMin = data.isMoving && rateM2PerSec > 0 && remainingCells > 0
    ? Math.max(1, Math.round(remainingM2 / rateM2PerSec / 60))
    : 0;

  const headline = data.isMoving && remainingMin > 0
    ? <>Encore <span style={{
        background: 'var(--grad-primary, linear-gradient(135deg, #7CFFB2, #2BAA66))',
        WebkitBackgroundClip: 'text', backgroundClip: 'text',
        WebkitTextFillColor: 'transparent', color: 'transparent',
      }}>{remainingMin} min</span> avant la maison.</>
    : data.state === "CHARGING"
      ? <>En charge · <span style={{
          background: 'var(--grad-primary, linear-gradient(135deg, #7CFFB2, #2BAA66))',
          WebkitBackgroundClip: 'text', backgroundClip: 'text',
          WebkitTextFillColor: 'transparent', color: 'transparent',
        }}>{Math.round(data.battery)} %</span></>
      : data.emergency
        ? <span style={{color: 'var(--rose, #FF6B7A)'}}>Arrêt d'urgence</span>
        : <>Mowgli est <em style={{fontStyle: 'italic', color: 'var(--lime, #7CFFB2)'}}>au repos</em>.</>;

  const subline = data.isMoving
    ? `Avance à ${data.gpsLabel.toLowerCase()} · ${data.currentArea ?? "zone active"}`
    : data.charging
      ? `Sur la dock, ${data.current.toFixed(1)} A en entrée.`
      : data.emergency
        ? "Sécurise la zone puis relance avec le bouton vert."
        : "Tape Play pour démarrer une tonte ou laisse le planning gérer.";

  const actions = {
    onStart: mowerAction("high_level_control", {Command: 1}),
    onPause: mowerAction("mower_logic", {Config: {Bools: [{Name: "manual_pause_mowing", Value: true}]}}),
    onHome: mowerAction("high_level_control", {Command: 2}),
    onStop: mowerAction("emergency", {Emergency: 1}),
  };

  return (
    <div style={{position: 'relative', minHeight: '100%'}}>
      <NoiseTexture/>
      <motion.div
        variants={staggerParent(0.06, 0.06)}
        initial="hidden" animate="show"
        style={{
          position: 'relative', zIndex: 1,
          maxWidth: isMobile ? 560 : 1280, margin: '0 auto',
        }}
      >
        {/* greeting strip */}
        <motion.header variants={riseFade} style={{
          display: 'flex', alignItems: 'baseline', justifyContent: 'space-between',
          marginBottom: isMobile ? 18 : 24,
        }}>
          <div>
            <div style={{
              fontSize: 11, color: 'rgba(236,255,244,0.42)',
              letterSpacing: '0.06em', textTransform: 'uppercase', fontWeight: 600,
            }}>
              {greetingFor()}
            </div>
            <div className="mn-display" style={{
              fontSize: isMobile ? 26 : 34,
              color: 'var(--text, #ECFFF4)', fontWeight: 400,
              letterSpacing: '-0.02em', lineHeight: 1.05, marginTop: 4,
            }}>
              {data.isMoving ? "Mowgli est en tonte" : data.charging ? "Mowgli charge" : "Bon retour"}
            </div>
          </div>
          <StatusOrb
            tone={data.emergency ? "alert" : data.isMoving ? "live" : data.charging ? "charging" : "resting"}
            size={10}
            label={data.isMoving ? "En tonte" : data.charging ? "En charge" : data.emergency ? "Alerte" : "Au repos"}
          />
        </motion.header>

        {/* layout */}
        {isMobile ? (
          <div style={{display: 'flex', flexDirection: 'column', gap: 14}}>
            <motion.div variants={popIn}>
              <HeroCard
                data={data} phase={phase} actions={actions}
                headline={headline} subline={subline}
                coveragePct={coveragePct}
                todayMowedM2={todayMowedM2} totalArea={totalArea}
              />
            </motion.div>
            <motion.div variants={riseFade}><LiveMapCard polygon={polygonNormalised} robot={robotNormalised} coverage={coveragePct}/></motion.div>
            <motion.div variants={riseFade}><TilesRow data={data}/></motion.div>
            <motion.div variants={riseFade}><HealthCard data={data}/></motion.div>
          </div>
        ) : (
          <div style={{display: 'grid', gridTemplateColumns: '1.2fr 1fr', gap: 22, alignItems: 'start'}}>
            <div style={{display: 'flex', flexDirection: 'column', gap: 18}}>
              <motion.div variants={popIn}>
                <HeroCard
                  data={data} phase={phase} actions={actions}
                  headline={headline} subline={subline}
                  coveragePct={coveragePct}
                  todayMowedM2={todayMowedM2} totalArea={totalArea}
                  large
                />
              </motion.div>
              <motion.div variants={riseFade}><HealthCard data={data}/></motion.div>
            </div>
            <div style={{display: 'flex', flexDirection: 'column', gap: 18}}>
              <motion.div variants={riseFade}><LiveMapCard polygon={polygonNormalised} robot={robotNormalised} coverage={coveragePct} height={300}/></motion.div>
              <motion.div variants={riseFade}><TilesRow data={data}/></motion.div>
            </div>
          </div>
        )}
      </motion.div>
    </div>
  );
};

// ─────────────────────────────────────────────────────────────────────
// Composed sub-cards
// ─────────────────────────────────────────────────────────────────────

interface HeroCardProps {
  data: ReturnType<typeof useMowerData>;
  phase: "idle" | "playing" | "returning" | "alert";
  actions: {
    onStart: () => void; onPause: () => void;
    onHome: () => void; onStop: () => void;
  };
  headline: React.ReactNode;
  subline: string;
  coveragePct: number;
  todayMowedM2: number;
  totalArea: number;
  large?: boolean;
}

function HeroCard({
  data, phase, actions, headline, subline, coveragePct, todayMowedM2, totalArea, large,
}: HeroCardProps) {
  return (
    <GlassCard variant="glow" padding={0}>
      <div style={{
        position: 'relative',
        padding: large ? '30px 28px 26px' : '24px 22px 22px',
        background:
          "radial-gradient(circle at 80% -20%, rgba(124,255,178,0.18) 0%, transparent 55%)," +
          "radial-gradient(circle at -20% 110%, rgba(69,214,232,0.16) 0%, transparent 50%)",
      }}>
        <div style={{display: 'grid', gridTemplateColumns: '1fr auto', gap: large ? 24 : 18, alignItems: 'center'}}>
          <div style={{minWidth: 0}}>
            <div style={{
              fontSize: 11, color: 'var(--lime, #7CFFB2)', fontWeight: 700,
              letterSpacing: '0.12em', textTransform: 'uppercase',
            }}>
              {data.currentArea ? `Mowgli · ${data.currentArea}` : 'Mowgli · sans zone'}
            </div>
            <h1 className="mn-display" style={{
              fontSize: large ? 38 : 30,
              lineHeight: 1.05, marginTop: 8,
              fontWeight: 400, letterSpacing: '-0.025em',
              color: 'var(--text, #ECFFF4)',
            }}>
              {headline}
            </h1>
            <p style={{
              fontSize: large ? 14 : 13, color: 'rgba(236,255,244,0.66)',
              marginTop: 10, lineHeight: 1.55, maxWidth: 460,
            }}>
              {subline}
            </p>
          </div>
          <BatteryRing
            percent={data.battery}
            size={large ? 156 : 124}
            thickness={large ? 11 : 9}
            charging={data.charging}
          >
            <div className="mn-num" style={{
              fontSize: large ? 38 : 30, fontWeight: 400, lineHeight: 1,
              color: 'var(--text, #ECFFF4)', letterSpacing: '-0.02em',
            }}>
              {Math.round(data.battery)}
            </div>
            <div style={{
              fontSize: 10, color: 'rgba(236,255,244,0.42)',
              letterSpacing: '0.1em', textTransform: 'uppercase',
              fontWeight: 600, marginTop: 2,
            }}>
              batterie
            </div>
          </BatteryRing>
        </div>

        {totalArea > 0 && (
          <div style={{marginTop: large ? 22 : 18}}>
            <div style={{
              display: 'flex', alignItems: 'baseline', justifyContent: 'space-between',
              marginBottom: 8,
            }}>
              <span style={{
                fontSize: 11, color: 'rgba(236,255,244,0.42)',
                letterSpacing: '0.08em', textTransform: 'uppercase', fontWeight: 600,
              }}>
                Couverture aujourd'hui
              </span>
              <span style={{fontSize: 12, color: 'rgba(236,255,244,0.66)', fontWeight: 600, fontFamily: '"Space Grotesk", monospace'}}>
                {todayMowedM2.toLocaleString()}<span style={{color: 'rgba(236,255,244,0.42)'}}> / {totalArea.toLocaleString()} cellules</span>
              </span>
            </div>
            <ProgressRibbon value={coveragePct} segments={24}/>
          </div>
        )}

        <div style={{marginTop: large ? 28 : 22}}>
          <ActionCluster phase={phase} {...actions}/>
        </div>
      </div>
    </GlassCard>
  );
}

interface LiveMapCardProps {
  polygon?: {x: number; y: number}[];
  robot?: {x: number; y: number; heading: number};
  coverage: number;
  height?: number;
}

function LiveMapCard({polygon, robot, coverage, height = 220}: LiveMapCardProps) {
  // LiveMapMini expects 0..1 normalised; we pass an empty/default if no
  // recorded area yet.
  return (
    <GlassCard padding={0} style={{overflow: 'hidden'}}>
      <div style={{
        display: 'flex', alignItems: 'baseline', justifyContent: 'space-between',
        padding: '16px 22px 8px',
      }}>
        <div>
          <div style={{
            fontSize: 11, color: 'rgba(236,255,244,0.42)',
            letterSpacing: '0.08em', textTransform: 'uppercase', fontWeight: 600,
          }}>
            Carte vivante
          </div>
          <div className="mn-display" style={{
            fontSize: 18, fontWeight: 400, marginTop: 2,
            color: 'var(--text, #ECFFF4)', letterSpacing: '-0.01em',
          }}>
            Trajectoire en direct
          </div>
        </div>
        <button style={{
          display: 'inline-flex', alignItems: 'center', gap: 4,
          background: 'transparent', border: 'none',
          fontSize: 12, fontWeight: 600, color: 'var(--lime, #7CFFB2)', cursor: 'pointer',
        }}>
          Voir la carte
          <ChevronRight size={14} strokeWidth={2.4}/>
        </button>
      </div>
      <LiveMapMini
        polygon={polygon && polygon.length > 0 ? normaliseToUnit(polygon) : undefined}
        robot={robot}
        coverage={coverage}
        height={height}
      />
    </GlassCard>
  );
}

function TilesRow({data}: {data: ReturnType<typeof useMowerData>}) {
  return (
    <div style={{display: 'grid', gridTemplateColumns: 'repeat(2, 1fr)', gap: 10}}>
      <StatTile label="GPS" value={`${Math.round(data.gps)}`} unit="%"
                hint={data.gpsLabel} accent="cyan" icon={<Wifi size={14}/>}/>
      <StatTile label="Lames" value={data.rpm > 0 ? Math.round(data.rpm).toString() : 'off'}
                unit={data.rpm > 0 ? 'rpm' : ''} hint={`${data.current.toFixed(1)} A`}
                accent="amber" icon={<Sparkles size={14}/>}/>
      <StatTile label="Moteur" value={data.motorTemp.toFixed(0)} unit="°c"
                hint={`ESC ${data.escTemp.toFixed(0)} °C`}
                accent={data.motorTemp > 55 ? "amber" : "lime"}
                icon={<Thermometer size={14}/>}/>
      <StatTile label="Pluie" value={data.rain ? 'détectée' : 'au sec'} unit=""
                hint={data.rain ? 'tonte mise en pause' : 'bonnes conditions'}
                accent={data.rain ? "amber" : "lime"}
                icon={<Droplets size={14}/>}/>
    </div>
  );
}

interface StatTileProps {
  label: string;
  value: string;
  unit: string;
  hint: string;
  accent: "lime" | "cyan" | "amber" | "rose";
  icon: React.ReactNode;
}

function StatTile({label, value, unit, hint, accent, icon}: StatTileProps) {
  const accentColor =
    accent === "lime"  ? "var(--lime, #7CFFB2)" :
    accent === "cyan"  ? "var(--aurora-cyan, #45D6E8)" :
    accent === "amber" ? "var(--amber, #F3A85C)" :
    "var(--rose, #FF6B7A)";
  return (
    <motion.div whileHover={{y: -2}} whileTap={{scale: 0.97}} transition={springSnap}>
      <GlassCard padding={14} style={{position: 'relative', overflow: 'hidden'}}>
        <span aria-hidden style={{
          position: 'absolute', top: -18, right: -18,
          width: 72, height: 72, borderRadius: 72,
          background: `radial-gradient(circle, ${accentColor}, transparent 70%)`,
          opacity: 0.25, pointerEvents: 'none',
        }}/>
        <div style={{
          display: 'flex', alignItems: 'center', gap: 8,
          fontSize: 10, fontWeight: 600,
          color: 'rgba(236,255,244,0.66)', letterSpacing: '0.08em', textTransform: 'uppercase',
        }}>
          <span style={{color: accentColor}}>{icon}</span>
          {label}
        </div>
        <div style={{display: 'flex', alignItems: 'baseline', gap: 4, marginTop: 8}}>
          <div className="mn-num" style={{
            fontSize: 30, color: 'var(--text, #ECFFF4)', lineHeight: 1,
          }}>
            {value}
          </div>
          {unit && (
            <div style={{
              fontSize: 12, color: 'rgba(236,255,244,0.42)', fontWeight: 600,
              fontFamily: '"Space Grotesk", monospace', textTransform: 'lowercase',
              letterSpacing: '0.04em',
            }}>{unit}</div>
          )}
        </div>
        <div style={{fontSize: 10, color: 'rgba(236,255,244,0.42)', marginTop: 6, lineHeight: 1.3}}>
          {hint}
        </div>
      </GlassCard>
    </motion.div>
  );
}

function HealthCard({data}: {data: ReturnType<typeof useMowerData>}) {
  const rows = [
    {k: 'Signal GPS',         ok: data.gps > 0,           note: data.gpsLabel},
    {k: data.rain ? 'Pluie détectée' : 'Pas de pluie',
                              ok: !data.rain,             note: data.rain ? 'tonte en pause' : 'conditions OK'},
    {k: data.emergency ? 'Alerte active' : 'Aucune alerte',
                              ok: !data.emergency,        note: data.emergency ? 'à réarmer' : 'tout clair'},
    {k: `Moteur ${data.motorTemp.toFixed(0)} °C`,
                              ok: data.motorTemp < 55,    note: data.motorTemp >= 55 ? 'tourne chaud' : 'nominal'},
  ];
  return (
    <GlassCard padding={20}>
      <div style={{
        display: 'flex', alignItems: 'center', justifyContent: 'space-between',
        marginBottom: 14,
      }}>
        <div style={{
          fontSize: 11, color: 'rgba(236,255,244,0.42)',
          letterSpacing: '0.08em', textTransform: 'uppercase', fontWeight: 600,
        }}>
          Bilan santé
        </div>
        <WeatherChip condition="partly" tempC={22}/>
      </div>
      <div style={{display: 'flex', flexDirection: 'column', gap: 10}}>
        {rows.map(r => (
          <div key={r.k} style={{display: 'flex', alignItems: 'center', gap: 12}}>
            <span style={{
              width: 8, height: 8, borderRadius: 4,
              background: r.ok ? 'var(--lime, #7CFFB2)' : 'var(--rose, #FF6B7A)',
              boxShadow: r.ok ? '0 0 8px rgba(124,255,178,0.4)' : '0 0 8px rgba(255,107,122,0.4)',
              flexShrink: 0,
            }}/>
            <div style={{flex: 1, minWidth: 0}}>
              <div style={{fontSize: 13, fontWeight: 600, color: 'var(--text, #ECFFF4)'}}>{r.k}</div>
              <div style={{fontSize: 11, color: 'rgba(236,255,244,0.42)'}}>{r.note}</div>
            </div>
          </div>
        ))}
      </div>
    </GlassCard>
  );
}

// ─────────────────────────────────────────────────────────────────────
// Utilities
// ─────────────────────────────────────────────────────────────────────

function greetingFor() {
  const h = new Date().getHours();
  if (h < 6)  return 'Bonsoir';
  if (h < 12) return 'Bonjour';
  if (h < 18) return 'Bel après-midi';
  return 'Bonsoir';
}

function normaliseToUnit(pts: {x: number; y: number}[]): {x: number; y: number}[] {
  let x0 = Infinity, y0 = Infinity, x1 = -Infinity, y1 = -Infinity;
  pts.forEach(p => {
    if (p.x < x0) x0 = p.x; if (p.y < y0) y0 = p.y;
    if (p.x > x1) x1 = p.x; if (p.y > y1) y1 = p.y;
  });
  const dx = (x1 - x0) || 1;
  const dy = (y1 - y0) || 1;
  return pts.map(p => ({
    x: (p.x - x0) / dx * 0.8 + 0.1,
    y: 1 - ((p.y - y0) / dy * 0.8 + 0.1),
  }));
}

export default MowgliNextPage;
