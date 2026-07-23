import type {ReactNode} from "react";
import {useMemo} from "react";
import {App, Button} from "antd";
import {motion} from "framer-motion";
import {useNavigate} from "react-router-dom";
import {useTranslation} from "react-i18next";
import type {TFunction} from "i18next";
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
import {useMowProgress} from "../hooks/useMowProgress.ts";
import {useFusionOdom} from "../hooks/useFusionOdom.ts";
import {rasterizeMowProgress} from "../utils/mowProgress.ts";
import {useMowerAction} from "../components/MowerActions.tsx";
import {computeBatteryPercent} from "../utils/battery.ts";
import {deriveGpsStatus} from "../utils/gpsStatus.ts";

import {GlassCard} from "../concept/components/GlassCard.tsx";
import {BatteryRing} from "../concept/components/BatteryRing.tsx";
import {StatusOrb} from "../concept/components/StatusOrb.tsx";
import {ActionCluster} from "../concept/components/ActionCluster.tsx";
import {LiveMapMini} from "../concept/components/LiveMapMini.tsx";
import type {MiniArea, MiniProgress} from "../concept/components/LiveMapMini.tsx";
import {ProgressRibbon} from "../concept/components/ProgressRibbon.tsx";
import {WeatherChip} from "../concept/components/WeatherChip.tsx";
import {useWeather} from "../hooks/useWeather.ts";
import {NoiseTexture} from "../concept/components/NoiseTexture.tsx";
import {staggerParent, riseFade, popIn, springSnap} from "../concept/motion.ts";

/**
 * Real-data Dashboard, rebuilt on top of the /concept components.
 *
 * Mobile -> single column; Desktop -> 1.2fr / 1fr layout (hero left,
 * live map + stats right). The page itself sits inside AppShell which
 * provides the side-rail + header chrome.
 */

function useMowerData() {
  const {t} = useTranslation();
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
    isEmergency ? "EMERGENCY" : isCharging ? "CHARGING" : "IDLE_DOCKED"
  );

  // Motion is derived from the NUMERIC high-level state, which status_nodes.cpp
  // publishes reliably every tick (2=AUTONOMOUS, 3=RECORDING, 4=MANUAL_MOWING).
  // The old string allowlist omitted state=2 substates (PLANNING,
  // OBSTACLE_BACKOFF, DYNAMIC_OBSTACLE_CLEARED, AREA_UNREACHABLE) and carried a
  // phantom SKIP_STRIP, so every planning/backoff phase read as "idle" mid-mow.
  const stateNum = highLevelStatus.state ?? -1;
  const isMoving = stateNum === 2 || stateNum === 3 || stateNum === 4;

  return {
    state: stateName,
    battery: batteryPercent,
    charging: isCharging,
    emergency: isEmergency,
    gps: gpsStatus.percent,
    gpsLabel: gpsStatus.label,
    vBattery: power.v_battery ?? 0,
    // Battery charge current (shown while docked/charging) vs. blade motor
    // current (shown on the Blades tile) are DIFFERENT signals — keep them
    // separate so the Blades tile doesn't read the charger.
    current: power.charge_current ?? 0,
    bladeCurrent: status.mower_esc_current ?? 0,
    rpm: status.mower_motor_rpm ?? 0,
    escTemp: status.mower_esc_temperature ?? 0,
    motorTemp: status.mower_motor_temperature ?? 0,
    rain: status.rain_detected ?? false,
    isMoving,
    toolWidth: (settings?.tool_width as number | undefined) ?? 0.18,
    currentAreaIndex: highLevelStatus.current_area ?? null,
    currentArea: highLevelStatus.current_area != null
      ? t('mowgliNextPage.areaN', {number: highLevelStatus.current_area + 1})
      : undefined,
    // Firmware <-> image compatibility (from the hardware_bridge handshake).
    // null until the first Status arrives, so the health card stays quiet
    // rather than flashing a false "incompatible" on load.
    firmwareCompatible: status.firmware_compatible ?? null,
    firmwareVersion: status.firmware_version ?? "",
  };
}

export const MowgliNextPage = () => {
  const {t} = useTranslation();
  const isMobile = useIsMobile();
  const navigate = useNavigate();
  const {modal, notification} = App.useApp();
  const mowerAction = useMowerAction();
  const data = useMowerData();
  const {snapshot} = useDiagnosticsSnapshot();
  const map = useMowingMap();
  const odom = useFusionOdom();

  const coverage = snapshot?.coverage ?? [];
  // Show the area the robot is ACTUALLY mowing, not always area 0 — otherwise
  // the progress bar / ETA report area 0's grid while it mows area 2.
  const activeArea = coverage.find(c => c.area_index === (data.currentAreaIndex ?? 0))
    ?? coverage.find(c => c.area_index === 0);
  const todayMowedM2 = activeArea ? activeArea.mowed_cells : 0;
  const totalArea = activeArea ? activeArea.total_cells : 0;
  const coveragePct = totalArea > 0 ? todayMowedM2 / totalArea : 0;

  // ── Normalised garden view (ALL areas + robot + mow progress) ──
  //
  // Build ONE bbox over every recorded area's outer ring, then map areas,
  // holes, the robot and the mow-progress grid through the SAME transform so
  // they overlay coherently. Was previously working_area[0] only, which made a
  // multi-area field render just the first zone on the dashboard.
  const workingAreas = map.working_area ?? [];
  const validAreas = workingAreas.filter(a => (a.area?.points?.length ?? 0) >= 3);
  const bbox = (() => {
    let x0 = Infinity, y0 = Infinity, x1 = -Infinity, y1 = -Infinity;
    validAreas.forEach(a => (a.area?.points ?? []).forEach(p => {
      const x = p.x ?? 0, y = p.y ?? 0;
      if (x < x0) x0 = x;
      if (y < y0) y0 = y;
      if (x > x1) x1 = x;
      if (y > y1) y1 = y;
    }));
    if (!isFinite(x0)) return null;
    return {x0, y0, dx: (x1 - x0) || 1, dy: (y1 - y0) || 1};
  })();
  // Map frame -> unit square with a 10 % inset (matches the old look). Non-
  // uniform per-axis scale — the robot dot, area rings and mow-progress raster
  // all ride the same distortion, so they stay aligned.
  const norm = (x: number, y: number): {x: number; y: number} => bbox
    ? {x: (x - bbox.x0) / bbox.dx * 0.8 + 0.1, y: 1 - ((y - bbox.y0) / bbox.dy * 0.8 + 0.1)}
    : {x: 0.5, y: 0.5};

  const polygons: MiniArea[] = bbox ? validAreas.map(a => ({
    outer: (a.area?.points ?? []).map(p => norm(p.x ?? 0, p.y ?? 0)),
    holes: (a.obstacles ?? [])
      .filter(h => (h.points?.length ?? 0) >= 3)
      .map(h => (h.points ?? []).map(p => norm(p.x ?? 0, p.y ?? 0))),
  })) : [];

  const pose = odom?.pose?.pose?.position;
  const ori = odom?.pose?.pose?.orientation;
  const robotYawDeg = ori
    ? (Math.atan2(2 * (ori.w * ori.z + ori.x * ori.y),
                  1 - 2 * (ori.y * ori.y + ori.z * ori.z)) * 180) / Math.PI
    : 0;
  const robotNormalised = (pose && bbox)
    ? {...norm(pose.x ?? 0, pose.y ?? 0), heading: robotYawDeg}
    : undefined;

  // Real mowed-cell overlay: same OccupancyGrid MapPage renders, rasterised
  // once per grid message (throttled to ~1 Hz) and placed in the same unit
  // space. The raster (heavy) is memoised on grid identity; the unit-rect math
  // (cheap) re-runs when the bbox moves.
  const mowGrid = useMowProgress();
  const raster = useMemo(() => rasterizeMowProgress(mowGrid), [mowGrid]);
  const progress: MiniProgress | null = (raster && bbox)
    ? (() => {
        const gW = raster.width * raster.resolution;
        const gH = raster.height * raster.resolution;
        const left = norm(raster.originX, 0).x;
        const right = norm(raster.originX + gW, 0).x;
        const top = norm(0, raster.originY + gH).y;   // max map-y -> smaller unit-y
        const bottom = norm(0, raster.originY).y;
        return {url: raster.dataUrl, x: left, y: top, w: right - left, h: bottom - top};
      })()
    : null;

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
  const toolWidthM = data.toolWidth;
  const liveVel = Math.abs(odom?.twist?.twist?.linear?.x ?? 0);
  const nominalSpeed = 0.35;               // typical OpenMower cruise
  const speedMs = liveVel > 0.05 ? liveVel : nominalSpeed;
  const rateM2PerSec = toolWidthM * speedMs;
  const remainingMin = data.isMoving && rateM2PerSec > 0 && remainingCells > 0
    ? Math.max(1, Math.round(remainingM2 / rateM2PerSec / 60))
    : 0;

  // Headline mirrors the subline's branch ORDER (moving first) so the two never
  // contradict. When moving WITH a usable ETA we show the countdown; when moving
  // but the coverage snapshot hasn't arrived yet (remainingMin === 0) we still
  // say "en tonte" instead of falling through to the idle "au repos" text.
  const headline = data.isMoving
    ? (remainingMin > 0
        ? <>{t('mowgliNextPage.headlineUntilHomePrefix')}<span style={{
            background: 'var(--grad-primary, linear-gradient(135deg, #7CFFB2, #2BAA66))',
            WebkitBackgroundClip: 'text', backgroundClip: 'text',
            WebkitTextFillColor: 'transparent', color: 'transparent',
          }}>{t('mowgliNextPage.headlineMinutes', {value: remainingMin})}</span>{t('mowgliNextPage.headlineUntilHomeSuffix')}</>
        : <>{t('mowgliNextPage.headlineMowingPrefix')}<em style={{fontStyle: 'italic', color: 'var(--lime, #7CFFB2)'}}>{t('mowgliNextPage.headlineMowingEmphasis')}</em>{t('mowgliNextPage.headlineMowingSuffix')}</>)
    : data.state === "CHARGING"
      ? <>{t('mowgliNextPage.headlineChargingPrefix')}<span style={{
          background: 'var(--grad-primary, linear-gradient(135deg, #7CFFB2, #2BAA66))',
          WebkitBackgroundClip: 'text', backgroundClip: 'text',
          WebkitTextFillColor: 'transparent', color: 'transparent',
        }}>{t('mowgliNextPage.headlinePercent', {value: Math.round(data.battery)})}</span></>
      : data.emergency
        ? <span style={{color: 'var(--rose, #FF6B7A)'}}>{t('mowgliNextPage.emergencyStop')}</span>
        : <>{t('mowgliNextPage.headlineIdlePrefix')}<em style={{fontStyle: 'italic', color: 'var(--lime, #7CFFB2)'}}>{t('mowgliNextPage.headlineIdleEmphasis')}</em>{t('mowgliNextPage.headlineIdleSuffix')}</>;

  const subline = data.isMoving
    ? t('mowgliNextPage.sublineMoving', {gps: data.gpsLabel.toLowerCase(), area: data.currentArea ?? t('mowgliNextPage.activeZone')})
    : data.charging
      ? t('mowgliNextPage.sublineCharging', {current: data.current.toFixed(1)})
      : data.emergency
        ? t('mowgliNextPage.sublineEmergency')
        : t('mowgliNextPage.sublineIdle');

  // The hero "stop" affordance latches the firmware EMERGENCY — this is the
  // real e-stop, not a soft pause. Gate it behind an explicit confirm so a
  // mistap can't latch the safety system. The dispatched command is unchanged.
  const fireEmergency = mowerAction("emergency", {Emergency: 1});
  const confirmEmergency = () => {
    modal.confirm({
      title: t('mowgliNextPage.emergencyStop'),
      content: t('mowgliNextPage.emergencyConfirmBody'),
      okText: t('mowgliNextPage.emergencyStop'),
      okType: "danger",
      cancelText: t('mowgliNextPage.cancel'),
      onOk: () => fireEmergency(),
    });
  };

  // mowerAction() returns a fn that THROWS on service error. ActionCluster
  // invokes these from a plain onClick, so a rejection would be swallowed with
  // zero feedback — a robot that "won't start" would look like a hardware
  // fault. Wrap each so failures surface a notification.
  const withFeedback = (fn: () => Promise<unknown>) => async () => {
    try {
      await fn();
    } catch (e: unknown) {
      notification.error({
        message: t('mowgliNextPage.actionFailed'),
        description: e instanceof Error ? e.message : undefined,
      });
    }
  };

  const actions = {
    onStart: withFeedback(mowerAction("high_level_control", {Command: 1})),
    // Pause = STOP (COMMAND_STOP=8 → StopHoldSequence: mower off, halt in place,
    // Nav2 left up so the mission can resume via START, no dock drive). The
    // separate Home control keeps HOME (Command 2 → return to dock).
    onPause: withFeedback(mowerAction("high_level_control", {Command: 8})),
    onHome: withFeedback(mowerAction("high_level_control", {Command: 2})),
    onStop: confirmEmergency,
    // Clear a latched emergency from the Dashboard. Without this the robot is
    // stuck: the BT EmergencyGuard halts before MainLogic, so Play (Command 1)
    // is inert while latched. Firmware is the safety authority — it only clears
    // the latch if the physical trigger is no longer asserted.
    onRearm: withFeedback(mowerAction("emergency", {Emergency: 0})),
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
              {greetingFor(t)}
            </div>
            <div className="mn-display" style={{
              fontSize: isMobile ? 26 : 34,
              color: 'var(--ink, #ECFFF4)', fontWeight: 400,
              letterSpacing: '-0.02em', lineHeight: 1.05, marginTop: 4,
            }}>
              {data.isMoving ? t('mowgliNextPage.mowgliMowing') : data.charging ? t('mowgliNextPage.mowgliCharging') : t('mowgliNextPage.welcomeBack')}
            </div>
          </div>
          <StatusOrb
            tone={data.emergency ? "alert" : data.isMoving ? "live" : data.charging ? "charging" : "resting"}
            size={10}
            label={data.isMoving ? t('mowgliNextPage.orbMowing') : data.charging ? t('mowgliNextPage.orbCharging') : data.emergency ? t('mowgliNextPage.orbAlert') : t('mowgliNextPage.orbIdle')}
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
            <motion.div variants={riseFade}><LiveMapCard polygons={polygons} progress={progress} robot={robotNormalised} coverage={coveragePct} onViewMap={() => navigate("/map")}/></motion.div>
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
              <motion.div variants={riseFade}><LiveMapCard polygons={polygons} progress={progress} robot={robotNormalised} coverage={coveragePct} height={300} onViewMap={() => navigate("/map")}/></motion.div>
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
    onHome: () => void; onStop: () => void; onRearm: () => void;
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
  const {t} = useTranslation();
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
              {data.currentArea ? t('mowgliNextPage.mowgliWithZone', {area: data.currentArea}) : t('mowgliNextPage.mowgliNoZone')}
            </div>
            <h1 className="mn-display" style={{
              fontSize: large ? 38 : 30,
              lineHeight: 1.05, marginTop: 8,
              fontWeight: 400, letterSpacing: '-0.025em',
              color: 'var(--ink, #ECFFF4)',
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
              color: 'var(--ink, #ECFFF4)', letterSpacing: '-0.02em',
            }}>
              {Math.round(data.battery)}
            </div>
            <div style={{
              fontSize: 10, color: 'rgba(236,255,244,0.42)',
              letterSpacing: '0.1em', textTransform: 'uppercase',
              fontWeight: 600, marginTop: 2,
            }}>
              {t('mowgliNextPage.battery')}
            </div>
          </BatteryRing>
        </div>

        {totalArea > 0 && (() => {
          // Grid is published at 5 cm; cells × cellArea = m².
          const cellAreaM2 = 0.05 * 0.05;
          const mowedM2 = todayMowedM2 * cellAreaM2;
          const totalM2 = totalArea * cellAreaM2;
          const pct = Math.round(coveragePct * 100);
          return (
            <div style={{marginTop: large ? 22 : 18}}>
              <div style={{
                display: 'flex', alignItems: 'baseline', justifyContent: 'space-between',
                marginBottom: 8,
              }}>
                <span style={{
                  fontSize: 11, color: 'rgba(236,255,244,0.42)',
                  letterSpacing: '0.08em', textTransform: 'uppercase', fontWeight: 600,
                }}>
                  {t('mowgliNextPage.coverageToday', {pct})}
                </span>
                <span style={{fontSize: 12, color: 'rgba(236,255,244,0.66)', fontWeight: 600, fontFamily: '"Space Grotesk", monospace'}}>
                  {mowedM2.toFixed(0)}<span style={{color: 'rgba(236,255,244,0.42)'}}> / {totalM2.toFixed(0)} m²</span>
                </span>
              </div>
              <ProgressRibbon value={coveragePct} segments={24}/>
            </div>
          );
        })()}

        <div style={{marginTop: large ? 28 : 22}}>
          <ActionCluster phase={phase} {...actions}/>
        </div>
      </div>
    </GlassCard>
  );
}

interface LiveMapCardProps {
  polygons: MiniArea[];
  progress: MiniProgress | null;
  robot?: {x: number; y: number; heading: number};
  coverage: number;
  height?: number;
  onViewMap?: () => void;
}

function LiveMapCard({polygons, progress, robot, coverage, height = 220, onViewMap}: LiveMapCardProps) {
  const {t} = useTranslation();
  const hasArea = polygons.length > 0;
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
            {t('mowgliNextPage.liveMap')}
          </div>
          <div className="mn-display" style={{
            fontSize: 18, fontWeight: 400, marginTop: 2,
            color: 'var(--ink, #ECFFF4)', letterSpacing: '-0.01em',
          }}>
            {t('mowgliNextPage.livePath')}
          </div>
        </div>
        <button onClick={onViewMap} style={{
          display: 'inline-flex', alignItems: 'center', gap: 4,
          background: 'transparent', border: 'none',
          fontSize: 12, fontWeight: 600, color: 'var(--lime, #7CFFB2)', cursor: 'pointer',
        }}>
          {t('mowgliNextPage.viewMap')}
          <ChevronRight size={14} strokeWidth={2.4}/>
        </button>
      </div>
      {hasArea ? (
        <LiveMapMini
          polygons={polygons}
          progress={progress}
          robot={robot}
          coverage={coverage}
          height={height}
        />
      ) : (
        // No recorded area — show an honest empty state rather than
        // LiveMapMini's decorative default polygon + fake robot.
        <div style={{
          height, display: 'flex', flexDirection: 'column',
          alignItems: 'center', justifyContent: 'center', gap: 6,
          padding: '0 22px 22px', textAlign: 'center',
        }}>
          <div className="mn-display" style={{fontSize: 15, color: 'var(--ink, #ECFFF4)'}}>
            {t('liveMapMini.noAreaYet')}
          </div>
          <div style={{fontSize: 12, color: 'rgba(236,255,244,0.42)'}}>
            {t('liveMapMini.noAreaHint')}
          </div>
        </div>
      )}
    </GlassCard>
  );
}

function TilesRow({data}: {data: ReturnType<typeof useMowerData>}) {
  const {t} = useTranslation();
  return (
    <div style={{display: 'grid', gridTemplateColumns: 'repeat(2, 1fr)', gap: 10}}>
      <StatTile label="GPS" value={`${Math.round(data.gps)}`} unit="%"
                hint={data.gpsLabel} accent="cyan" icon={<Wifi size={14}/>}/>
      <StatTile label={t('mowgliNextPage.blades')} value={data.rpm > 0 ? Math.round(data.rpm).toString() : t('mowgliNextPage.bladesOff')}
                unit={data.rpm > 0 ? 'rpm' : ''} hint={`${data.bladeCurrent.toFixed(1)} A`}
                accent="amber" icon={<Sparkles size={14}/>}/>
      <StatTile label={t('mowgliNextPage.motor')} value={data.motorTemp.toFixed(0)} unit="°c"
                hint={`ESC ${data.escTemp.toFixed(0)} °C`}
                accent={data.motorTemp > 55 ? "amber" : "lime"}
                icon={<Thermometer size={14}/>}/>
      <StatTile label={t('mowgliNextPage.rain')} value={data.rain ? t('mowgliNextPage.rainDetected') : t('mowgliNextPage.dry')} unit=""
                hint={data.rain ? t('mowgliNextPage.mowingPaused') : t('mowgliNextPage.goodConditions')}
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
            fontSize: 30, color: 'var(--ink, #ECFFF4)', lineHeight: 1,
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

type HealthRow = {k: string; ok: boolean; note: string; action?: ReactNode};

function HealthCard({data}: {data: ReturnType<typeof useMowerData>}) {
  const {t} = useTranslation();
  const navigate = useNavigate();
  const weather = useWeather();
  const rows: HealthRow[] = [
    {k: t('mowgliNextPage.gpsSignal'),         ok: data.gps > 0,           note: data.gpsLabel},
    {k: data.rain ? t('mowgliNextPage.rainDetectedRow') : t('mowgliNextPage.noRain'),
                              ok: !data.rain,             note: data.rain ? t('mowgliNextPage.mowingPausedShort') : t('mowgliNextPage.conditionsOk')},
    {k: data.emergency ? t('mowgliNextPage.alertActive') : t('mowgliNextPage.noAlerts'),
                              ok: !data.emergency,        note: data.emergency ? t('mowgliNextPage.toRearm') : t('mowgliNextPage.allClear')},
    {k: t('mowgliNextPage.motorTemp', {temp: data.motorTemp.toFixed(0)}),
                              ok: data.motorTemp < 55,    note: data.motorTemp >= 55 ? t('mowgliNextPage.runningHot') : t('mowgliNextPage.nominal')},
  ];
  // Firmware compatibility row — only shown once the bridge has reported a
  // verdict (firmwareCompatible !== null). When incompatible, it reads red and
  // tells the operator to reflash; mowing is blocked by PreFlightCheck.
  if (data.firmwareCompatible !== null) {
    rows.push({
      k: data.firmwareCompatible
        ? t('mowgliNextPage.firmwareOk')
        : t('mowgliNextPage.firmwareIncompatible'),
      ok: data.firmwareCompatible,
      note: data.firmwareCompatible
        ? t('mowgliNextPage.firmwareVersion', {version: data.firmwareVersion || '?'})
        : t('mowgliNextPage.firmwareReflash', {version: data.firmwareVersion || '?'}),
      // When incompatible, offer a one-click jump to the flash screen (opens the
      // onboarding firmware step with the prebuilt flash form ready to go).
      action: data.firmwareCompatible ? undefined : (
        <Button
          type="primary"
          size="small"
          onClick={() => navigate('/onboarding?step=firmware&flash=1')}
        >
          {t('mowgliNextPage.firmwareFlashCta')}
        </Button>
      ),
    });
  }
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
          {t('mowgliNextPage.healthCheck')}
        </div>
        {weather?.available && (
          <WeatherChip condition={weather.condition} tempC={weather.temp_c} rainSoon={weather.is_raining}/>
        )}
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
              <div style={{fontSize: 13, fontWeight: 600, color: 'var(--ink, #ECFFF4)'}}>{r.k}</div>
              <div style={{fontSize: 11, color: 'rgba(236,255,244,0.42)'}}>{r.note}</div>
            </div>
            {r.action && <div style={{flexShrink: 0}}>{r.action}</div>}
          </div>
        ))}
      </div>
    </GlassCard>
  );
}

// ─────────────────────────────────────────────────────────────────────
// Utilities
// ─────────────────────────────────────────────────────────────────────

function greetingFor(t: TFunction) {
  const h = new Date().getHours();
  if (h < 6)  return t('mowgliNextPage.greetingNight');
  if (h < 12) return t('mowgliNextPage.greetingMorning');
  if (h < 18) return t('mowgliNextPage.greetingAfternoon');
  return t('mowgliNextPage.greetingEvening');
}

export default MowgliNextPage;
