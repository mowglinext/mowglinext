import {useTranslation} from "react-i18next";
import {useThemeMode} from "../../theme/ThemeContext.tsx";

/**
 * Right-side live-preview panel for the Settings page.
 *
 * Reflects the currently-edited values (chassis dimensions, tool width,
 * battery thresholds) onto small SVG schematics so the operator sees the
 * effect of a tweak before saving. Read-only -- inputs stay on the left.
 *
 * Renders on desktop only (caller is responsible for hiding on mobile).
 */

interface SettingsPreviewProps {
  values: Record<string, unknown>;
  section: string;
}

const m = (v: unknown, fallback = 0): number => {
  if (typeof v === 'number' && Number.isFinite(v)) return v;
  if (typeof v === 'string') {
    const n = parseFloat(v);
    return Number.isFinite(n) ? n : fallback;
  }
  return fallback;
};

function ChassisPreview({values}: {values: Record<string, unknown>}) {
  const {t} = useTranslation();
  const {colors} = useThemeMode();
  const length = m(values.chassis_length, 0.62);
  const width = m(values.chassis_width, 0.46);
  const wheelTrack = m(values.wheel_track, 0.32);
  const toolWidth = m(values.tool_width, 0.18);
  const bladeRadius = m(values.blade_radius, 0.09);
  const wheelRadius = m(values.wheel_radius, 0.045);
  const wheelWidth = m(values.wheel_width, 0.04);
  const chassisCenterX = m(values.chassis_center_x, 0.18);
  const wheelXOffset = m(values.wheel_x_offset, 0);

  // Scale so the longest dimension fits the SVG, with padding.
  const longest = Math.max(length, width) || 1;
  const target = 180;
  const scale = target / longest;
  const px = (v: number) => v * scale;

  const cw = px(width);
  const cl = px(length);
  const svgW = 220;
  const svgH = 220;
  const cx = svgW / 2;
  const cy = svgH / 2;
  // base_link is the rear drive-wheel axle, offset toward the rear (down in
  // this forward-up view) from the chassis centre by chassis_center_x. The
  // drive wheels sit there (shifted forward by wheel_x_offset when set).
  const axleY = cy + px(chassisCenterX - wheelXOffset);

  const labelStyle = {fontSize: 9, fill: colors.textDim};

  return (
    <div>
      <div style={{
        fontSize: 11, color: colors.textMuted, letterSpacing: '0.08em',
        textTransform: 'uppercase' as const, marginBottom: 8,
      }}>
        {t("settingsPreview.chassisTitle")}
      </div>
      <svg viewBox={`0 0 ${svgW} ${svgH}`} width="100%" style={{display: 'block', maxHeight: svgH}}>
        {/* tool-width footprint (cut area) */}
        <rect x={cx - px(toolWidth) / 2} y={cy - cl / 2 + 4}
              width={px(toolWidth)} height={cl - 8}
              fill={colors.accentSoft} stroke={colors.accent} strokeWidth={1} strokeDasharray="3 3"/>
        <text x={cx + px(toolWidth) / 2 + 4} y={cy + 3} {...labelStyle}>
          {t("settingsPreview.cutLabel", {cm: (toolWidth * 100).toFixed(0)})}
        </text>

        {/* chassis */}
        <rect x={cx - cw / 2} y={cy - cl / 2}
              width={cw} height={cl} rx={Math.min(cw, cl) * 0.12}
              fill={colors.bgElevated} stroke={colors.text} strokeWidth={1.5}/>

        {/* wheel track (dashed band across the rear axle) */}
        <rect x={cx - px(wheelTrack) / 2 - 4} y={axleY - 4}
              width={px(wheelTrack) + 8} height={8}
              fill="none" stroke={colors.amber} strokeWidth={0.8} strokeDasharray="2 2"/>
        <text x={cx - px(wheelTrack) / 2 - 6} y={axleY + 18} textAnchor="end" {...labelStyle}>
          {t("settingsPreview.trackLabel", {cm: (wheelTrack * 100).toFixed(0)})}
        </text>

        {/* drive wheels at the rear axle. Seen from above, a wheel's diameter
            runs along the forward (vertical) axis and the tyre width laterally,
            so each is a tall rectangle — not a square. */}
        <rect x={cx - px(wheelTrack) / 2 - px(wheelWidth) / 2} y={axleY - px(wheelRadius)}
              width={px(wheelWidth)} height={px(wheelRadius) * 2}
              rx={2} fill={colors.text}/>
        <rect x={cx + px(wheelTrack) / 2 - px(wheelWidth) / 2} y={axleY - px(wheelRadius)}
              width={px(wheelWidth)} height={px(wheelRadius) * 2}
              rx={2} fill={colors.text}/>

        {/* base_link marker at the rear axle */}
        <circle cx={cx} cy={axleY} r={2} fill={colors.textDim}/>
        <text x={cx} y={axleY + px(wheelRadius) + 9} textAnchor="middle"
              fontSize={7} fill={colors.textDim} fontFamily="monospace">
          base_link
        </text>

        {/* blade (centred under the chassis, per the URDF) */}
        <circle cx={cx} cy={cy} r={px(bladeRadius)}
                fill={colors.amberSoft} stroke={colors.amber}/>

        {/* length / width labels */}
        <text x={cx} y={cy - cl / 2 - 6} textAnchor="middle" {...labelStyle}>
          {(length * 100).toFixed(0)}cm
        </text>
        <text x={cx + cw / 2 + 4} y={cy} {...labelStyle}>
          {(width * 100).toFixed(0)}cm
        </text>
      </svg>
      <div style={{
        marginTop: 8, fontSize: 11, color: colors.textDim, lineHeight: 1.5,
      }}>
        {t("settingsPreview.toolWidthNote")}
      </div>
    </div>
  );
}

function SwathsPreview({values}: {values: Record<string, unknown>}) {
  const {t} = useTranslation();
  const {colors} = useThemeMode();
  const toolWidth = m(values.tool_width, 0.18);
  const safetyInset = m(values.chassis_safety_inset, 0.05);
  const fieldW = 4.0; // metres -- a 4m strip for visualization
  const fieldL = 2.5;
  const target = 220;
  const scale = target / fieldW;
  const svgW = 240;
  const svgH = fieldL * scale + 30;

  const innerW = fieldW - safetyInset * 2;
  const innerStartX = 10 + safetyInset * scale;
  const innerTopY = 24 + safetyInset * scale;
  const innerH = fieldL * scale - safetyInset * scale * 2;
  const innerWpx = innerW * scale;

  // Lay out swaths spaced by toolWidth
  const swathCount = Math.floor(innerW / toolWidth);
  const swathPx = toolWidth * scale;

  return (
    <div style={{marginTop: 14}}>
      <div style={{
        fontSize: 11, color: colors.textMuted, letterSpacing: '0.08em',
        textTransform: 'uppercase' as const, marginBottom: 8,
      }}>
        {t("settingsPreview.swathLayoutTitle")}
      </div>
      <svg viewBox={`0 0 ${svgW} ${svgH}`} width="100%" style={{display: 'block', maxHeight: svgH}}>
        {/* field outline */}
        <rect x={10} y={24} width={fieldW * scale} height={fieldL * scale}
              fill={colors.bgElevated} stroke={colors.border} strokeWidth={1}/>
        {/* headland inset */}
        <rect x={innerStartX} y={innerTopY} width={innerWpx} height={innerH}
              fill="none" stroke={colors.amber} strokeWidth={1} strokeDasharray="2 3"/>
        {/* swaths */}
        {Array.from({length: swathCount}).map((_, i) => (
          <rect key={i}
                x={innerStartX + i * swathPx} y={innerTopY}
                width={swathPx - 1} height={innerH}
                fill={i % 2 === 0 ? `${colors.accent}30` : `${colors.accent}20`}
                stroke={colors.accent} strokeWidth={0.4}/>
        ))}
        <text x={svgW / 2} y={16} textAnchor="middle" fontSize={10} fill={colors.textDim}>
          {t("settingsPreview.swathCountLabel", {count: swathCount, cm: (toolWidth * 100).toFixed(0)})}
        </text>
      </svg>
    </div>
  );
}

function BatteryPreview({values}: {values: Record<string, unknown>}) {
  const {t} = useTranslation();
  const {colors} = useThemeMode();
  const full = m(values.battery_full_voltage, 28.5);
  const empty = m(values.battery_empty_voltage, 24.0);
  const fullPct = m(values.battery_full_percent, 95);
  // Real thresholds from mowgli_robot.yaml: battery_low_percent starts docking,
  // battery_critical_percent forces an emergency dock.
  const lowReturn = m(values.battery_low_percent, 20);
  const criticalReturn = m(values.battery_critical_percent, 10);

  const trackW = 200;
  return (
    <div style={{marginTop: 14}}>
      <div style={{
        fontSize: 11, color: colors.textMuted, letterSpacing: '0.08em',
        textTransform: 'uppercase' as const, marginBottom: 8,
      }}>
        {t("settingsPreview.batteryThresholds")}
      </div>
      <div style={{position: 'relative', width: trackW, height: 28}}>
        <div style={{
          position: 'absolute', inset: 0, borderRadius: 4,
          background: `linear-gradient(90deg, ${colors.danger}, ${colors.amber} 35%, ${colors.accent} 70%)`,
          opacity: 0.45,
        }}/>
        {[criticalReturn, lowReturn].map((pct, i) => (
          <div key={i} style={{
            position: 'absolute', left: `${pct}%`, top: -4, bottom: -4,
            width: 1.5, background: i === 0 ? colors.danger : colors.amber,
          }}/>
        ))}
        <div style={{
          position: 'absolute', left: `${criticalReturn}%`, top: -16, fontSize: 9, color: colors.danger,
          transform: 'translateX(-50%)',
        }}>
          {criticalReturn}%
        </div>
        <div style={{
          position: 'absolute', left: `${lowReturn}%`, bottom: -16, fontSize: 9, color: colors.amber,
          transform: 'translateX(-50%)',
        }}>
          {lowReturn}%
        </div>
      </div>
      <div style={{fontSize: 11, color: colors.textDim, marginTop: 22, lineHeight: 1.5}}>
        {t("settingsPreview.batteryVoltageLine", {
          empty: empty.toFixed(2), full: full.toFixed(2), fullPct,
        })}{" "}
        {t("settingsPreview.batteryReturnLine", {low: lowReturn, critical: criticalReturn})}
      </div>
    </div>
  );
}

export function SettingsPreview({values, section}: SettingsPreviewProps) {
  const {t} = useTranslation();
  const {colors} = useThemeMode();

  const showChassis = section === 'hardware' || section === 'mowing' || section === 'navigation';
  const showSwaths = section === 'hardware' || section === 'mowing';
  const showBattery = section === 'battery';

  if (!showChassis && !showSwaths && !showBattery) {
    return (
      <div style={{
        background: colors.bgCard, borderRadius: 12, padding: 16,
        color: colors.textMuted, fontSize: 12,
      }}>
        {t("settingsPreview.emptyState")}
      </div>
    );
  }

  return (
    <div style={{
      background: colors.bgCard, borderRadius: 12, padding: 16,
      position: 'sticky', top: 8,
    }}>
      {showChassis && <ChassisPreview values={values}/>}
      {showSwaths && <SwathsPreview values={values}/>}
      {showBattery && <BatteryPreview values={values}/>}
    </div>
  );
}
