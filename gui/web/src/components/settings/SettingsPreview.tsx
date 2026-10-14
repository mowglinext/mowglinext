import {MowerPreview} from "../robot/MowerPreview";
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

  const showChassis = section === 'mowing' || section === 'navigation';
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
      {showChassis && <MowerPreview values={values} compact/>}
      {showSwaths && <SwathsPreview values={values}/>}
      {showBattery && <BatteryPreview values={values}/>}
    </div>
  );
}
