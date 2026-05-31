import {useThemeMode} from "../../theme/ThemeContext.tsx";

/**
 * Compact domain-specific visualisations used by the dashboard tiles instead
 * of the generic sparkline. Each glyph is a single SVG sized for the tile
 * footer (width ~160, height ~22 desktop / 18 mobile).
 */

interface BaseProps {
  width?: number;
  height?: number;
  color: string;
}

interface BatteryGlyphProps extends BaseProps {
  /** 0..100 */
  percent: number;
  charging: boolean;
}

export function BatteryGlyph({percent, charging, width = 160, height = 22, color}: BatteryGlyphProps) {
  const {colors} = useThemeMode();
  const bodyW = width - 6;
  const fillW = (bodyW - 4) * Math.max(0, Math.min(1, percent / 100));
  return (
    <svg width={width} height={height} viewBox={`0 0 ${width} ${height}`}>
      <rect x={1} y={3} width={bodyW} height={height - 6} rx={3} ry={3}
            fill="none" stroke={colors.borderSubtle} strokeWidth={1.2}/>
      <rect x={bodyW + 1} y={height / 2 - 4} width={4} height={8} rx={1}
            fill={colors.borderSubtle}/>
      <rect x={3} y={5} width={fillW} height={height - 10} rx={2} fill={color}/>
      {charging && (
        <g transform={`translate(${width / 2 - 4}, ${height / 2 - 5})`}>
          <path d="M 4 0 L 0 5 L 3 5 L 1 10 L 8 4 L 5 4 Z"
                fill="#fff" stroke={color} strokeWidth={0.5}/>
        </g>
      )}
    </svg>
  );
}

interface GpsBarsProps extends BaseProps {
  /** 0..100 */
  percent: number;
  /** RTK fixed / float / 3D / etc. */
  ok: boolean;
}

export function GpsBars({percent, ok, width = 160, height = 22, color}: GpsBarsProps) {
  const {colors} = useThemeMode();
  const numBars = 8;
  const barW = (width - (numBars - 1) * 3) / numBars;
  const activeBars = Math.round((percent / 100) * numBars);
  return (
    <svg width={width} height={height} viewBox={`0 0 ${width} ${height}`}>
      {Array.from({length: numBars}).map((_, i) => {
        const h = 4 + ((i + 1) / numBars) * (height - 6);
        const x = i * (barW + 3);
        const y = height - 1 - h;
        const active = i < activeBars;
        return (
          <rect key={i}
                x={x} y={y} width={barW} height={h} rx={1}
                fill={active ? (ok ? color : colors.amber) : colors.borderSubtle}/>
        );
      })}
    </svg>
  );
}

interface BladeTachProps extends BaseProps {
  /** rpm; 0 = off */
  rpm: number;
  /** Maximum RPM for full needle deflection (mower-dependent; default 4000) */
  maxRpm?: number;
}

export function BladeTach({rpm, maxRpm = 4000, width = 160, height = 22, color}: BladeTachProps) {
  const {colors} = useThemeMode();
  const cy = height - 1;
  const radius = Math.min(width / 2 - 2, height + 4);
  const t = Math.max(0, Math.min(1, rpm / maxRpm));
  const angle = -Math.PI + t * Math.PI;
  const needleX = width / 2 + Math.cos(angle) * (radius - 4);
  const needleY = cy + Math.sin(angle) * (radius - 4);

  return (
    <svg width={width} height={height} viewBox={`0 0 ${width} ${height}`}>
      <path
        d={`M ${width / 2 - radius} ${cy} A ${radius} ${radius} 0 0 1 ${width / 2 + radius} ${cy}`}
        fill="none" stroke={colors.borderSubtle} strokeWidth={1.5}
      />
      <path
        d={`M ${width / 2 - radius} ${cy} A ${radius} ${radius} 0 0 1 ${
          width / 2 + Math.cos(-Math.PI + t * Math.PI) * radius
        } ${
          cy + Math.sin(-Math.PI + t * Math.PI) * radius
        }`}
        fill="none" stroke={rpm > 0 ? color : colors.borderSubtle} strokeWidth={2}
        strokeLinecap="round"
      />
      <line x1={width / 2} y1={cy} x2={needleX} y2={needleY}
            stroke={rpm > 0 ? color : colors.muted} strokeWidth={1.5} strokeLinecap="round"/>
      <circle cx={width / 2} cy={cy} r={2} fill={color}/>
    </svg>
  );
}

interface DualThermoProps extends BaseProps {
  motorC: number;
  escC: number;
  /** Temperature value mapped to full bar */
  maxC?: number;
}

export function DualThermo({motorC, escC, maxC = 80, width = 160, height = 22, color}: DualThermoProps) {
  const {colors} = useThemeMode();
  const trackW = width - 36;
  const motorFrac = Math.max(0, Math.min(1, motorC / maxC));
  const escFrac = Math.max(0, Math.min(1, escC / maxC));
  const warnMotor = motorC > 55;
  const warnEsc = escC > 60;

  const bar = (label: string, frac: number, warn: boolean, y: number) => (
    <g>
      <text x={0} y={y + 6} fontSize={9} fill={colors.textMuted}>{label}</text>
      <rect x={20} y={y + 2} width={trackW} height={4} rx={2} fill={colors.borderSubtle}/>
      <rect x={20} y={y + 2} width={trackW * frac} height={4} rx={2}
            fill={warn ? colors.amber : color}/>
    </g>
  );

  return (
    <svg width={width} height={height} viewBox={`0 0 ${width} ${height}`}>
      {bar('M', motorFrac, warnMotor, 2)}
      {bar('E', escFrac, warnEsc, 13)}
    </svg>
  );
}
