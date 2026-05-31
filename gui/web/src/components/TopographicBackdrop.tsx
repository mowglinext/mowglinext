import {useThemeMode} from "../theme/ThemeContext.tsx";

/**
 * Decorative topographic-contour layer. SVG of irregular concentric lines,
 * tuned to read like a USGS / hiking map. Positioned absolutely behind a
 * hero surface; pointer-events: none so it never interferes with content.
 *
 * Quiet by default (0.07 opacity) -- it's atmosphere, not chart data. Pass
 * `intensity` to bump it on emptier surfaces.
 */

interface TopographicBackdropProps {
  intensity?: number;
  rotate?: number;
}

export function TopographicBackdrop({intensity = 0.07, rotate = -8}: TopographicBackdropProps) {
  const {colors} = useThemeMode();
  return (
    <svg
      aria-hidden
      viewBox="0 0 600 320"
      preserveAspectRatio="xMidYMid slice"
      style={{
        position: 'absolute', inset: 0, width: '100%', height: '100%',
        pointerEvents: 'none', opacity: intensity,
        transform: `rotate(${rotate}deg) scale(1.4)`,
        transformOrigin: 'center',
      }}
    >
      <g fill="none" stroke={colors.accent} strokeWidth={0.6}>
        <path d="M -40 180 Q 80 140, 200 180 T 440 200 T 700 180"/>
        <path d="M -40 200 Q 80 156, 220 198 T 460 224 T 700 200"/>
        <path d="M -40 220 Q 100 174, 240 216 T 480 244 T 700 220"/>
        <path d="M -40 240 Q 120 192, 260 234 T 500 264 T 700 240"/>
        <path d="M -40 260 Q 140 210, 280 252 T 520 284 T 700 260"/>

        <path d="M -40 80 Q 100 50, 240 90 T 500 110 T 700 80"/>
        <path d="M -40 60 Q 80 30, 220 70 T 480 90 T 700 60"/>

        <ellipse cx={120} cy={150} rx={70} ry={42}/>
        <ellipse cx={120} cy={150} rx={48} ry={28}/>
        <ellipse cx={120} cy={150} rx={26} ry={14}/>

        <ellipse cx={470} cy={130} rx={60} ry={36} transform="rotate(15 470 130)"/>
        <ellipse cx={470} cy={130} rx={38} ry={22} transform="rotate(15 470 130)"/>
        <ellipse cx={470} cy={130} rx={18} ry={10} transform="rotate(15 470 130)"/>
      </g>
      {/* a handful of survey markers */}
      <g fill={colors.accent} opacity={0.5}>
        <circle cx={120} cy={150} r={1.2}/>
        <circle cx={470} cy={130} r={1.2}/>
      </g>
    </svg>
  );
}
