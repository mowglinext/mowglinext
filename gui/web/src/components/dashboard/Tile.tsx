import type {ReactNode} from "react";
import {useThemeMode} from "../../theme/ThemeContext.tsx";
import {DashCard} from "./Card.tsx";
import {Sparkline} from "./Sparkline.tsx";

interface DashTileProps {
  icon: ReactNode;
  label: string;
  value: string | number;
  unit?: string;
  accent?: string;
  hint?: string;
  trail?: number[];
  /** Optional domain-specific glyph (battery, gps bars, tach, thermometer);
   *  takes precedence over `trail` when provided. */
  visual?: ReactNode;
  compact?: boolean;
}

export function DashTile({icon, label, value, unit, accent, hint, trail, visual, compact}: DashTileProps) {
  const {colors} = useThemeMode();
  const tileAccent = accent ?? colors.accent;
  return (
    <DashCard padding={compact ? 12 : 18} style={{
      display: 'flex', flexDirection: 'column',
      gap: compact ? 6 : 10, minHeight: compact ? 0 : 96,
    }}>
      <div style={{display: 'flex', alignItems: 'center', gap: compact ? 8 : 10}}>
        <div style={{
          width: compact ? 26 : 32, height: compact ? 26 : 32, borderRadius: compact ? 8 : 10,
          background: `${tileAccent}20`, color: tileAccent,
          display: 'flex', alignItems: 'center', justifyContent: 'center',
          flexShrink: 0,
        }}>
          {icon}
        </div>
        <div style={{fontSize: compact ? 11 : 12, color: colors.textDim, fontWeight: 500}}>{label}</div>
      </div>
      <div style={{display: 'flex', alignItems: 'baseline', gap: 4}}>
        <div style={{fontSize: compact ? 20 : 26, fontWeight: 700, color: colors.text, letterSpacing: '-0.02em'}}>
          {value}
        </div>
        {unit && <div style={{fontSize: compact ? 11 : 13, color: colors.textDim, fontWeight: 500}}>{unit}</div>}
      </div>
      {visual ?? (trail && (
        <Sparkline
          data={trail} width={compact ? 120 : 160} height={compact ? 18 : 22}
          stroke={tileAccent} fill={`${tileAccent}22`} strokeWidth={1.8}
        />
      ))}
      {hint && <div style={{fontSize: compact ? 10 : 11, color: colors.textMuted}}>{hint}</div>}
    </DashCard>
  );
}
