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
    <DashCard padding={compact ? 14 : 20} style={{
      display: 'flex', flexDirection: 'column',
      gap: compact ? 8 : 12, minHeight: compact ? 0 : 112,
      position: 'relative', overflow: 'hidden',
    }}>
      {/* subtle accent watermark to differentiate tiles by domain */}
      <div aria-hidden style={{
        position: 'absolute', top: -20, right: -20, width: 80, height: 80, borderRadius: 80,
        background: `radial-gradient(circle, ${tileAccent}18 0%, transparent 70%)`,
        pointerEvents: 'none',
      }}/>
      <div style={{display: 'flex', alignItems: 'center', gap: compact ? 8 : 10, position: 'relative'}}>
        <div style={{
          width: compact ? 28 : 34, height: compact ? 28 : 34, borderRadius: compact ? 9 : 11,
          background: `linear-gradient(135deg, ${tileAccent}28, ${tileAccent}12)`,
          color: tileAccent,
          display: 'flex', alignItems: 'center', justifyContent: 'center',
          flexShrink: 0,
        }}>
          {icon}
        </div>
        <div style={{
          fontSize: compact ? 11 : 12, color: colors.textDim, fontWeight: 600,
          letterSpacing: '0.04em', textTransform: 'uppercase' as const,
        }}>{label}</div>
      </div>
      <div style={{display: 'flex', alignItems: 'baseline', gap: 4, position: 'relative'}}>
        <div className="mn-num" style={{
          fontSize: compact ? 30 : 44, color: colors.text,
          lineHeight: 1,
        }}>
          {value}
        </div>
        {unit && (
          <div style={{
            fontSize: compact ? 12 : 14, color: colors.textDim, fontWeight: 500,
            fontFamily: "'Geist Mono', 'JetBrains Mono', monospace",
            textTransform: 'lowercase' as const, letterSpacing: '0.04em',
          }}>{unit}</div>
        )}
      </div>
      {visual ?? (trail && (
        <Sparkline
          data={trail} width={compact ? 120 : 160} height={compact ? 18 : 22}
          stroke={tileAccent} fill={`${tileAccent}22`} strokeWidth={1.8}
        />
      ))}
      {hint && (
        <div style={{
          fontSize: compact ? 10 : 11, color: colors.textMuted,
          marginTop: 'auto', fontWeight: 500,
        }}>{hint}</div>
      )}
    </DashCard>
  );
}
