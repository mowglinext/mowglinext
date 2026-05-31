import type {CSSProperties, ReactNode} from "react";
import {useThemeMode} from "../../theme/ThemeContext.tsx";

/**
 * Shared glass surface. Background, backdrop blur, soft border, and a
 * luminous edge gradient applied at the top-left via mask-composite --
 * the visual signature of the tech-garden language. Used by every page.
 */

interface DashCardProps {
  children: ReactNode;
  style?: CSSProperties;
  tone?: 'accent' | 'danger' | 'glow';
  padding?: number | string;
  onClick?: () => void;
}

export function DashCard({children, style, tone, padding = 18, onClick}: DashCardProps) {
  const {colors} = useThemeMode();
  const bg = tone === 'accent'
    ? `linear-gradient(135deg, ${colors.accent}, ${colors.accent}aa)`
    : tone === 'danger'
      ? 'linear-gradient(135deg, rgba(255,107,122,0.18), rgba(255,107,122,0.04))'
      : 'rgba(11, 24, 20, 0.6)';

  const shadow = tone === 'glow'
    ? '0 24px 60px -20px rgba(0,0,0,0.7), 0 0 60px rgba(124,255,178,0.18), 0 0 0 1px rgba(124,255,178,0.16) inset'
    : '0 24px 60px -20px rgba(0,0,0,0.5), 0 4px 16px -4px rgba(0,0,0,0.3), inset 0 1px 0 rgba(255,255,255,0.04)';

  return (
    <div
      onClick={onClick}
      className={onClick ? 'mn-glass-card mn-card-hover' : 'mn-glass-card'}
      style={{
        position: 'relative',
        background: bg,
        backdropFilter: 'blur(24px) saturate(140%)',
        WebkitBackdropFilter: 'blur(24px) saturate(140%)',
        borderRadius: 18,
        padding,
        border: tone === 'accent' ? 'none' : `1px solid ${colors.border}`,
        boxShadow: shadow,
        cursor: onClick ? 'pointer' : 'default',
        transition: 'transform .15s, box-shadow .15s',
        overflow: 'hidden',
        ...style,
      }}
    >
      {children}
    </div>
  );
}
