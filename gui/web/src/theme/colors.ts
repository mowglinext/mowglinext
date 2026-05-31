/**
 * MowgliNext brand tokens -- Direction B extension
 *
 * All existing field names are preserved for backwards compat;
 * new fields are additive.
 */

export type ThemeMode = 'light' | 'dark';

interface ColorTokens {
  bgBase: string;
  bgCard: string;
  bgElevated: string;
  bgSubtle: string;
  primary: string;
  primaryLight: string;
  primaryDark: string;
  primaryBg: string;
  accent: string;
  accentAmber: string;
  danger: string;
  dangerBg: string;
  warning: string;
  info: string;
  success: string;
  text: string;
  textSecondary: string;
  muted: string;
  border: string;
  borderSubtle: string;
  glassBackground: string;
  glassBorder: string;
  glassShadow: string;

  // Direction B additions
  /** Primary card surface */
  panel: string;
  /** Elevated / hover card surface */
  panelHi: string;
  /** Translucent green bg for chips/tiles */
  accentSoft: string;
  /** Secondary data color (GPS, sky, charts) */
  sky: string;
  skySoft: string;
  /** Warning / low-battery accent */
  amber: string;
  amberSoft: string;
  /** Tertiary accent (decorative) */
  pink: string;
  /** Body label color (~60% opacity) */
  textDim: string;
  /** Caption color (~38% opacity) */
  textMuted: string;
}

const LIGHT: ColorTokens = {
  bgBase: '#FAFAF7',
  bgCard: '#FFFFFF',
  bgElevated: '#F2F2EF',
  bgSubtle: '#EDEDEA',
  primary: '#1B9D52',
  primaryLight: '#2CC76B',
  primaryDark: '#14853F',
  primaryBg: 'rgba(27, 157, 82, 0.08)',
  accent: '#1B9D52',
  accentAmber: '#F5A523',
  danger: '#C93020',
  dangerBg: 'rgba(201, 48, 32, 0.08)',
  warning: '#F5A523',
  info: '#1565C0',
  success: '#1B9D52',
  text: '#141614',
  textSecondary: 'rgba(20, 22, 20, 0.62)',
  muted: '#9E9E9E',
  border: 'rgba(0, 0, 0, 0.08)',
  borderSubtle: 'rgba(0, 0, 0, 0.05)',
  glassBackground: 'rgba(255, 255, 255, 0.85)',
  glassBorder: '1px solid rgba(0, 0, 0, 0.08)',
  glassShadow: '0 2px 12px rgba(0, 0, 0, 0.08)',

  panel: '#FFFFFF',
  panelHi: '#F6F6F3',
  accentSoft: 'rgba(27, 157, 82, 0.10)',
  sky: '#3A8FD9',
  skySoft: 'rgba(58, 143, 217, 0.10)',
  amber: '#E8A028',
  amberSoft: 'rgba(232, 160, 40, 0.12)',
  pink: '#E07598',
  textDim: 'rgba(20, 22, 20, 0.62)',
  textMuted: 'rgba(20, 22, 20, 0.40)',
};

// Mowgli dark palette -- forest floor at dusk. Moss-green dominant, ember
// amber accent for warnings + dock, the rest is paper/charcoal so the
// editorial type can carry the surface.
const DARK: ColorTokens = {
  bgBase: '#0B100D',           // deep moss-charcoal
  bgCard: '#161C18',
  bgElevated: '#1B221E',
  bgSubtle: '#11171393',
  primary: '#5EE3A0',          // pale mint that pops against the dark
  primaryLight: '#7CECB3',
  primaryDark: '#3FC983',
  primaryBg: 'rgba(94, 227, 160, 0.10)',
  accent: '#5EE3A0',
  accentAmber: '#F3A85C',
  danger: '#F26565',
  dangerBg: 'rgba(242, 101, 101, 0.14)',
  warning: '#F3A85C',
  info: '#8FB8FF',
  success: '#5EE3A0',
  text: '#F4F1EA',             // paper-warm, not pure white
  textSecondary: 'rgba(244, 241, 234, 0.62)',
  muted: 'rgba(244, 241, 234, 0.42)',
  border: 'rgba(244, 241, 234, 0.06)',
  borderSubtle: 'rgba(244, 241, 234, 0.035)',
  glassBackground: 'rgba(22, 28, 24, 0.78)',
  glassBorder: '1px solid rgba(244, 241, 234, 0.08)',
  glassShadow: '0 12px 40px rgba(0, 0, 0, 0.55)',

  panel: '#161C18',
  panelHi: '#1B221E',
  accentSoft: 'rgba(94, 227, 160, 0.10)',
  sky: '#8FB8FF',
  skySoft: 'rgba(143, 184, 255, 0.12)',
  amber: '#F3A85C',
  amberSoft: 'rgba(243, 168, 92, 0.14)',
  pink: '#E8839C',
  textDim: 'rgba(244, 241, 234, 0.62)',
  textMuted: 'rgba(244, 241, 234, 0.40)',
};

export function getColors(mode: ThemeMode): ColorTokens {
  return mode === 'dark' ? DARK : LIGHT;
}

export let COLORS: ColorTokens = LIGHT;

export function setColors(mode: ThemeMode) {
  COLORS = getColors(mode);
}
