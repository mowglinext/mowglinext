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

// Mowgli dark palette -- premium "tech-garden" tokens shared with the
// /concept prototype. Deep emerald canvas, lime hero accent, aurora-
// cyan secondary, ember amber for warnings, rose for danger. Paper-warm
// ink so the editorial type sits on the surface like print.
const DARK: ColorTokens = {
  bgBase: '#02110D',           // puits émeraude
  bgCard: '#0B1814',
  bgElevated: '#101F19',
  bgSubtle: '#061812',
  primary: '#7CFFB2',          // lime hero
  primaryLight: '#A3FFCB',
  primaryDark: '#45D688',
  primaryBg: 'rgba(124, 255, 178, 0.10)',
  accent: '#7CFFB2',
  accentAmber: '#F3A85C',
  danger: '#FF6B7A',
  dangerBg: 'rgba(255, 107, 122, 0.14)',
  warning: '#F3A85C',
  info: '#45D6E8',             // aurora cyan
  success: '#7CFFB2',
  text: '#ECFFF4',             // papier-vert chaud
  textSecondary: 'rgba(236, 255, 244, 0.66)',
  muted: 'rgba(236, 255, 244, 0.42)',
  border: 'rgba(236, 255, 244, 0.07)',
  borderSubtle: 'rgba(236, 255, 244, 0.04)',
  glassBackground: 'rgba(11, 24, 20, 0.78)',
  glassBorder: '1px solid rgba(236, 255, 244, 0.08)',
  glassShadow: '0 24px 60px -20px rgba(0, 0, 0, 0.7), 0 4px 16px -4px rgba(0, 0, 0, 0.4)',

  panel: '#0B1814',
  panelHi: '#101F19',
  accentSoft: 'rgba(124, 255, 178, 0.10)',
  sky: '#45D6E8',
  skySoft: 'rgba(69, 214, 232, 0.12)',
  amber: '#F3A85C',
  amberSoft: 'rgba(243, 168, 92, 0.14)',
  pink: '#FF6B7A',
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
