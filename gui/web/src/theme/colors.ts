/**
 * MowgliNext brand tokens -- Direction B extension
 *
 * All existing field names are preserved for backwards compat;
 * new fields are additive.
 */

export type ThemeMode = 'light' | 'dark';

/**
 * SINGLE SOURCE OF TRUTH for every brand colour.
 *
 * The dark "tech-garden" palette is defined exactly once here. The `DARK`
 * token object below derives from it, and `cssVars()` mirrors it into the
 * `[data-concept]` CSS custom properties at runtime (see ThemeContext), so
 * inline-style JS (useThemeMode) and CSS-var consumers (.glass, KEYFRAMES_CSS)
 * read from the same place. Do NOT re-hardcode these hex values anywhere else.
 */
export const PALETTE = {
  bgDeep:       '#02110D',
  bgCanvas:     '#061812',
  bgCard:       '#0B1814',
  bgElevated:   '#101F19',
  lime:         '#7CFFB2',
  limeBright:   '#A3FFCB',
  mint:         '#45D688',
  emerald:      '#2BAA66',
  emeraldDeep:  '#167A48',
  auroraCyan:   '#45D6E8',
  auroraViolet: '#6B7FFF',
  amber:        '#F3A85C',
  rose:         '#FF6B7A',
  ink:          '#ECFFF4',
} as const;

/** RGB triple of the warm paper-green ink — the basis for every ink opacity stop. */
const INK_RGB = '236, 255, 244';
const LIME_RGB = '124, 255, 178';

/** Translucent ink at an arbitrary opacity (the app's most common "dim text" need). */
export function inkAlpha(opacity: number): string {
  return `rgba(${INK_RGB}, ${opacity})`;
}
/** Translucent lime accent at an arbitrary opacity. */
export function limeAlpha(opacity: number): string {
  return `rgba(${LIME_RGB}, ${opacity})`;
}

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

  // Named brand accents (mirror the concept CSS vars; single-sourced from PALETTE)
  /** Mid-green between lime and emerald — gradient mid-stop, "on" switch fill */
  mint: string;
  /** Deepest brand green — gradient end-stop, robot-body fill */
  emeraldDeep: string;
  /** Decorative aurora violet (rare tertiary accent) */
  auroraViolet: string;
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

  mint: '#2CC76B',
  emeraldDeep: '#14853F',
  auroraViolet: '#5B6CE0',
};

// Mowgli dark palette -- premium "tech-garden" tokens shared with the
// /concept prototype. Deep emerald canvas, lime hero accent, aurora-
// cyan secondary, ember amber for warnings, rose for danger. Paper-warm
// ink so the editorial type sits on the surface like print.
const DARK: ColorTokens = {
  bgBase: PALETTE.bgDeep,           // puits émeraude
  bgCard: PALETTE.bgCard,
  bgElevated: PALETTE.bgElevated,
  bgSubtle: PALETTE.bgCanvas,
  primary: PALETTE.lime,            // lime hero
  primaryLight: PALETTE.limeBright,
  primaryDark: PALETTE.mint,
  primaryBg: limeAlpha(0.10),
  accent: PALETTE.lime,
  accentAmber: PALETTE.amber,
  danger: PALETTE.rose,
  dangerBg: 'rgba(255, 107, 122, 0.14)',
  warning: PALETTE.amber,
  info: PALETTE.auroraCyan,         // aurora cyan
  success: PALETTE.lime,
  text: PALETTE.ink,                // papier-vert chaud
  textSecondary: inkAlpha(0.66),
  muted: inkAlpha(0.42),
  border: inkAlpha(0.07),
  borderSubtle: inkAlpha(0.04),
  glassBackground: 'rgba(11, 24, 20, 0.78)',
  glassBorder: `1px solid ${inkAlpha(0.08)}`,
  glassShadow: '0 24px 60px -20px rgba(0, 0, 0, 0.7), 0 4px 16px -4px rgba(0, 0, 0, 0.4)',

  panel: PALETTE.bgCard,
  panelHi: PALETTE.bgElevated,
  accentSoft: limeAlpha(0.10),
  sky: PALETTE.auroraCyan,
  skySoft: 'rgba(69, 214, 232, 0.12)',
  amber: PALETTE.amber,
  amberSoft: 'rgba(243, 168, 92, 0.14)',
  pink: PALETTE.rose,
  textDim: inkAlpha(0.62),
  textMuted: inkAlpha(0.40),

  mint: PALETTE.mint,
  emeraldDeep: PALETTE.emeraldDeep,
  auroraViolet: PALETTE.auroraViolet,
};

export function getColors(mode: ThemeMode): ColorTokens {
  return mode === 'dark' ? DARK : LIGHT;
}

export let COLORS: ColorTokens = DARK;

export function setColors(mode: ThemeMode) {
  COLORS = getColors(mode);
}

/**
 * Maps the canonical palette into the `[data-concept]` CSS custom properties.
 * ThemeContext writes these onto document.documentElement at startup so the
 * CSS-var layer (tokens.css gradients, concept.css .glass, KEYFRAMES_CSS)
 * resolves from the same source as the JS `colors` object.
 */
export function cssVars(): Record<string, string> {
  return {
    '--bg-deep': PALETTE.bgDeep,
    '--bg-canvas': PALETTE.bgCanvas,
    '--bg-card': 'rgba(255, 255, 255, 0.045)',
    '--bg-elevated': 'rgba(255, 255, 255, 0.07)',
    '--bg-pressed': 'rgba(255, 255, 255, 0.10)',
    '--bg-card-solid': PALETTE.bgCard,
    '--border-soft': 'rgba(255, 255, 255, 0.08)',
    '--border-sharp': 'rgba(255, 255, 255, 0.14)',
    '--border-glow': limeAlpha(0.22),
    '--lime': PALETTE.lime,
    '--lime-bright': PALETTE.limeBright,
    '--mint': PALETTE.mint,
    '--emerald': PALETTE.emerald,
    '--emerald-deep': PALETTE.emeraldDeep,
    '--aurora-cyan': PALETTE.auroraCyan,
    '--aurora-violet': PALETTE.auroraViolet,
    '--amber': PALETTE.amber,
    '--rose': PALETTE.rose,
    '--ink': PALETTE.ink,
    '--ink-2': inkAlpha(0.66),
    '--ink-3': inkAlpha(0.42),
    '--ink-4': inkAlpha(0.24),
  };
}
