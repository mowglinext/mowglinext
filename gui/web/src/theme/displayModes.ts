export const DISPLAY_MODES = ['visual', 'balanced', 'efficient'] as const;
export type DisplayMode = typeof DISPLAY_MODES[number];
