import {DISPLAY_MODES, type DisplayMode} from "./displayModes";
export type {DisplayMode} from "./displayModes";
import {createContext, useCallback, useContext, useEffect, useState} from "react";
import type {ThemeMode} from "./colors.ts";
import {cssVars, getColors, setColors} from "./colors.ts";

const DISPLAY_MODE_STORAGE_KEY = 'mowgli.display-mode';

function isDisplayMode(value: string | null): value is DisplayMode {
    return value !== null && (DISPLAY_MODES as readonly string[]).includes(value);
}

function readDisplayMode(): DisplayMode {
    try {
        const stored = window.localStorage.getItem(DISPLAY_MODE_STORAGE_KEY);
        return isDisplayMode(stored) ? stored : 'balanced';
    } catch {
        return 'balanced';
    }
}

interface ThemeContextValue {
    mode: ThemeMode;
    toggleMode: () => void;
    colors: ReturnType<typeof getColors>;
    displayMode: DisplayMode;
    setDisplayMode: (mode: DisplayMode) => void;
}

const ThemeContext = createContext<ThemeContextValue>({
    mode: 'light',
    toggleMode: () => {},
    colors: getColors('light'),
    displayMode: 'balanced',
    setDisplayMode: () => {},
});

// Mowgli is dark-mode-only. The light tokens stay in `colors.ts` for now in
// case we ever revisit, but the provider is hard-locked to dark and the
// toggleMode is a no-op (kept so existing call sites don't break).
export function ThemeProvider({children}: {children: React.ReactNode}) {
    const mode: ThemeMode = 'dark';
    const colors = getColors(mode);
    const [displayMode, setDisplayMode] = useState<DisplayMode>(readDisplayMode);

    useEffect(() => {
        setColors(mode);
        const root = document.documentElement;
        // Single source of truth: mirror the palette into the CSS custom
        // properties so the var() layer (tokens.css gradients, .glass,
        // KEYFRAMES_CSS) resolves from the same place as the JS `colors`.
        const vars = cssVars();
        for (const [k, v] of Object.entries(vars)) root.style.setProperty(k, v);
        root.style.background = colors.bgBase;
        document.body.style.background = colors.bgBase;
        document.body.style.fontFamily = "'Satoshi', 'Inter', -apple-system, BlinkMacSystemFont, 'Helvetica Neue', sans-serif";
        root.style.colorScheme = 'dark';
        const meta = document.querySelector('meta[name="theme-color"]');
        if (meta) meta.setAttribute('content', colors.bgBase);
    }, [mode, colors.bgBase]);

    const toggleMode = useCallback(() => { /* dark-only */ }, []);

    useEffect(() => {
        document.documentElement.dataset.displayMode = displayMode;
        try {
            window.localStorage.setItem(DISPLAY_MODE_STORAGE_KEY, displayMode);
        } catch {
            // Private browsing or a locked-down WebView can reject storage.
            // The selected mode still applies for this session.
        }
    }, [displayMode]);

    return (
        <ThemeContext.Provider value={{mode, toggleMode, colors, displayMode, setDisplayMode}}>
            {children}
        </ThemeContext.Provider>
    );
}

export function useThemeMode() {
    return useContext(ThemeContext);
}
