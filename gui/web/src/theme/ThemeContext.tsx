import {createContext, useCallback, useContext, useEffect} from "react";
import type {ThemeMode} from "./colors.ts";
import {cssVars, getColors, setColors} from "./colors.ts";

interface ThemeContextValue {
    mode: ThemeMode;
    toggleMode: () => void;
    colors: ReturnType<typeof getColors>;
}

const ThemeContext = createContext<ThemeContextValue>({
    mode: 'light',
    toggleMode: () => {},
    colors: getColors('light'),
});

// Mowgli is dark-mode-only. The light tokens stay in `colors.ts` for now in
// case we ever revisit, but the provider is hard-locked to dark and the
// toggleMode is a no-op (kept so existing call sites don't break).
export function ThemeProvider({children}: {children: React.ReactNode}) {
    const mode: ThemeMode = 'dark';
    const colors = getColors(mode);

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

    return (
        <ThemeContext.Provider value={{mode, toggleMode, colors}}>
            {children}
        </ThemeContext.Provider>
    );
}

export function useThemeMode() {
    return useContext(ThemeContext);
}
