import {createContext, useCallback, useContext, useEffect} from "react";
import type {ThemeMode} from "./colors.ts";
import {getColors, setColors} from "./colors.ts";

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
        document.documentElement.style.background = colors.bgBase;
        document.body.style.background = colors.bgBase;
        document.body.style.fontFamily = "'Satoshi', 'Inter', -apple-system, BlinkMacSystemFont, 'Helvetica Neue', sans-serif";
        document.documentElement.style.colorScheme = 'dark';
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
