import React from 'react'
import ReactDOM from 'react-dom/client'
import {createHashRouter, RouterProvider,} from "react-router-dom";
import AppShell from "./components/AppShell.tsx";
import {App, ConfigProvider, theme} from "antd";
import {Spinner} from "./components/Spinner.tsx";
import {ThemeProvider, useThemeMode} from "./theme/ThemeContext.tsx";
import {NotificationCenterProvider} from "./hooks/useNotificationCenter.tsx";

// Lazy-load each page so the first paint only ships the shell + the route
// the user actually opens. Everything else streams in on demand.
const SettingsPage     = React.lazy(() => import("./pages/SettingsPage.tsx"));
const LogsPage         = React.lazy(() => import("./pages/LogsPage.tsx"));
const MowgliNextPage   = React.lazy(() => import("./pages/MowgliNextPage.tsx"));
const MapPage          = React.lazy(() => import("./pages/MapPage.tsx"));
const OnboardingPage   = React.lazy(() => import("./pages/OnboardingPage.tsx"));
const SchedulePage     = React.lazy(() => import("./pages/SchedulePage.tsx"));
const DiagnosticsPage  = React.lazy(() => import("./pages/DiagnosticsPage.tsx"));
const StatisticsPage   = React.lazy(() => import("./pages/StatisticsPage.tsx"));
const ConceptRoot      = React.lazy(() => import("./concept/ConceptRoot.tsx"));

const router = createHashRouter([
    {
        // Standalone premium concept -- lives outside the AntD chrome so
        // its tokens + CSS don't fight with the operator app.
        path: "/concept",
        element: <ConceptRoot/>,
    },
    {
        path: "/",
        element: <AppShell/>,
        children: [
            {
                element: <SettingsPage/>,
                path: "/settings",
            },
            {
                element: <LogsPage/>,
                path: "/logs",
            },
            {
                element: <MowgliNextPage/>,
                path: "/mowglinext",
            },
            {
                element: <MapPage/>,
                path: "/map",
            },
            {
                element: <OnboardingPage/>,
                path: "/onboarding",
            },
            {
                element: <SchedulePage/>,
                path: "/schedule",
            },
            {
                element: <DiagnosticsPage/>,
                path: "/diagnostics",
            },
            {
                element: <StatisticsPage/>,
                path: "/statistics",
            }
        ]
    },
]);

function ThemedApp() {
    const {colors} = useThemeMode();

    return (
        <ConfigProvider theme={{
            algorithm: theme.darkAlgorithm,
            token: {
                colorPrimary: colors.primary,
                colorSuccess: colors.success,
                colorWarning: colors.warning,
                colorError: colors.danger,
                colorInfo: colors.info,
                colorBgContainer: colors.bgCard,
                colorBgLayout: colors.bgBase,
                colorBorder: colors.border,
                colorText: colors.text,
                colorTextSecondary: colors.textSecondary,
                borderRadius: 12,
                fontFamily: '"Satoshi", "Inter", -apple-system, BlinkMacSystemFont, sans-serif',
                fontFamilyCode: '"Space Grotesk", "JetBrains Mono", ui-monospace, monospace',
            },
            components: {
                Card: {
                    colorBorderSecondary: 'transparent',
                },
                Button: {
                    borderRadius: 8,
                },
                Input: {
                    colorBgContainer: colors.bgElevated,
                    activeBorderColor: colors.accent,
                    hoverBorderColor: colors.accent,
                },
                Select: {
                    colorBgContainer: colors.bgElevated,
                },
            },
        }}>
            <App style={{height: "100%"}}>
                <NotificationCenterProvider>
                    <React.Suspense fallback={<Spinner/>}>
                        <RouterProvider router={router}/>
                    </React.Suspense>
                </NotificationCenterProvider>
            </App>
        </ConfigProvider>
    );
}

ReactDOM.createRoot(document.getElementById('root')!).render(
  <React.StrictMode>
      <ThemeProvider>
          <ThemedApp/>
      </ThemeProvider>
  </React.StrictMode>,
)
