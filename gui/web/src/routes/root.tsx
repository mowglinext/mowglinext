import {Outlet, useMatches, useNavigate} from "react-router-dom";
import {Layout} from "antd";
import React, {useCallback, useEffect, useState} from "react";
import {AnimatePresence, motion} from "framer-motion";
import {MowerStatus} from "../components/MowerStatus.tsx";
import {LiveStatusStrip} from "../components/LiveStatusStrip.tsx";
import {NotificationBell} from "../components/NotificationBell.tsx";
import {useAutoNotifications} from "../hooks/useNotificationCenter.tsx";
import {useHighLevelStatus} from "../hooks/useHighLevelStatus.ts";
import {useEmergency} from "../hooks/useEmergency.ts";
import {useStatus} from "../hooks/useStatus.ts";
import {useIsMobile} from "../hooks/useIsMobile";
import {useIOSInstallPrompt} from "../hooks/useIOSInstallPrompt";
import {IOSInstallBanner} from "../components/IOSInstallBanner.tsx";
import {useThemeMode} from "../theme/ThemeContext.tsx";
import {
  IconMower, IconMap, IconSchedule, IconStats, IconLogs, IconDiag,
  IconGear, IconRocket, FONT, KEYFRAMES_CSS,
} from "../components/dashboard";

interface NavItem {
  key: string;
  label: string;
  icon: React.ReactNode;
  subtitle?: string;
}

const navItems: NavItem[] = [
  {key: '/mowglinext', label: 'Dashboard', icon: <IconMower size={20}/>, subtitle: 'Live overview'},
  {key: '/map', label: 'Map', icon: <IconMap size={20}/>, subtitle: 'Zones & boundaries'},
  {key: '/schedule', label: 'Schedule', icon: <IconSchedule size={20}/>, subtitle: 'Weekly plan'},
  {key: '/onboarding', label: 'Onboarding', icon: <IconRocket size={20}/>},
  {key: '/settings', label: 'Settings', icon: <IconGear size={20}/>},
  {key: '/logs', label: 'Logs', icon: <IconLogs size={20}/>},
  {key: '/diagnostics', label: 'Diagnostics', icon: <IconDiag size={20}/>},
  {key: '/statistics', label: 'Statistics', icon: <IconStats size={20}/>, subtitle: 'Lifetime data'},
];

const bottomNavItems: NavItem[] = [
  {key: '/mowglinext', label: 'Home', icon: <IconMower size={20}/>},
  {key: '/map', label: 'Map', icon: <IconMap size={20}/>},
  {key: '/statistics', label: 'Stats', icon: <IconStats size={20}/>},
  {key: '/settings', label: 'Settings', icon: <IconGear size={20}/>},
];

const pageTitles: Record<string, string> = {
  '/mowglinext': 'Dashboard',
  '/onboarding': 'Onboarding',
  '/settings': 'Settings',
  '/map': 'Map',
  '/schedule': 'Schedule',
  '/logs': 'Logs',
  '/diagnostics': 'Diagnostics',
  '/statistics': 'Statistics',
};

const pageSubtitles: Record<string, string> = {
  '/mowglinext': 'Live overview',
  '/map': 'Zones & boundaries',
  '/schedule': 'Weekly plan',
  '/statistics': 'Lifetime data',
};

const PIN_STORAGE_KEY = 'mowglinext-sidebar-pinned';

function getInitialPinned(): boolean {
  try {
    const stored = localStorage.getItem(PIN_STORAGE_KEY);
    if (stored === 'true' || stored === 'false') return stored === 'true';
  } catch { /* ignore */ }
  // Default to collapsed -- users opt in to pinning via the toggle once the
  // sidebar is open. Avoids surprising layout changes for existing users and
  // keeps the test harness's default-collapsed assertion intact.
  return false;
}

export default function Root() {
  const {colors} = useThemeMode();
  const route = useMatches();
  const navigate = useNavigate();
  const isMobile = useIsMobile();
  const [sidebarOpen, setSidebarOpen] = useState(false);
  const [pinned, setPinned] = useState(getInitialPinned);
  const [hovering, setHovering] = useState(false);
  const railExpanded = pinned || hovering;
  const {showPrompt: showInstallPrompt, dismiss: dismissInstallPrompt} = useIOSInstallPrompt();

  useEffect(() => {
    try { localStorage.setItem(PIN_STORAGE_KEY, pinned ? 'true' : 'false'); } catch { /* ignore */ }
  }, [pinned]);

  const {highLevelStatus} = useHighLevelStatus();
  const emergency = useEmergency();
  const hwStatus = useStatus();
  useAutoNotifications({
    emergencyActive: highLevelStatus.emergency ?? emergency.active_emergency ?? false,
    emergencyLatched: emergency.latched_emergency ?? false,
    rainDetected: hwStatus.rain_detected ?? false,
    state: highLevelStatus.state_name,
  });

  const [configChecked, setConfigChecked] = useState(false);
  useEffect(() => {
    if (configChecked) return;
    (async () => {
      try {
        const base = import.meta.env.DEV
          ? `http://${(import.meta.env.VITE_API_HOST as string | undefined) ?? 'localhost:4006'}`
          : '';
        const res = await fetch(`${base}/api/settings/status`);
        const data = await res.json();
        if (!data.onboarding_completed && currentPath !== '/onboarding') {
          navigate({pathname: '/onboarding'});
        }
      } catch { /* ignore */ }
      setConfigChecked(true);
    })();
  }, [configChecked]);

  useEffect(() => {
    if (route.length === 1 && route[0].pathname === "/") {
      navigate({pathname: '/mowglinext'});
    }
  }, [route, navigate]);

  const currentPath = route.length > 1 ? route[1].pathname : '/mowglinext';
  const pageTitle = pageTitles[currentPath] ?? 'MowgliNext';
  const pageSubtitle = pageSubtitles[currentPath];

  const handleNavigate = useCallback((key: string) => {
    navigate({pathname: key});
    setSidebarOpen(false);
  }, [navigate]);

  if (isMobile) {
    return (
      <div style={{
        display: 'flex',
        flexDirection: 'column',
        height: '100%',
        background: colors.bgBase,
        overflow: 'hidden',
        fontFamily: FONT,
      }}>
        <style>{KEYFRAMES_CSS}</style>
        <LiveStatusStrip/>
        {/* Mobile Header */}
        <header style={{
          display: 'flex',
          alignItems: 'center',
          justifyContent: 'space-between',
          padding: '0 16px',
          paddingTop: 'env(safe-area-inset-top, 0px)',
          background: colors.bgBase,
          borderBottom: `1px solid ${colors.border}`,
          minHeight: 56,
          flexShrink: 0,
          zIndex: 100,
        }}>
          <div style={{display: 'flex', alignItems: 'center', gap: 10, minWidth: 0, flex: 1}}>
            <button
              onClick={() => setSidebarOpen(!sidebarOpen)}
              aria-label={sidebarOpen ? 'Close menu' : 'Open menu'}
              style={{
                background: 'none', border: 'none', color: colors.text,
                fontSize: 18, padding: 4, cursor: 'pointer', flexShrink: 0,
                display: 'flex', alignItems: 'center',
              }}
            >
              {sidebarOpen ? (
                <svg viewBox="0 0 24 24" width={20} height={20} fill="none" stroke="currentColor" strokeWidth={2} strokeLinecap="round"><path d="M18 6L6 18M6 6l12 12"/></svg>
              ) : (
                <svg viewBox="0 0 24 24" width={20} height={20} fill="none" stroke="currentColor" strokeWidth={2} strokeLinecap="round"><path d="M3 7h18M3 12h18M3 17h18"/></svg>
              )}
            </button>
            <div>
              <div className="mn-display" style={{fontSize: 22, color: colors.text, letterSpacing: '-0.01em', lineHeight: 1.1}}>
                {pageTitle}
              </div>
              {pageSubtitle && (
                <div style={{fontSize: 12, color: colors.textDim}}>{pageSubtitle}</div>
              )}
            </div>
          </div>
          <div style={{display: 'flex', alignItems: 'center', gap: 4}}>
            <NotificationBell/>
            <MowerStatus/>
          </div>
        </header>

        {/* Slide-over Backdrop */}
        <div
          onClick={() => setSidebarOpen(false)}
          style={{
            position: 'fixed', inset: 0, top: 56,
            background: 'rgba(0,0,0,0.5)', zIndex: 199,
            opacity: sidebarOpen ? 1 : 0,
            pointerEvents: sidebarOpen ? 'auto' : 'none',
            transition: 'opacity 0.25s ease-out',
          }}
        />
        {/* Slide-over Nav */}
        <nav style={{
          position: 'fixed', top: 56, left: 0, bottom: 56,
          width: 260, background: colors.bgCard, zIndex: 200,
          borderRight: `1px solid ${colors.border}`,
          overflowY: 'auto',
          transform: sidebarOpen ? 'translateX(0)' : 'translateX(-100%)',
          transition: 'transform 0.25s ease-out',
        }}>
          <div style={{padding: '8px 0'}}>
            {navItems.map(item => {
              const isActive = currentPath === item.key;
              return (
                <button
                  key={item.key}
                  onClick={() => handleNavigate(item.key)}
                  style={{
                    display: 'flex',
                    alignItems: 'center',
                    gap: 12,
                    width: '100%',
                    padding: '12px 20px',
                    background: isActive ? colors.accentSoft : 'transparent',
                    border: 'none',
                    borderLeft: isActive ? `3px solid ${colors.accent}` : '3px solid transparent',
                    color: isActive ? colors.accent : colors.text,
                    fontSize: 15,
                    cursor: 'pointer',
                    fontFamily: FONT,
                    transition: 'background 0.15s, color 0.15s',
                  }}
                >
                  <span style={{display: 'flex', alignItems: 'center'}}>{item.icon}</span>
                  {item.label}
                </button>
              );
            })}
          </div>
        </nav>

        {/* Content */}
        <main style={{flex: 1, overflow: 'auto', padding: '8px 12px 0', minHeight: 0}}>
          <AnimatePresence mode="wait">
            <motion.div
              key={currentPath}
              initial={{opacity: 0, y: 8}}
              animate={{opacity: 1, y: 0}}
              exit={{opacity: 0, y: -4}}
              transition={{duration: 0.18, ease: 'easeOut'}}
              style={{height: '100%'}}
            >
              <Outlet/>
            </motion.div>
          </AnimatePresence>
        </main>

        {/* Bottom Tab Bar */}
        <nav style={{
          display: 'flex',
          alignItems: 'stretch',
          background: colors.bgBase,
          borderTop: `1px solid ${colors.border}`,
          minHeight: 56,
          paddingBottom: 'env(safe-area-inset-bottom, 0px)',
          flexShrink: 0,
          zIndex: 100,
        }}>
          {bottomNavItems.map(item => {
            const isActive = currentPath === item.key;
            return (
              <button
                key={item.key}
                onClick={() => handleNavigate(item.key)}
                aria-label={item.label}
                aria-current={isActive ? 'page' : undefined}
                style={{
                  flex: 1,
                  display: 'flex',
                  flexDirection: 'column',
                  alignItems: 'center',
                  justifyContent: 'center',
                  gap: 3,
                  background: 'none',
                  border: 'none',
                  cursor: 'pointer',
                  color: isActive ? colors.accent : colors.textMuted,
                  borderTop: isActive ? `2px solid ${colors.accent}` : '2px solid transparent',
                  transition: 'color 0.2s, border-color 0.2s',
                  padding: 0,
                  fontFamily: FONT,
                }}
              >
                {item.icon}
                <span style={{fontSize: 10, fontWeight: isActive ? 600 : 500, lineHeight: 1}}>
                  {item.label}
                </span>
              </button>
            );
          })}
        </nav>
        {showInstallPrompt && <IOSInstallBanner onDismiss={dismissInstallPrompt}/>}
      </div>
    );
  }

  // Desktop layout
  return (
    <div style={{
      display: 'flex', height: '100%', minHeight: '100%', maxHeight: '100%',
      overflow: 'hidden', fontFamily: FONT,
    }}>
      <style>{KEYFRAMES_CSS}</style>
      <nav
        onMouseEnter={() => setHovering(true)}
        onMouseLeave={() => setHovering(false)}
        style={{
          width: railExpanded ? 200 : 60,
          minWidth: railExpanded ? 200 : 60,
          background: colors.bgCard,
          borderRight: `1px solid ${colors.border}`,
          display: 'flex',
          flexDirection: 'column',
          transition: 'width 0.2s ease, min-width 0.2s ease',
          overflow: 'hidden',
          flexShrink: 0,
          height: '100%',
        }}
      >
        {/* Logo area */}
        <div style={{
          display: 'flex',
          alignItems: 'center',
          justifyContent: railExpanded ? 'flex-start' : 'center',
          gap: 10,
          padding: railExpanded ? '20px 16px' : '20px 0',
          borderBottom: `1px solid ${colors.borderSubtle}`,
          overflow: 'hidden',
          height: 64,
          flexShrink: 0,
        }}>
          <div style={{
            width: 32, height: 32, borderRadius: 9, flexShrink: 0,
            background: `linear-gradient(135deg, ${colors.accent}, ${colors.accent}aa)`,
            display: 'flex', alignItems: 'center', justifyContent: 'center',
            color: '#0a0f0b',
          }}>
            <IconMower size={18}/>
          </div>
          {railExpanded && (
            <>
              <span className="mn-display" style={{
                fontSize: 26, color: colors.text, whiteSpace: 'nowrap',
                flex: 1, lineHeight: 1, letterSpacing: '-0.01em',
              }}>
                Mowgli<em style={{color: colors.accent}}>Next</em>
              </span>
              <button
                onClick={() => setPinned(p => !p)}
                aria-label={pinned ? 'Unpin sidebar' : 'Pin sidebar'}
                title={pinned ? 'Unpin sidebar' : 'Pin sidebar'}
                style={{
                  background: 'transparent', border: 'none', cursor: 'pointer',
                  padding: 4, color: pinned ? colors.accent : colors.textMuted,
                  display: 'flex', alignItems: 'center',
                }}
              >
                <svg viewBox="0 0 24 24" width={14} height={14} fill="none" stroke="currentColor" strokeWidth={2} strokeLinecap="round" strokeLinejoin="round">
                  {pinned ? (
                    <path d="M12 17v5M9 10.5V3h6v7.5l3 3.5H6l3-3.5z"/>
                  ) : (
                    <path d="M12 17v5M9 10.5V3h6v7.5l3 3.5H6l3-3.5zM3 3l18 18"/>
                  )}
                </svg>
              </button>
            </>
          )}
        </div>

        {/* Nav items */}
        <div style={{flex: 1, padding: '8px 0', overflowY: 'auto'}}>
          {navItems.map(item => {
            const isActive = currentPath === item.key;
            return (
              <button
                key={item.key}
                onClick={() => handleNavigate(item.key)}
                aria-label={item.label}
                style={{
                  display: 'flex',
                  alignItems: 'center',
                  gap: 12,
                  width: '100%',
                  padding: '10px 0',
                  paddingLeft: railExpanded ? 16 : 0,
                  justifyContent: railExpanded ? 'flex-start' : 'center',
                  background: isActive ? colors.accentSoft : 'transparent',
                  border: 'none',
                  borderLeft: isActive ? `3px solid ${colors.accent}` : '3px solid transparent',
                  color: isActive ? colors.accent : colors.text,
                  fontSize: 14,
                  cursor: 'pointer',
                  fontFamily: FONT,
                  transition: 'background 0.15s, color 0.15s, padding-left 0.2s ease',
                  overflow: 'hidden',
                  whiteSpace: 'nowrap',
                }}
                onMouseOver={(e) => {
                  if (!isActive) e.currentTarget.style.background = colors.bgElevated;
                }}
                onMouseOut={(e) => {
                  if (!isActive) e.currentTarget.style.background = 'transparent';
                }}
              >
                <span style={{
                  flexShrink: 0, width: 28, display: 'inline-flex',
                  alignItems: 'center', justifyContent: 'center',
                }}>
                  {item.icon}
                </span>
                {railExpanded && <span>{item.label}</span>}
              </button>
            );
          })}
        </div>

      </nav>

      {/* Main content */}
      <div style={{flex: 1, display: 'flex', flexDirection: 'column', height: '100%', overflow: 'hidden', minWidth: 0}}>
        <LiveStatusStrip/>
        <Layout.Header style={{
          display: 'flex',
          alignItems: 'center',
          justifyContent: 'space-between',
          gap: 12,
          padding: '0 24px',
          background: colors.bgBase,
          borderBottom: `1px solid ${colors.border}`,
          height: 56,
          lineHeight: 'normal',
          overflow: 'visible',
          flexShrink: 0,
          position: 'relative',
          zIndex: 20,
        }}>
          <div>
            <div className="mn-display" style={{
              fontSize: 24, color: colors.text,
              letterSpacing: '-0.01em', lineHeight: 1.1,
              whiteSpace: 'nowrap', overflow: 'hidden', textOverflow: 'ellipsis',
            }}>
              {pageTitle}
            </div>
            {pageSubtitle && (
              <div style={{fontSize: 12, color: colors.textMuted}}>{pageSubtitle}</div>
            )}
          </div>
          <div style={{display: 'flex', alignItems: 'center', gap: 4}}>
            <NotificationBell/>
            <MowerStatus/>
          </div>
        </Layout.Header>
        <main style={{flex: 1, padding: '20px 24px 0', overflow: 'auto', minHeight: 0, background: colors.bgBase}}>
          <AnimatePresence mode="wait">
            <motion.div
              key={currentPath}
              initial={{opacity: 0, y: 6}}
              animate={{opacity: 1, y: 0}}
              exit={{opacity: 0, y: -4}}
              transition={{duration: 0.18, ease: 'easeOut'}}
              style={{height: '100%'}}
            >
              <Outlet/>
            </motion.div>
          </AnimatePresence>
        </main>
      </div>
    </div>
  );
}
