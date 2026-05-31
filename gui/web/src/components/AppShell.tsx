import {type ReactNode, useEffect, useMemo, useState} from "react";
import {Outlet, useMatches, useNavigate} from "react-router-dom";
import {AnimatePresence, motion, LayoutGroup} from "framer-motion";
import {
  Home, Map as MapIcon, Calendar, Compass, Settings, Terminal, Rocket, Activity,
} from "lucide-react";

import {MowerStatus} from "./MowerStatus.tsx";
import {NotificationBell} from "./NotificationBell.tsx";
import {LiveStatusStrip} from "./LiveStatusStrip.tsx";
import {useAutoNotifications} from "../hooks/useNotificationCenter.tsx";
import {useHighLevelStatus} from "../hooks/useHighLevelStatus.ts";
import {useEmergency} from "../hooks/useEmergency.ts";
import {useStatus} from "../hooks/useStatus.ts";
import {useIsMobile} from "../hooks/useIsMobile";
import {useThemeMode} from "../theme/ThemeContext.tsx";
import {KEYFRAMES_CSS} from "./dashboard";
import "../concept/concept.css";

/**
 * Premium tech-garden shell shared by the whole app.
 *
 * Desktop -> 88px fixed glass side-rail on the left + the page content in a
 *            comfortable max-width column with a sticky top status strip.
 * Mobile   -> glass bottom-nav with a sliding lime pill + slim top header.
 *
 * All surfaces inherit the /concept tokens (data-concept scope on body).
 */

interface NavItem {
  key: string;          // path
  label: string;
  shortLabel?: string;  // for the bottom-nav
  icon: typeof Home;
  showInBottom?: boolean;
}

const NAV: NavItem[] = [
  {key: '/mowglinext',  label: 'Accueil',     shortLabel: 'Accueil',  icon: Home,     showInBottom: true},
  {key: '/map',         label: 'Carte',                                icon: MapIcon,  showInBottom: true},
  {key: '/schedule',    label: 'Planning',    shortLabel: 'Planning',  icon: Calendar, showInBottom: true},
  {key: '/diagnostics', label: 'Diagnostic',  shortLabel: 'Diag',      icon: Activity, showInBottom: true},
  {key: '/statistics',  label: 'Stats',                                icon: Compass,  showInBottom: false},
  {key: '/settings',    label: 'Réglages',                             icon: Settings, showInBottom: false},
  {key: '/logs',        label: 'Logs',                                 icon: Terminal, showInBottom: false},
  {key: '/onboarding',  label: 'Onboarding',                           icon: Rocket,   showInBottom: false},
];

const PAGE_META: Record<string, {title: string; subtitle?: string}> = {
  '/mowglinext':  {title: 'Accueil',        subtitle: 'Vue en direct'},
  '/map':         {title: 'Carte',          subtitle: 'Zones et trajectoire'},
  '/schedule':    {title: 'Planning',       subtitle: 'Semaine type'},
  '/diagnostics': {title: 'Diagnostic',     subtitle: 'Capteurs et état système'},
  '/statistics':  {title: 'Statistiques',   subtitle: 'Historique long terme'},
  '/settings':    {title: 'Réglages',       subtitle: 'Configuration robot'},
  '/logs':        {title: 'Logs',           subtitle: 'Sortie des conteneurs'},
  '/onboarding':  {title: 'Onboarding'},
};

export function AppShell() {
  const {colors} = useThemeMode();
  const navigate = useNavigate();
  const route = useMatches();
  const isMobile = useIsMobile();

  const currentPath = route.length > 1 ? route[1].pathname : '/mowglinext';
  const meta = PAGE_META[currentPath] ?? {title: 'MowgliNext'};

  // Empty path -> dashboard. Without this `/` renders the shell with an
  // empty Outlet, which looks broken (the previous Root had this redirect
  // and we lost it in the AppShell rewrite).
  useEffect(() => {
    if (route.length === 1 && route[0].pathname === '/') {
      navigate({pathname: '/mowglinext'}, {replace: true});
    }
  }, [route, navigate]);

  // Onboarding gate (kept from the previous Root)
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
  }, [configChecked, currentPath, navigate]);

  // Auto-notifications hook (BT-state derived push notifications)
  const {highLevelStatus} = useHighLevelStatus();
  const emergency = useEmergency();
  const hwStatus = useStatus();
  useAutoNotifications({
    emergencyActive: highLevelStatus.emergency ?? emergency.active_emergency ?? false,
    emergencyLatched: emergency.latched_emergency ?? false,
    rainDetected: hwStatus.rain_detected ?? false,
    state: highLevelStatus.state_name,
  });

  // Bottom-nav items
  const bottomItems = useMemo(() => NAV.filter(n => n.showInBottom), []);

  if (isMobile) {
    return (
      <div data-concept style={{
        display: 'flex', flexDirection: 'column',
        height: '100%', background: colors.bgBase, overflow: 'hidden',
      }}>
        <style>{KEYFRAMES_CSS}</style>
        <AuroraBackdrop/>
        <LiveStatusStrip/>

        <header style={{
          display: 'flex', alignItems: 'center', justifyContent: 'space-between',
          padding: '0 16px',
          paddingTop: 'max(env(safe-area-inset-top, 0px), 6px)',
          minHeight: 56,
          background: 'rgba(2, 17, 13, 0.6)',
          backdropFilter: 'blur(20px) saturate(140%)',
          borderBottom: `1px solid ${colors.borderSubtle}`,
          flexShrink: 0,
          position: 'relative', zIndex: 10,
        }}>
          <div>
            <div className="mn-display" style={{
              fontSize: 22, fontWeight: 400, color: colors.text,
              letterSpacing: '-0.01em', lineHeight: 1.1,
            }}>
              {meta.title}
            </div>
            {meta.subtitle && (
              <div style={{fontSize: 11, color: 'rgba(236, 255, 244, 0.42)', marginTop: 1}}>
                {meta.subtitle}
              </div>
            )}
          </div>
          <div style={{display: 'flex', alignItems: 'center', gap: 6}}>
            <NotificationBell/>
            <MowerStatus/>
          </div>
        </header>

        <main style={{
          flex: 1, overflow: 'auto', minHeight: 0,
          padding: '12px 14px 110px',
          position: 'relative', zIndex: 1,
        }}>
          <AnimatedOutlet currentPath={currentPath}/>
        </main>

        <MobileBottomNav
          items={bottomItems}
          activePath={currentPath}
          onNavigate={(k) => navigate({pathname: k})}
        />
      </div>
    );
  }

  // ─── Desktop ───
  return (
    <div data-concept style={{
      display: 'flex',
      height: '100%', minHeight: '100%', overflow: 'hidden',
      background: colors.bgBase,
      position: 'relative',
    }}>
      <style>{KEYFRAMES_CSS}</style>
      <AuroraBackdrop/>

      <DesktopSideRail
        items={NAV}
        activePath={currentPath}
        onNavigate={(k) => navigate({pathname: k})}
      />

      <div style={{
        flex: 1, minWidth: 0, height: '100%',
        display: 'flex', flexDirection: 'column',
        marginLeft: 88,
        position: 'relative', zIndex: 1,
      }}>
        <LiveStatusStrip/>
        <header style={{
          display: 'flex', alignItems: 'center', justifyContent: 'space-between',
          padding: '18px 32px',
          background: 'rgba(2, 17, 13, 0.45)',
          backdropFilter: 'blur(20px) saturate(140%)',
          borderBottom: `1px solid ${colors.borderSubtle}`,
          position: 'sticky', top: 0, zIndex: 30, overflow: 'visible',
        }}>
          <div>
            <div className="mn-display" style={{
              fontSize: 28, fontWeight: 400, color: colors.text,
              letterSpacing: '-0.015em', lineHeight: 1.05,
            }}>
              {meta.title}
            </div>
            {meta.subtitle && (
              <div style={{fontSize: 12, color: 'rgba(236, 255, 244, 0.42)', marginTop: 2}}>
                {meta.subtitle}
              </div>
            )}
          </div>
          <div style={{display: 'flex', alignItems: 'center', gap: 12}}>
            <NotificationBell/>
            <MowerStatus/>
          </div>
        </header>
        <main style={{flex: 1, overflow: 'auto', minHeight: 0, padding: '24px 32px 48px'}}>
          <AnimatedOutlet currentPath={currentPath}/>
        </main>
      </div>
    </div>
  );
}

// ─────────────────────────────────────────────────────────────────────
// Sub-shells
// ─────────────────────────────────────────────────────────────────────

function AuroraBackdrop() {
  return (
    <div aria-hidden style={{
      position: 'fixed', inset: -100, pointerEvents: 'none',
      background:
        "radial-gradient(circle at 18% 14%, rgba(69, 214, 232, 0.18) 0%, transparent 38%)," +
        "radial-gradient(circle at 80% 80%, rgba(124, 255, 178, 0.22) 0%, transparent 42%)," +
        "radial-gradient(circle at 50% 50%, rgba(107, 127, 255, 0.05) 0%, transparent 65%)",
      filter: 'blur(8px)', zIndex: 0,
    }}/>
  );
}

function AnimatedOutlet({currentPath}: {currentPath: string}) {
  return (
    <AnimatePresence mode="wait">
      <motion.div
        key={currentPath}
        initial={{opacity: 0, y: 10}}
        animate={{opacity: 1, y: 0}}
        exit={{opacity: 0, y: -6}}
        transition={{duration: 0.28, ease: [0.2, 0.7, 0.2, 1]}}
        style={{height: '100%'}}
      >
        <Outlet/>
      </motion.div>
    </AnimatePresence>
  );
}

// ─── Side-rail ───
interface RailProps {
  items: NavItem[];
  activePath: string;
  onNavigate: (k: string) => void;
}

function DesktopSideRail({items, activePath, onNavigate}: RailProps) {
  return (
    <aside style={{
      position: 'fixed', top: 0, bottom: 0, left: 0, width: 88,
      display: 'flex', flexDirection: 'column',
      paddingTop: 24, paddingBottom: 24,
      background: 'linear-gradient(180deg, rgba(2, 17, 13, 0.92), rgba(2, 17, 13, 0.84))',
      borderRight: '1px solid rgba(236, 255, 244, 0.07)',
      backdropFilter: 'blur(22px) saturate(140%)',
      zIndex: 40,
    }}>
      <div style={{
        display: 'flex', flexDirection: 'column', alignItems: 'center', gap: 6,
        marginBottom: 24,
      }}>
        <div style={{
          width: 44, height: 44, borderRadius: 14,
          background: 'linear-gradient(135deg, #7CFFB2 0%, #45D688 50%, #2BAA66 100%)',
          color: '#02110D',
          display: 'flex', alignItems: 'center', justifyContent: 'center',
          fontFamily: 'Satoshi', fontWeight: 900, fontSize: 22, lineHeight: 1,
          boxShadow: '0 10px 24px -8px rgba(124, 255, 178, 0.5)',
        }}>
          m
        </div>
        <div style={{
          fontSize: 9, color: 'rgba(236, 255, 244, 0.42)',
          letterSpacing: '0.18em', textTransform: 'uppercase', fontWeight: 700,
        }}>
          Mowgli
        </div>
      </div>

      <LayoutGroup>
        <nav style={{display: 'flex', flexDirection: 'column', gap: 4, padding: '0 12px', flex: 1, overflowY: 'auto'}}>
          {items.map(({key, label, icon: Icon}) => {
            const isActive = key === activePath;
            return (
              <button
                key={key}
                onClick={() => onNavigate(key)}
                aria-label={label}
                aria-current={isActive ? 'page' : undefined}
                style={{
                  position: 'relative',
                  display: 'flex', flexDirection: 'column',
                  alignItems: 'center', justifyContent: 'center', gap: 4,
                  padding: '12px 4px 10px',
                  borderRadius: 14,
                  background: 'transparent',
                  border: 'none', cursor: 'pointer',
                  color: isActive ? '#02110D' : 'rgba(236, 255, 244, 0.62)',
                  fontSize: 9, fontWeight: 700,
                  letterSpacing: '0.06em', textTransform: 'uppercase',
                  zIndex: 1,
                  transition: 'color 0.15s',
                }}
              >
                {isActive && (
                  <motion.span
                    layoutId="app-rail-pill"
                    style={{
                      position: 'absolute', inset: 0,
                      background: 'linear-gradient(135deg, #7CFFB2 0%, #45D688 50%, #2BAA66 100%)',
                      borderRadius: 14,
                      boxShadow: '0 12px 26px -6px rgba(124, 255, 178, 0.5), inset 0 1px 0 rgba(255, 255, 255, 0.32)',
                      zIndex: -1,
                    }}
                    transition={{type: 'spring', stiffness: 380, damping: 32}}
                  />
                )}
                <Icon size={18} strokeWidth={isActive ? 2.4 : 2}/>
                <span>{label}</span>
              </button>
            );
          })}
        </nav>
      </LayoutGroup>
    </aside>
  );
}

// ─── Mobile bottom nav ───
function MobileBottomNav({items, activePath, onNavigate}: RailProps) {
  return (
    <nav style={{
      position: 'fixed', left: 0, right: 0, bottom: 0,
      paddingBottom: 'calc(env(safe-area-inset-bottom, 0px) + 10px)',
      paddingTop: 10,
      paddingLeft: 14, paddingRight: 14,
      background: 'linear-gradient(180deg, rgba(2, 17, 13, 0) 0%, rgba(2, 17, 13, 0.85) 30%, rgba(2, 17, 13, 0.97) 100%)',
      backdropFilter: 'blur(22px) saturate(140%)',
      zIndex: 50,
    }}>
      <LayoutGroup>
        <div style={{
          display: 'grid',
          gridTemplateColumns: `repeat(${items.length}, 1fr)`,
          gap: 2, padding: 6,
          background: 'rgba(255, 255, 255, 0.04)',
          border: '1px solid rgba(236, 255, 244, 0.08)',
          borderRadius: 999,
          backdropFilter: 'blur(28px)',
        }}>
          {items.map(({key, label, shortLabel, icon: Icon}) => {
            const isActive = key === activePath;
            return (
              <button
                key={key}
                onClick={() => onNavigate(key)}
                aria-label={label}
                style={{
                  position: 'relative',
                  display: 'flex', flexDirection: 'column',
                  alignItems: 'center', justifyContent: 'center', gap: 2,
                  padding: '10px 4px 8px',
                  borderRadius: 999,
                  background: 'transparent', border: 'none', cursor: 'pointer',
                  color: isActive ? '#02110D' : 'rgba(236, 255, 244, 0.66)',
                  fontSize: 10, fontWeight: 600,
                  letterSpacing: '0.02em',
                  zIndex: 1,
                  transition: 'color 0.15s',
                }}
              >
                {isActive && (
                  <motion.span
                    layoutId="app-bottom-pill"
                    style={{
                      position: 'absolute', inset: 0,
                      background: 'linear-gradient(135deg, #7CFFB2 0%, #45D688 50%, #2BAA66 100%)',
                      borderRadius: 999,
                      boxShadow: '0 6px 20px -6px rgba(124, 255, 178, 0.55), inset 0 1px 0 rgba(255, 255, 255, 0.3)',
                      zIndex: -1,
                    }}
                    transition={{type: 'spring', stiffness: 380, damping: 32}}
                  />
                )}
                <Icon size={18} strokeWidth={isActive ? 2.4 : 2}/>
                <span>{shortLabel ?? label}</span>
              </button>
            );
          })}
        </div>
      </LayoutGroup>
    </nav>
  );
}

export default AppShell;

// Type alias for callers that previously imported Root.
export {AppShell as Root};
export type {ReactNode};
