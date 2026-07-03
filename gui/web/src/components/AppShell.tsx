import {type ReactNode, useEffect, useMemo, useState} from "react";
import {Outlet, useMatches, useNavigate} from "react-router-dom";
import {AnimatePresence, motion, LayoutGroup} from "framer-motion";
import {
  Home, Map as MapIcon, Calendar, Compass, Settings, Terminal, Rocket, Activity,
  MoreHorizontal, X, SlidersHorizontal,
} from "lucide-react";

import {useTranslation} from "react-i18next";

import {MowerStatus} from "./MowerStatus.tsx";
import {NotificationBell} from "./NotificationBell.tsx";
import {LanguageSwitcher} from "./LanguageSwitcher.tsx";
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
  key: string;            // path
  labelKey: string;       // i18n key
  shortLabelKey?: string; // for the bottom-nav
  icon: typeof Home;
  showInBottom?: boolean;
}

const NAV: NavItem[] = [
  {key: '/mowglinext',  labelKey: 'nav.home',        shortLabelKey: 'nav.home',      icon: Home,     showInBottom: true},
  {key: '/map',         labelKey: 'nav.map',                                          icon: MapIcon,  showInBottom: true},
  {key: '/schedule',    labelKey: 'nav.schedule',    shortLabelKey: 'nav.schedule',  icon: Calendar, showInBottom: true},
  {key: '/diagnostics', labelKey: 'nav.diagnostics', shortLabelKey: 'nav.diagShort', icon: Activity, showInBottom: true},
  {key: '/statistics',  labelKey: 'nav.stats',                                        icon: Compass,  showInBottom: false},
  {key: '/settings',    labelKey: 'nav.settings',                                     icon: Settings, showInBottom: false},
  {key: '/parameters',  labelKey: 'nav.parameters',                                   icon: SlidersHorizontal, showInBottom: false},
  {key: '/logs',        labelKey: 'nav.logs',                                         icon: Terminal, showInBottom: false},
  {key: '/onboarding',  labelKey: 'nav.onboarding',                                   icon: Rocket,   showInBottom: false},
];

// Title falls back to the nav label key where they coincide; statistics has a
// fuller title than its short nav label.
const PAGE_META: Record<string, {titleKey: string; subtitleKey?: string}> = {
  '/mowglinext':  {titleKey: 'nav.home',                 subtitleKey: 'pageMeta.home.subtitle'},
  '/map':         {titleKey: 'nav.map',                  subtitleKey: 'pageMeta.map.subtitle'},
  '/schedule':    {titleKey: 'nav.schedule',             subtitleKey: 'pageMeta.schedule.subtitle'},
  '/diagnostics': {titleKey: 'nav.diagnostics',          subtitleKey: 'pageMeta.diagnostics.subtitle'},
  '/statistics':  {titleKey: 'pageMeta.statistics.title', subtitleKey: 'pageMeta.statistics.subtitle'},
  '/settings':    {titleKey: 'nav.settings',             subtitleKey: 'pageMeta.settings.subtitle'},
  '/parameters':  {titleKey: 'nav.parameters',           subtitleKey: 'pageMeta.parameters.subtitle'},
  '/logs':        {titleKey: 'nav.logs',                 subtitleKey: 'pageMeta.logs.subtitle'},
  '/onboarding':  {titleKey: 'nav.onboarding'},
};

export function AppShell() {
  const {colors} = useThemeMode();
  const {t} = useTranslation();
  const navigate = useNavigate();
  const route = useMatches();
  const isMobile = useIsMobile();

  const currentPath = route.length > 1 ? route[1].pathname : '/mowglinext';
  const metaKeys = PAGE_META[currentPath];
  const meta = {
    title: metaKeys ? t(metaKeys.titleKey) : 'MowgliNext',
    subtitle: metaKeys?.subtitleKey ? t(metaKeys.subtitleKey) : undefined,
  };

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

  // Bottom-nav items: the primary destinations live in the bar; the rest are
  // reachable through a "More" overflow sheet so nothing is unreachable on mobile.
  const bottomItems = useMemo(() => NAV.filter(n => n.showInBottom), []);
  const overflowItems = useMemo(() => NAV.filter(n => !n.showInBottom), []);
  const [moreOpen, setMoreOpen] = useState(false);

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
            <LanguageSwitcher/>
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

        <MobileMoreSheet
          open={moreOpen}
          items={overflowItems}
          activePath={currentPath}
          onClose={() => setMoreOpen(false)}
          onNavigate={(k) => { setMoreOpen(false); navigate({pathname: k}); }}
        />

        <MobileBottomNav
          items={bottomItems}
          activePath={currentPath}
          onNavigate={(k) => navigate({pathname: k})}
          onMore={() => setMoreOpen(true)}
          moreActive={overflowItems.some(n => n.key === currentPath) || moreOpen}
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
            <LanguageSwitcher/>
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
  onMore?: () => void;
  moreActive?: boolean;
}

function DesktopSideRail({items, activePath, onNavigate}: RailProps) {
  const {t} = useTranslation();
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
          {items.map(({key, labelKey, icon: Icon}) => {
            const isActive = key === activePath;
            return (
              <button
                key={key}
                onClick={() => onNavigate(key)}
                aria-label={t(labelKey)}
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
                <span>{t(labelKey)}</span>
              </button>
            );
          })}
        </nav>
      </LayoutGroup>
    </aside>
  );
}

// ─── Mobile bottom nav ───
const bottomNavBtnStyle = (isActive: boolean): React.CSSProperties => ({
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
});

const bottomNavPill = (
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
);

function MobileBottomNav({items, activePath, onNavigate, onMore, moreActive}: RailProps) {
  const {t} = useTranslation();
  const columns = items.length + (onMore ? 1 : 0);
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
          gridTemplateColumns: `repeat(${columns}, 1fr)`,
          gap: 2, padding: 6,
          background: 'rgba(255, 255, 255, 0.04)',
          border: '1px solid rgba(236, 255, 244, 0.08)',
          borderRadius: 999,
          backdropFilter: 'blur(28px)',
        }}>
          {items.map(({key, labelKey, shortLabelKey, icon: Icon}) => {
            const isActive = key === activePath;
            return (
              <button key={key} onClick={() => onNavigate(key)} aria-label={t(labelKey)} style={bottomNavBtnStyle(isActive)}>
                {isActive && bottomNavPill}
                <Icon size={18} strokeWidth={isActive ? 2.4 : 2}/>
                <span>{t(shortLabelKey ?? labelKey)}</span>
              </button>
            );
          })}
          {onMore && (
            <button onClick={onMore} aria-label={t('nav.more')} style={bottomNavBtnStyle(!!moreActive)}>
              {moreActive && bottomNavPill}
              <MoreHorizontal size={18} strokeWidth={moreActive ? 2.4 : 2}/>
              <span>{t('nav.more')}</span>
            </button>
          )}
        </div>
      </LayoutGroup>
    </nav>
  );
}

// ─── Mobile "More" overflow sheet ───
interface MoreSheetProps {
  open: boolean;
  items: NavItem[];
  activePath: string;
  onClose: () => void;
  onNavigate: (k: string) => void;
}

function MobileMoreSheet({open, items, activePath, onClose, onNavigate}: MoreSheetProps) {
  const {t} = useTranslation();
  return (
    <AnimatePresence>
      {open && (
        <>
          <motion.div
            initial={{opacity: 0}} animate={{opacity: 1}} exit={{opacity: 0}}
            onClick={onClose}
            style={{position: 'fixed', inset: 0, background: 'rgba(2, 17, 13, 0.6)', zIndex: 60}}
          />
          <motion.div
            initial={{y: '100%'}} animate={{y: 0}} exit={{y: '100%'}}
            transition={{type: 'spring', stiffness: 420, damping: 38}}
            style={{
              position: 'fixed', left: 0, right: 0, bottom: 0, zIndex: 61,
              padding: '14px 14px calc(env(safe-area-inset-bottom, 0px) + 18px)',
              background: 'rgba(6, 24, 18, 0.97)',
              backdropFilter: 'blur(24px) saturate(140%)',
              borderTop: '1px solid rgba(236, 255, 244, 0.1)',
              borderRadius: '20px 20px 0 0',
            }}
          >
            <div style={{display: 'flex', alignItems: 'center', justifyContent: 'space-between', marginBottom: 12}}>
              <span style={{fontSize: 13, fontWeight: 600, color: 'rgba(236, 255, 244, 0.66)'}}>{t('nav.more')}</span>
              <button onClick={onClose} aria-label={t('nav.close')} style={{
                background: 'transparent', border: 'none', cursor: 'pointer', color: 'rgba(236, 255, 244, 0.66)',
                display: 'flex', padding: 4,
              }}>
                <X size={20}/>
              </button>
            </div>
            <div style={{display: 'grid', gridTemplateColumns: 'repeat(2, 1fr)', gap: 10}}>
              {items.map(({key, labelKey, icon: Icon}) => {
                const isActive = key === activePath;
                return (
                  <button key={key} onClick={() => onNavigate(key)} style={{
                    display: 'flex', alignItems: 'center', gap: 12,
                    padding: '14px 16px', borderRadius: 14, cursor: 'pointer',
                    background: isActive ? 'rgba(124, 255, 178, 0.14)' : 'rgba(255, 255, 255, 0.04)',
                    border: `1px solid ${isActive ? 'rgba(124, 255, 178, 0.4)' : 'rgba(236, 255, 244, 0.08)'}`,
                    color: isActive ? '#7CFFB2' : 'rgba(236, 255, 244, 0.82)',
                    fontSize: 14, fontWeight: 600,
                  }}>
                    <Icon size={20}/>
                    <span>{t(labelKey)}</span>
                  </button>
                );
              })}
            </div>
          </motion.div>
        </>
      )}
    </AnimatePresence>
  );
}

export default AppShell;

// Type alias for callers that previously imported Root.
export {AppShell as Root};
export type {ReactNode};
