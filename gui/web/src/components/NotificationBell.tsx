import {useEffect, useRef, useState} from "react";
import {AnimatePresence, motion} from "framer-motion";
import {useNotificationCenter, type NotificationItem} from "../hooks/useNotificationCenter.tsx";
import {useThemeMode} from "../theme/ThemeContext.tsx";

const formatRelative = (ms: number): string => {
    const diff = Date.now() - ms;
    if (diff < 60_000) return 'just now';
    if (diff < 3_600_000) return `${Math.floor(diff / 60_000)}m ago`;
    if (diff < 86_400_000) return `${Math.floor(diff / 3_600_000)}h ago`;
    return `${Math.floor(diff / 86_400_000)}d ago`;
};

const dotKeyframes = `
@keyframes notifBellWiggle {
  0%, 100% { transform: rotate(0); }
  20% { transform: rotate(-10deg); }
  40% { transform: rotate(8deg); }
  60% { transform: rotate(-6deg); }
  80% { transform: rotate(4deg); }
}
`;

interface BellIconProps {
    color: string;
    animate: boolean;
    size?: number;
}
function BellIcon({color, animate, size = 18}: BellIconProps) {
    return (
        <svg width={size} height={size} viewBox="0 0 24 24" fill="none"
             stroke={color} strokeWidth={1.8} strokeLinecap="round" strokeLinejoin="round"
             style={{
                 transformOrigin: '50% 30%',
                 animation: animate ? 'notifBellWiggle 1.6s ease-in-out infinite' : 'none',
             }}>
            <path d="M6 8a6 6 0 1 1 12 0c0 7 3 9 3 9H3s3-2 3-9"/>
            <path d="M10.3 21a1.94 1.94 0 0 0 3.4 0"/>
        </svg>
    );
}

const levelColor = (lvl: NotificationItem['level'], colors: ReturnType<typeof useThemeMode>['colors']): string => {
    switch (lvl) {
        case 'error': return colors.danger;
        case 'warning': return colors.warning;
        case 'success': return colors.accent;
        default: return colors.info;
    }
};

export function NotificationBell() {
    const {colors} = useThemeMode();
    const {items, unread, markRead, markAllRead, dismiss, clear} = useNotificationCenter();
    const [open, setOpen] = useState(false);
    const wrapRef = useRef<HTMLDivElement | null>(null);

    // Click-outside to close
    useEffect(() => {
        if (!open) return;
        const handler = (e: MouseEvent) => {
            if (!wrapRef.current?.contains(e.target as Node)) setOpen(false);
        };
        document.addEventListener('mousedown', handler);
        return () => document.removeEventListener('mousedown', handler);
    }, [open]);

    return (
        <div ref={wrapRef} style={{position: 'relative'}}>
            <style>{dotKeyframes}</style>
            <button
                onClick={() => {
                    const next = !open;
                    setOpen(next);
                    if (next) markAllRead();
                }}
                aria-label={`Notifications${unread > 0 ? ` (${unread} unread)` : ''}`}
                style={{
                    position: 'relative', background: 'transparent', border: 'none',
                    cursor: 'pointer', padding: 6, display: 'inline-flex',
                    alignItems: 'center', color: colors.text,
                }}
            >
                <BellIcon color={colors.text} animate={unread > 0}/>
                {unread > 0 && (
                    <span style={{
                        position: 'absolute', top: 2, right: 2,
                        minWidth: 16, height: 16, padding: '0 4px', borderRadius: 8,
                        background: colors.danger, color: colors.bgBase,
                        fontSize: 10, fontWeight: 700,
                        display: 'inline-flex', alignItems: 'center', justifyContent: 'center',
                        boxShadow: `0 0 0 2px ${colors.bgBase}`,
                    }}>
                        {unread > 9 ? '9+' : unread}
                    </span>
                )}
            </button>

            <AnimatePresence>
                {open && (
                    <motion.div
                        initial={{opacity: 0, y: -6}}
                        animate={{opacity: 1, y: 0}}
                        exit={{opacity: 0, y: -6}}
                        transition={{duration: 0.15}}
                        style={{
                            position: 'absolute', top: 'calc(100% + 8px)', right: 0,
                            width: 'min(340px, calc(100vw - 24px))',
                            maxHeight: 'calc(100vh - 100px)',
                            background: colors.bgCard, border: `1px solid ${colors.border}`,
                            borderRadius: 12, boxShadow: colors.glassShadow,
                            overflow: 'hidden', zIndex: 1000,
                            display: 'flex', flexDirection: 'column',
                        }}
                    >
                        <div style={{
                            display: 'flex', alignItems: 'center', justifyContent: 'space-between',
                            padding: '12px 14px', borderBottom: `1px solid ${colors.border}`,
                        }}>
                            <div style={{fontSize: 13, fontWeight: 700, color: colors.text}}>
                                Notifications
                            </div>
                            {items.length > 0 && (
                                <button
                                    onClick={clear}
                                    style={{
                                        background: 'transparent', border: 'none', cursor: 'pointer',
                                        color: colors.textMuted, fontSize: 11, fontWeight: 600,
                                    }}
                                >Clear all</button>
                            )}
                        </div>
                        <div style={{flex: 1, overflowY: 'auto'}}>
                            {items.length === 0 && (
                                <div style={{
                                    padding: '40px 16px', textAlign: 'center',
                                    color: colors.textMuted, fontSize: 13,
                                }}>
                                    Nothing right now.
                                    <div style={{fontSize: 11, marginTop: 6}}>
                                        Critical events (emergency, rain, completion) will land here.
                                    </div>
                                </div>
                            )}
                            {items.map(item => {
                                const accent = levelColor(item.level, colors);
                                return (
                                    <div
                                        key={item.id}
                                        onClick={() => markRead(item.id)}
                                        style={{
                                            padding: '12px 14px',
                                            borderBottom: `1px solid ${colors.borderSubtle}`,
                                            cursor: 'pointer',
                                            background: item.read ? 'transparent' : `${accent}0a`,
                                            display: 'flex', gap: 10, alignItems: 'flex-start',
                                            transition: 'background 0.15s',
                                        }}
                                    >
                                        <span style={{
                                            width: 6, height: 6, borderRadius: 3, marginTop: 6,
                                            background: accent, flexShrink: 0,
                                            boxShadow: item.read ? 'none' : `0 0 6px ${accent}`,
                                        }}/>
                                        <div style={{flex: 1, minWidth: 0}}>
                                            <div style={{
                                                fontSize: 13, fontWeight: item.read ? 500 : 700,
                                                color: colors.text,
                                            }}>{item.title}</div>
                                            {item.body && (
                                                <div style={{fontSize: 12, color: colors.textDim, marginTop: 2, lineHeight: 1.4}}>
                                                    {item.body}
                                                </div>
                                            )}
                                            <div style={{fontSize: 10, color: colors.textMuted, marginTop: 4}}>
                                                {formatRelative(item.timestamp)}
                                            </div>
                                        </div>
                                        <button
                                            onClick={(e) => { e.stopPropagation(); dismiss(item.id); }}
                                            aria-label="Dismiss"
                                            style={{
                                                background: 'transparent', border: 'none', cursor: 'pointer',
                                                color: colors.textMuted, padding: 4, lineHeight: 1,
                                            }}
                                        >×</button>
                                    </div>
                                );
                            })}
                        </div>
                    </motion.div>
                )}
            </AnimatePresence>
        </div>
    );
}
