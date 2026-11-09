import {
    createContext, useContext, useEffect, useRef,
} from "react";
import {useTranslation} from "react-i18next";

/**
 * App-scoped notification center.
 *
 * Buffers persistent notifications across pages so operators can come back to
 * them, and exposes a bell + drawer UI. Independent from AntD's transient
 * `notification.error()` toasts; for now items only enter the buffer by an
 * explicit `push()` call. Future work: auto-mirror critical AntD toasts here,
 * and auto-generate items from BT state transitions (boundary recovery,
 * coverage failed, etc.).
 */

export type NotificationLevel = 'info' | 'warning' | 'error' | 'success';

export interface NotificationItem {
    id: string;
    level: NotificationLevel;
    title: string;
    body?: string;
    timestamp: number;
    read: boolean;
    href?: string;
}

export interface NotificationCenterValue {
    items: NotificationItem[];
    unread: number;
    push: (n: Omit<NotificationItem, 'id' | 'timestamp' | 'read'>) => void;
    markRead: (id: string) => void;
    markAllRead: () => void;
    dismiss: (id: string) => void;
    clear: () => void;
}

export const NotificationCenterContext = createContext<NotificationCenterValue>({
    items: [],
    unread: 0,
    push: () => { /* no-op */ },
    markRead: () => { /* no-op */ },
    markAllRead: () => { /* no-op */ },
    dismiss: () => { /* no-op */ },
    clear: () => { /* no-op */ },
});

export function useNotificationCenter(): NotificationCenterValue {
    return useContext(NotificationCenterContext);
}

/**
 * Hook variant that subscribes to selected app events and auto-pushes
 * notifications. Mounted once at app root so the drawer is populated even
 * when the operator is on a different page.
 */
export function useAutoNotifications(opts: {
    emergencyActive: boolean;
    emergencyLatched: boolean;
    rainDetected: boolean;
    state: string | undefined;
}) {
    const {t} = useTranslation();
    const {push} = useNotificationCenter();
    const prevEmergencyRef = useRef(opts.emergencyActive);
    const prevRainRef = useRef(opts.rainDetected);
    const prevStateRef = useRef(opts.state);

    useEffect(() => {
        if (opts.emergencyActive && !prevEmergencyRef.current) {
            push({level: 'error', title: t('notificationCenter.emergencyTitle'),
                  body: t('notificationCenter.emergencyBody')});
        }
        prevEmergencyRef.current = opts.emergencyActive;
    }, [opts.emergencyActive, push, t]);

    useEffect(() => {
        if (opts.rainDetected && !prevRainRef.current) {
            push({level: 'warning', title: t('notificationCenter.rainTitle'),
                  body: t('notificationCenter.rainBody')});
        }
        prevRainRef.current = opts.rainDetected;
    }, [opts.rainDetected, push, t]);

    useEffect(() => {
        if (opts.state === 'MOWING_COMPLETE' && prevStateRef.current !== 'MOWING_COMPLETE') {
            push({level: 'success', title: t('notificationCenter.mowingCompleteTitle'),
                  body: t('notificationCenter.mowingCompleteBody')});
        }
        if (opts.state === 'BOUNDARY_EMERGENCY_STOP' && prevStateRef.current !== 'BOUNDARY_EMERGENCY_STOP') {
            push({level: 'error', title: t('notificationCenter.boundaryTitle'),
                  body: t('notificationCenter.boundaryBody')});
        }
        prevStateRef.current = opts.state;
    }, [opts.state, push, t]);
}
