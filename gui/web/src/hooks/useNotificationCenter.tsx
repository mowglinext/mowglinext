import {
    createContext, useCallback, useContext, useEffect, useMemo, useRef, useState,
    type ReactNode,
} from "react";

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
}

interface NotificationCenterValue {
    items: NotificationItem[];
    unread: number;
    push: (n: Omit<NotificationItem, 'id' | 'timestamp' | 'read'>) => void;
    markRead: (id: string) => void;
    markAllRead: () => void;
    dismiss: (id: string) => void;
    clear: () => void;
}

const NotificationCenterContext = createContext<NotificationCenterValue>({
    items: [],
    unread: 0,
    push: () => { /* no-op */ },
    markRead: () => { /* no-op */ },
    markAllRead: () => { /* no-op */ },
    dismiss: () => { /* no-op */ },
    clear: () => { /* no-op */ },
});

const MAX_ITEMS = 100;

export function NotificationCenterProvider({children}: {children: ReactNode}) {
    const [items, setItems] = useState<NotificationItem[]>([]);
    const counterRef = useRef(0);

    const push = useCallback((n: Omit<NotificationItem, 'id' | 'timestamp' | 'read'>) => {
        setItems(prev => {
            const next: NotificationItem = {
                ...n,
                id: `n-${Date.now()}-${counterRef.current++}`,
                timestamp: Date.now(),
                read: false,
            };
            const trimmed = [next, ...prev];
            return trimmed.length > MAX_ITEMS ? trimmed.slice(0, MAX_ITEMS) : trimmed;
        });
    }, []);

    const markRead = useCallback((id: string) => {
        setItems(prev => prev.map(it => it.id === id ? {...it, read: true} : it));
    }, []);

    const markAllRead = useCallback(() => {
        setItems(prev => prev.map(it => it.read ? it : {...it, read: true}));
    }, []);

    const dismiss = useCallback((id: string) => {
        setItems(prev => prev.filter(it => it.id !== id));
    }, []);

    const clear = useCallback(() => setItems([]), []);

    const unread = items.reduce((acc, it) => acc + (it.read ? 0 : 1), 0);

    const value = useMemo<NotificationCenterValue>(() => ({
        items, unread, push, markRead, markAllRead, dismiss, clear,
    }), [items, unread, push, markRead, markAllRead, dismiss, clear]);

    return (
        <NotificationCenterContext.Provider value={value}>
            {children}
        </NotificationCenterContext.Provider>
    );
}

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
    const {push} = useNotificationCenter();
    const prevEmergencyRef = useRef(opts.emergencyActive);
    const prevRainRef = useRef(opts.rainDetected);
    const prevStateRef = useRef(opts.state);

    useEffect(() => {
        if (opts.emergencyActive && !prevEmergencyRef.current) {
            push({level: 'error', title: 'Emergency stop triggered',
                  body: 'Firmware latched the e-stop. Check the robot, then tap Reset to clear.'});
        }
        prevEmergencyRef.current = opts.emergencyActive;
    }, [opts.emergencyActive, push]);

    useEffect(() => {
        if (opts.rainDetected && !prevRainRef.current) {
            push({level: 'warning', title: 'Rain detected',
                  body: 'Mowing paused. The robot will resume automatically once the rain clears.'});
        }
        prevRainRef.current = opts.rainDetected;
    }, [opts.rainDetected, push]);

    useEffect(() => {
        if (opts.state === 'MOWING_COMPLETE' && prevStateRef.current !== 'MOWING_COMPLETE') {
            push({level: 'success', title: 'Mowing complete', body: 'All scheduled areas have been mowed.'});
        }
        if (opts.state === 'BOUNDARY_EMERGENCY_STOP' && prevStateRef.current !== 'BOUNDARY_EMERGENCY_STOP') {
            push({level: 'error', title: 'Crossed the boundary',
                  body: 'The robot stopped after leaving the area. Lift it back in, then resume.'});
        }
        prevStateRef.current = opts.state;
    }, [opts.state, push]);
}
