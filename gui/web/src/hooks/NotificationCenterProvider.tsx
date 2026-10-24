import {useCallback, useEffect, useMemo, useRef, useState, type ReactNode} from "react";
import {useTranslation} from "react-i18next";
import {updaterRequest, useHostUpdater} from './useHostUpdater';
import {NotificationCenterContext, type NotificationCenterValue, type NotificationItem} from './useNotificationCenter';

const MAX_ITEMS = 100;

export function NotificationCenterProvider({children}: {children: ReactNode}) {
    const {t} = useTranslation();
    const host = useHostUpdater();
    const [items, setItems] = useState<NotificationItem[]>([]);
    const counterRef = useRef(0);

    useEffect(() => {
        if (!host.data) return;
        const notices: NotificationItem[] = host.data.state.notices.filter(n => !n.dismissed).map(n => ({
            id: `update:${n.id}`, level: 'info', title: t(n.kind === 'updater' ? 'hostUpdater.agentNotice' : n.kind === 'available' ? 'hostUpdater.notice' : 'hostUpdater.reviewNotice'),
            body: n.deployment, timestamp: new Date(n.created_at).getTime(), read: n.read, href: '/#/settings?section=updates',
        }));
        setItems(previous => [...notices, ...previous.filter(n => !n.id.startsWith('update:'))].slice(0, MAX_ITEMS));
    }, [host.data, t]);

    const acknowledgeUpdate = useCallback((id: string, dismiss: boolean) => {
        if (id.startsWith('update:')) void updaterRequest('notice', {id: id.slice(7), dismiss}).then(host.refresh).catch(() => { /* Next cached status read restores unacknowledged notices. */ });
    }, [host.refresh]);

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
        acknowledgeUpdate(id, false);
        setItems(prev => prev.map(it => it.id === id ? {...it, read: true} : it));
    }, [acknowledgeUpdate]);

    const markAllRead = useCallback(() => {
        items.filter(it => !it.read).forEach(it => acknowledgeUpdate(it.id, false));
        setItems(prev => prev.map(it => it.read ? it : {...it, read: true}));
    }, [items, acknowledgeUpdate]);

    const dismiss = useCallback((id: string) => {
        acknowledgeUpdate(id, true);
        setItems(prev => prev.filter(it => it.id !== id));
    }, [acknowledgeUpdate]);

    const clear = useCallback(() => {items.forEach(it => acknowledgeUpdate(it.id, true)); setItems([]);}, [items, acknowledgeUpdate]);

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
