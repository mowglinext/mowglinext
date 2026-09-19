import {useCallback, useEffect, useState} from "react";
import {Alert, App, Button, Card, Spin, Switch, Typography} from "antd";
import {BellOutlined, ReloadOutlined} from "@ant-design/icons";
import {useTranslation} from "react-i18next";
import {useApi} from "../../hooks/useApi.ts";
import {apiErrorMessage} from "../../types/irrisense.ts";
import type {
    NotificationDeliveryStatus,
    NotificationSettings,
    NotificationSettingsUpdate,
} from "../../types/notifications.ts";
import {NotificationsChannelCard} from "./NotificationsChannelCard.tsx";
import {NotificationsEventsCard} from "./NotificationsEventsCard.tsx";
import type {ExternalSaver} from "../../hooks/useSettingsManager.ts";

const SAVER_ID = "notifications";

export interface NotificationsSectionProps {
    /** Hook into the settings page's single Save / Revert buttons. */
    registerSaver?: (id: string, saver: ExternalSaver) => void;
    unregisterSaver?: (id: string) => void;
}

const {Text, Paragraph} = Typography;

/**
 * Push notifications to the operator's phone (ntfy, Telegram or a webhook).
 * Like IrriSense this section owns its own load/save: the settings (tokens
 * included) live in the GUI's key-value DB, never in mowgli_robot.yaml, and
 * it registers with useSettingsManager as an external saver so the page's
 * ONE Save button covers it.
 */
export function NotificationsSection({registerSaver, unregisterSaver}: NotificationsSectionProps = {}) {
    const {t} = useTranslation();
    const guiApi = useApi();
    const {notification} = App.useApp();
    const [settings, setSettings] = useState<NotificationSettings | null>(null);
    const [draft, setDraft] = useState<NotificationSettingsUpdate>({});
    const [status, setStatus] = useState<NotificationDeliveryStatus | null>(null);
    const [loadError, setLoadError] = useState<string | null>(null);
    const [testing, setTesting] = useState(false);

    const refreshStatus = useCallback(async () => {
        try {
            const res = await guiApi.request<NotificationDeliveryStatus>({path: "/notifications/status", method: "GET", format: "json"});
            setStatus(res.data);
        } catch {
            // The status line is informational; a failed refresh keeps the last value.
        }
    }, [guiApi]);

    useEffect(() => {
        void (async () => {
            try {
                const res = await guiApi.request<NotificationSettings>({path: "/notifications/settings", method: "GET", format: "json"});
                setSettings(res.data);
                await refreshStatus();
            } catch (e: unknown) {
                setLoadError(apiErrorMessage(e));
            }
        })();
    }, [guiApi, refreshStatus]);

    const onChange = useCallback((patch: NotificationSettingsUpdate) => {
        setDraft((prev) => ({...prev, ...patch}));
    }, []);

    const isDirty = Object.keys(draft).length > 0;

    const save = useCallback(async (): Promise<boolean> => {
        if (!isDirty) return true;
        try {
            const res = await guiApi.request<NotificationSettings>({
                path: "/notifications/settings", method: "PUT", body: draft, format: "json",
            });
            setSettings(res.data);
            setDraft({});
            notification.success({message: t("settingsNotifications.saved")});
            void refreshStatus();
            return true;
        } catch (e: unknown) {
            notification.error({message: t("settingsNotifications.saveFailed"), description: apiErrorMessage(e)});
            return false;
        }
    }, [draft, guiApi, isDirty, notification, refreshStatus, t]);

    useEffect(() => {
        registerSaver?.(SAVER_ID, {
            dirtyCount: Object.keys(draft).length,
            save,
            revert: () => setDraft({}),
        });
    }, [draft, registerSaver, save]);
    useEffect(() => () => unregisterSaver?.(SAVER_ID), [unregisterSaver]);

    const sendTest = useCallback(async () => {
        setTesting(true);
        try {
            // The test uses the STORED settings, so pending edits go first.
            if (!(await save())) return;
            await guiApi.request({path: "/notifications/test", method: "POST", format: "json"});
            notification.success({message: t("settingsNotifications.testOk")});
        } catch (e: unknown) {
            notification.error({message: t("settingsNotifications.testFailed"), description: apiErrorMessage(e)});
        } finally {
            setTesting(false);
            void refreshStatus();
        }
    }, [guiApi, notification, refreshStatus, save, t]);

    if (loadError) {
        return <Alert type="error" showIcon message={t("settingsNotifications.loadFailed")} description={loadError}/>;
    }
    if (!settings) {
        return <Spin style={{display: "block", margin: "40px auto"}}/>;
    }

    const enabled = draft.enabled ?? settings.enabled;

    return (
        <div>
            <Card size="small" style={{marginBottom: 16}}>
                <div style={{display: "flex", justifyContent: "space-between", alignItems: "center", gap: 12}}>
                    <div>
                        <Text strong style={{fontSize: 14}}>
                            <BellOutlined style={{marginRight: 6}}/>
                            {t("settingsNotifications.title_section")}
                        </Text>
                        <Paragraph type="secondary" style={{margin: "4px 0 0"}}>
                            {t("settingsNotifications.description")}
                        </Paragraph>
                    </div>
                    <Switch
                        checked={enabled}
                        onChange={(checked) => onChange({enabled: checked})}
                        aria-label={t("settingsNotifications.title_section")}
                    />
                </div>
            </Card>

            <NotificationsChannelCard
                settings={settings}
                draft={draft}
                onChange={onChange}
                testing={testing}
                onSendTest={() => void sendTest()}
            />
            <NotificationsEventsCard settings={settings} draft={draft} onChange={onChange}/>

            <Card
                size="small"
                title={t("settingsNotifications.status")}
                style={{marginBottom: 16}}
                extra={<Button size="small" icon={<ReloadOutlined/>} onClick={() => void refreshStatus()}>{t("settingsNotifications.refresh")}</Button>}
            >
                {status ? <DeliveryStatusLine status={status}/> : <Text type="secondary">{t("settingsNotifications.statusUnknown")}</Text>}
            </Card>
        </div>
    );
}

function DeliveryStatusLine({status}: {status: NotificationDeliveryStatus}) {
    const {t} = useTranslation();
    const state = !status.configured
        ? t("settingsNotifications.stateNotConfigured")
        : !status.enabled
            ? t("settingsNotifications.stateDisabled")
            : t("settingsNotifications.stateArmed");
    return (
        <div style={{display: "flex", flexDirection: "column", gap: 4}}>
            <Text>{state}</Text>
            <Text type="secondary" style={{fontSize: 12}}>
                {t("settingsNotifications.counts", {sent: status.sentCount, failed: status.failedCount})}
            </Text>
            {status.lastSentAt && (
                <Text type="secondary" style={{fontSize: 12}}>
                    {t("settingsNotifications.lastSent", {when: new Date(status.lastSentAt).toLocaleString(), message: status.lastMessage ?? ""})}
                </Text>
            )}
            {status.lastError && (
                <Text type="danger" style={{fontSize: 12}}>
                    {t("settingsNotifications.lastError", {error: status.lastError})}
                </Text>
            )}
        </div>
    );
}
