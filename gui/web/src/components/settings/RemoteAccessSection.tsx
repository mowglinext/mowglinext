import {useCallback, useEffect, useState} from "react";
import {Alert, App, Card, Spin, Switch, Typography} from "antd";
import {GlobalOutlined} from "@ant-design/icons";
import {useTranslation} from "react-i18next";
import {useApi} from "../../hooks/useApi.ts";
import {useRemoteAccessStatus} from "../../hooks/useRemoteAccessStatus.ts";
import {apiErrorMessage} from "../../types/irrisense.ts";
import type {RemoteAccessSettings, RemoteAccessSettingsUpdate} from "../../types/remoteAccess.ts";
import {RemoteAccessConnectionCard} from "./RemoteAccessConnectionCard.tsx";
import {RemoteAccessStatusCard} from "./RemoteAccessStatusCard.tsx";
import type {ExternalSaver} from "../../hooks/useSettingsManager.ts";

const SAVER_ID = "remote_access";

export interface RemoteAccessSectionProps {
    /** Hook into the settings page's single Save / Revert buttons. */
    registerSaver?: (id: string, saver: ExternalSaver) => void;
    unregisterSaver?: (id: string) => void;
}

const {Text, Paragraph} = Typography;

/** Long enough for the backend to have started reconciling after a save. */
const STATUS_REFRESH_AFTER_SAVE_MS = 1000;

/**
 * Remote access through a Tailscale sidecar. Like IrriSense this section owns
 * its own load/save (the settings live in the GUI's key-value DB, never in
 * mowgli_robot.yaml) and registers with useSettingsManager so the page's ONE
 * Save button covers it.
 */
export function RemoteAccessSection({registerSaver, unregisterSaver}: RemoteAccessSectionProps = {}) {
    const {t} = useTranslation();
    const guiApi = useApi();
    const {notification} = App.useApp();
    const [settings, setSettings] = useState<RemoteAccessSettings | null>(null);
    const [draft, setDraft] = useState<RemoteAccessSettingsUpdate>({});
    const [loadError, setLoadError] = useState<string | null>(null);
    const [busy, setBusy] = useState(false);
    const enabled = draft.enabled ?? settings?.enabled ?? false;
    const {status, error: statusError, refresh: refreshStatus} = useRemoteAccessStatus({enabled: settings?.enabled ?? false});

    useEffect(() => {
        void (async () => {
            try {
                const res = await guiApi.request<RemoteAccessSettings>({path: "/remote-access/settings", method: "GET", format: "json"});
                setSettings(res.data);
            } catch (e: unknown) {
                setLoadError(apiErrorMessage(e));
            }
        })();
    }, [guiApi]);

    // An `undefined` in a patch withdraws that key (e.g. setting a key
    // cancels a pending clear) instead of counting as a pending edit.
    const onChange = useCallback((patch: RemoteAccessSettingsUpdate) => {
        setDraft((prev) => Object.fromEntries(
            Object.entries({...prev, ...patch}).filter(([, v]) => v !== undefined),
        ) as RemoteAccessSettingsUpdate);
    }, []);

    const isDirty = Object.keys(draft).length > 0;

    const save = useCallback(async (): Promise<boolean> => {
        if (!isDirty) return true;
        try {
            const res = await guiApi.request<RemoteAccessSettings>({
                path: "/remote-access/settings", method: "PUT", body: draft, format: "json",
            });
            setSettings(res.data);
            setDraft({});
            notification.success({message: t("settingsRemoteAccess.saved")});
            window.setTimeout(() => void refreshStatus(), STATUS_REFRESH_AFTER_SAVE_MS);
            return true;
        } catch (e: unknown) {
            notification.error({message: t("settingsRemoteAccess.saveFailed"), description: apiErrorMessage(e)});
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

    const runAction = useCallback(async (path: string, failureKey: string) => {
        setBusy(true);
        try {
            await guiApi.request({path, method: "POST", format: "json"});
            window.setTimeout(() => void refreshStatus(), STATUS_REFRESH_AFTER_SAVE_MS);
        } catch (e: unknown) {
            notification.error({message: t(failureKey), description: apiErrorMessage(e)});
        } finally {
            setBusy(false);
        }
    }, [guiApi, notification, refreshStatus, t]);

    if (loadError) {
        return <Alert type="error" showIcon message={t("settingsRemoteAccess.loadFailed")} description={loadError}/>;
    }
    if (!settings) {
        return <Spin style={{display: "block", margin: "40px auto"}}/>;
    }

    return (
        <div>
            <Card size="small" style={{marginBottom: 16}}>
                <div style={{display: "flex", justifyContent: "space-between", alignItems: "center", gap: 12}}>
                    <div>
                        <Text strong style={{fontSize: 14}}>
                            <GlobalOutlined style={{marginRight: 6}}/>
                            {t("settingsRemoteAccess.title")}
                        </Text>
                        <Paragraph type="secondary" style={{margin: "4px 0 0"}}>
                            {t("settingsRemoteAccess.description")}
                        </Paragraph>
                    </div>
                    <Switch
                        checked={enabled}
                        onChange={(checked) => onChange({enabled: checked})}
                        aria-label={t("settingsRemoteAccess.title")}
                    />
                </div>
            </Card>

            {enabled && (
                <>
                    <Alert
                        type="info"
                        showIcon
                        style={{marginBottom: 16}}
                        message={t("settingsRemoteAccess.scopeTitle")}
                        description={t("settingsRemoteAccess.scopeBody")}
                    />
                    <RemoteAccessConnectionCard settings={settings} draft={draft} onChange={onChange}/>
                    {settings.enabled ? (
                        <RemoteAccessStatusCard
                            status={status}
                            fetchError={statusError}
                            onRefresh={() => void refreshStatus()}
                            onRetry={() => void runAction("/remote-access/apply", "settingsRemoteAccess.retryFailed")}
                            onLogout={() => void runAction("/remote-access/logout", "settingsRemoteAccess.logoutFailed")}
                            busy={busy}
                        />
                    ) : (
                        <Alert type="warning" showIcon style={{marginBottom: 16}} message={t("settingsRemoteAccess.saveToStart")}/>
                    )}
                </>
            )}
        </div>
    );
}
