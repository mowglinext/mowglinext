import {useCallback, useEffect, useState} from "react";
import {Alert, App, Card, Spin, Switch, Typography} from "antd";
import {CloudSyncOutlined} from "@ant-design/icons";
import {useTranslation} from "react-i18next";
import {useApi} from "../../hooks/useApi.ts";
import {useSoilStatus} from "../../hooks/useSoilStatus.ts";
import {
    apiErrorMessage,
    type IrriSenseGardenSummary,
    type IrriSenseSettings,
    type IrriSenseSettingsUpdate,
} from "../../types/irrisense.ts";
import {IrriSenseConnectionCard} from "./IrriSenseConnectionCard.tsx";
import {IrriSenseRuleCard} from "./IrriSenseRuleCard.tsx";
import {IrriSenseStatusLine} from "./IrriSenseStatusLine.tsx";
import type {ExternalSaver} from "../../hooks/useSettingsManager.ts";

const SAVER_ID = "irrisense";

export interface IrriSenseSectionProps {
    /** Hook into the settings page's single Save / Revert buttons. */
    registerSaver?: (id: string, saver: ExternalSaver) => void;
    unregisterSaver?: (id: string) => void;
}

const {Text, Paragraph} = Typography;

/** Long enough for the backend's immediate re-poll after a settings change. */
const STATUS_REFRESH_AFTER_SAVE_MS = 1500;

/**
 * IrriSense Cloud soil-moisture integration. Unlike the yaml-backed sections
 * this one owns its own load/save: the settings (token included) live in the
 * GUI's key-value DB, never in mowgli_robot.yaml — but it registers itself
 * with useSettingsManager as an external saver so the page's ONE Save button
 * (and Revert) covers it; there is no section-local Save button.
 */
export function IrriSenseSection({registerSaver, unregisterSaver}: IrriSenseSectionProps = {}) {
    const {t} = useTranslation();
    const guiApi = useApi();
    const {notification} = App.useApp();
    const [settings, setSettings] = useState<IrriSenseSettings | null>(null);
    const [draft, setDraft] = useState<IrriSenseSettingsUpdate>({});
    const [gardens, setGardens] = useState<IrriSenseGardenSummary[]>([]);
    const [loadError, setLoadError] = useState<string | null>(null);
    const [testing, setTesting] = useState(false);
    const {status, refresh: refreshStatus} = useSoilStatus();

    useEffect(() => {
        void (async () => {
            try {
                const res = await guiApi.request<IrriSenseSettings>({path: "/irrisense/settings", method: "GET", format: "json"});
                setSettings(res.data);
            } catch (e: unknown) {
                setLoadError(apiErrorMessage(e));
            }
        })();
    }, [guiApi]);

    const onChange = useCallback((patch: IrriSenseSettingsUpdate) => {
        setDraft((prev) => ({...prev, ...patch}));
    }, []);

    const isDirty = Object.keys(draft).length > 0;

    const save = useCallback(async (): Promise<boolean> => {
        if (!isDirty) return true;
        try {
            const res = await guiApi.request<IrriSenseSettings>({
                path: "/irrisense/settings", method: "PUT", body: draft, format: "json",
            });
            setSettings(res.data);
            setDraft({});
            notification.success({message: t("settingsIrriSense.saved")});
            window.setTimeout(refreshStatus, STATUS_REFRESH_AFTER_SAVE_MS);
            return true;
        } catch (e: unknown) {
            notification.error({message: t("settingsIrriSense.saveFailed"), description: apiErrorMessage(e)});
            return false;
        }
    }, [draft, guiApi, isDirty, notification, refreshStatus, t]);

    // Keep the page-level Save button informed of our pending edits.
    useEffect(() => {
        registerSaver?.(SAVER_ID, {
            dirtyCount: Object.keys(draft).length,
            save,
            revert: () => setDraft({}),
        });
    }, [draft, registerSaver, save]);
    useEffect(() => () => unregisterSaver?.(SAVER_ID), [unregisterSaver]);

    const testConnection = useCallback(async () => {
        setTesting(true);
        try {
            // The gardens call uses the STORED token, so pending edits go first.
            if (!(await save())) return;
            const res = await guiApi.request<{gardens: IrriSenseGardenSummary[]}>({
                path: "/irrisense/gardens", method: "GET", format: "json",
            });
            const list = res.data.gardens ?? [];
            setGardens(list);
            notification.success({
                message: t("settingsIrriSense.testOk", {count: list.length}),
                description: list.map((g) => g.name).join(", ") || undefined,
            });
        } catch (e: unknown) {
            notification.error({message: t("settingsIrriSense.testFailed"), description: apiErrorMessage(e)});
        } finally {
            setTesting(false);
        }
    }, [guiApi, notification, save, t]);

    if (loadError) {
        return <Alert type="error" showIcon message={t("settingsIrriSense.loadFailed")} description={loadError}/>;
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
                            <CloudSyncOutlined style={{marginRight: 6}}/>
                            {t("settingsIrriSense.title")}
                        </Text>
                        <Paragraph type="secondary" style={{margin: "4px 0 0"}}>
                            {t("settingsIrriSense.description")}
                        </Paragraph>
                    </div>
                    <Switch
                        checked={enabled}
                        onChange={(checked) => onChange({enabled: checked})}
                        aria-label={t("settingsIrriSense.title")}
                    />
                </div>
            </Card>

            {enabled && (
                <>
                    <Alert
                        type="info"
                        showIcon
                        style={{marginBottom: 16}}
                        message={t("settingsIrriSense.failOpenTitle")}
                        description={t("settingsIrriSense.failOpenBody")}
                    />
                    <IrriSenseConnectionCard
                        settings={settings}
                        draft={draft}
                        onChange={onChange}
                        gardens={gardens}
                        testing={testing}
                        onTestConnection={() => void testConnection()}
                    />
                    <IrriSenseRuleCard settings={settings} draft={draft} onChange={onChange}/>
                    <Card size="small" title={t("settingsIrriSense.status")} style={{marginBottom: 16}}>
                        <IrriSenseStatusLine status={status} onRefresh={() => void refreshStatus()}/>
                    </Card>
                </>
            )}

        </div>
    );
}
