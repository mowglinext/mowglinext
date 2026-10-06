import React, { useEffect, useRef, useState } from "react";
import { Alert, Button, Card, Descriptions, Space } from "antd";
import { useTranslation } from "react-i18next";
import { ContentType } from "../../api/Api.ts";
import { useApi } from "../../hooks/useApi.ts";

type Calibration = {
    state: "idle" | "recording" | "ready" | "applied" | "failed";
    error?: string; samples: number; distance_m: number; ticks_per_meter: number;
    left_motor_revolutions: number; right_motor_revolutions: number;
};

export const MavrosCalibrationCard: React.FC<{
    acceptPersistedValues?: (values: Record<string, unknown>) => void;
}> = ({ acceptPersistedValues }) => {
    const { t } = useTranslation();
    const api = useApi();
    const [calibration, setCalibration] = useState<Calibration | null>(null);
    const [error, setError] = useState<string>();
    const [busy, setBusy] = useState(false);
    const actionEpoch = useRef(0);
    useEffect(() => {
        let cancelled = false;
        const refresh = async () => {
            const epoch = actionEpoch.current;
            try {
                const response = await api.request({ path: "/tools/drive/mavros-calibration", method: "GET", format: "json" });
                if (!cancelled && epoch === actionEpoch.current && !response.error) setCalibration(response.data as Calibration);
            } catch { /* Explicit actions show transport errors below. */ }
        };
        void refresh();
        const timer = window.setInterval(() => void refresh(), 1000);
        return () => { cancelled = true; window.clearInterval(timer); };
    }, [api]);
    const action = async (name: "start" | "finish" | "apply") => {
        actionEpoch.current++;
        setBusy(true); setError(undefined);
        try {
            const response = await api.request({ path: `/tools/drive/mavros-calibration/${name}`, method: "POST", format: "json",
                type: ContentType.Json, body: name === "apply" ? { confirm: true } : {} });
            if (response.error) throw new Error((response.error as { error?: string }).error ?? t("settingsMavrosCalibration.unavailable"));
            const result = response.data as Calibration;
            actionEpoch.current++;
            setCalibration(result);
            if (result.state === "applied") acceptPersistedValues?.({ ticks_per_meter: result.ticks_per_meter });
        } catch (e) { setError(e instanceof Error ? e.message : t("settingsMavrosCalibration.unavailable")); }
        finally { setBusy(false); }
    };
    return <Card title={t("settingsMavrosCalibration.title")}>
        <Space direction="vertical" style={{ width: "100%" }}>
            <Alert type="info" showIcon message={t("settingsMavrosCalibration.instructions")} />
            <Descriptions column={1} size="small">
                <Descriptions.Item label={t("settingsMavrosCalibration.state")}>{t(`settingsMavrosCalibration.states.${calibration?.state ?? "idle"}`)}</Descriptions.Item>
                <Descriptions.Item label={t("settingsMavrosCalibration.samples")}>{calibration?.samples ?? 0}</Descriptions.Item>
                {calibration && ["ready", "applied"].includes(calibration.state) && <>
                    <Descriptions.Item label={t("settingsMavrosCalibration.distance")}>{calibration.distance_m.toFixed(3)} m</Descriptions.Item>
                    <Descriptions.Item label={t("settingsMavrosCalibration.proposal")}>{calibration.ticks_per_meter.toFixed(3)}</Descriptions.Item>
                </>}
            </Descriptions>
            {(error || calibration?.error) && <Alert type="error" message={error ?? calibration?.error} />}
            <Space>
                <Button disabled={busy || calibration?.state === "recording"} onClick={() => void action("start")}>{t("settingsMavrosCalibration.start")}</Button>
                <Button disabled={busy || calibration?.state !== "recording"} onClick={() => void action("finish")}>{t("settingsMavrosCalibration.finish")}</Button>
                <Button disabled={busy || calibration?.state !== "ready"} onClick={() => void action("apply")}>{t("settingsMavrosCalibration.apply")}</Button>
            </Space>
        </Space>
    </Card>;
};
