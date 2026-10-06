import React from "react";
import { Alert, App, Card, Descriptions, Space, Switch, Tag } from "antd";
import { useTranslation } from "react-i18next";
import { useMavrosDiagnostics } from "../../hooks/useMavrosDiagnostics.ts";

export const MavrosSafetySection: React.FC<{
    values: Record<string, unknown>;
    onChange: (key: string, value: boolean) => void;
}> = ({ values, onChange }) => {
    const { t } = useTranslation();
    const { modal } = App.useApp();
    const { safety, lift } = useMavrosDiagnostics();
    const switchState = ["engaged", "released"].includes(safety?.entry.message ?? "") ? safety!.entry.message : "unknown";
    const valid = lift?.fields.state_valid === "true";
    const left = valid ? lift?.fields.left_lifted : "unknown";
    const right = valid ? lift?.fields.right_lifted : "unknown";
    const lifted = Number(left === "true") + Number(right === "true");
    const color = lifted === 2 ? "error" : lifted === 1 ? "orange" : "success";
    const enabled = values.mavros_wheel_lift_safety_enabled !== false;
    const wheelTag = (state?: string) => <Tag color={state === "unknown" || state == null ? "default" : color}>
        {t(`settingsMavrosSafety.states.${state === "true" ? "lifted" : state === "false" ? "ground" : "unknown"}`)}
    </Tag>;

    return <Card title={t("settingsMavrosSafety.title")}>
        <Space direction="vertical" style={{ width: "100%" }}>
            <Alert type="info" showIcon message={t("settingsMavrosSafety.physicalSafety")} />
            <Descriptions column={1}>
                <Descriptions.Item label={t("settingsMavrosSafety.switch")}><Tag color={switchState === "engaged" ? "error" : switchState === "released" ? "success" : "default"}>{t(`settingsMavrosSafety.states.${switchState}`)}</Tag></Descriptions.Item>
                <Descriptions.Item label={t("settingsMavrosSafety.left")}>{wheelTag(left)}</Descriptions.Item>
                <Descriptions.Item label={t("settingsMavrosSafety.right")}>{wheelTag(right)}</Descriptions.Item>
                <Descriptions.Item label={t("settingsMavrosSafety.liveProtection")}>{lift ? t(`settingsMavrosSafety.${lift.fields.safety_enabled === "true" ? "on" : "off"}`) : t("settingsMavrosSafety.states.unknown")}</Descriptions.Item>
            </Descriptions>
            <Space>
                <Switch aria-label={t("settingsMavrosSafety.protection")} checked={enabled} onChange={(checked) => {
                    if (checked) onChange("mavros_wheel_lift_safety_enabled", true);
                    else modal.confirm({ title: t("settingsMavrosSafety.disableTitle"), content: t("settingsMavrosSafety.disableDescription"),
                        onOk: () => onChange("mavros_wheel_lift_safety_enabled", false) });
                }} />
                {t("settingsMavrosSafety.protection")}
            </Space>
        </Space>
    </Card>;
};
