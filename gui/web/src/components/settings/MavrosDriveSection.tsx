import React from "react";
import { Alert, App, Card, Descriptions, InputNumber, Space, Switch, Typography } from "antd";
import { useTranslation } from "react-i18next";
import { useMavrosDiagnostics } from "../../hooks/useMavrosDiagnostics.ts";
import { MavrosCalibrationCard } from "./MavrosCalibrationCard.tsx";

const { Text } = Typography;

type Props = {
    values: Record<string, unknown>;
    onChange: (key: string, value: number | boolean | null) => void;
    acceptPersistedValues?: (values: Record<string, unknown>) => void;
};

export const MavrosDriveSection: React.FC<Props> = ({ values, onChange, acceptPersistedValues }) => {
    const { t } = useTranslation();
    const { modal } = App.useApp();
    const { odometry } = useMavrosDiagnostics();
    const source = odometry?.entry;
    const fields = odometry?.fields ?? {};
    const prefix = `${fields.wheel_tick_source ?? fields.active_source}/`;
    const state = !source ? t("settingsMavrosDrive.status.unavailable")
        : source.level === 0 ? t("settingsMavrosDrive.status.live")
            : t("settingsMavrosDrive.status.unavailable");

    return (
        <Space direction="vertical" size={16} style={{ width: "100%" }}>
            <Alert type="info" showIcon message={t("settingsMavrosDrive.notice.title")}
                description={t("settingsMavrosDrive.notice.description")} />
            <Card title={t("settingsMavrosTraction.title")}>
                <Space direction="vertical">
                    <Text>{t("settingsMavrosTraction.description")}</Text>
                    <Switch aria-label={t("settingsMavrosTraction.title")} checked={values.mavros_manual_control_enabled === true}
                        onChange={(enabled) => {
                            if (!enabled) onChange("mavros_manual_control_enabled", false);
                            else modal.confirm({ title: t("settingsMavrosTraction.confirmTitle"), content: t("settingsMavrosTraction.confirmDescription"),
                                onOk: () => onChange("mavros_manual_control_enabled", true) });
                        }} />
                </Space>
            </Card>
            <Card title={t("settingsMavrosDrive.calibration.title")}>
                <Space direction="vertical" style={{ width: "100%" }}>
                    <Text>{t("settingsMavrosDrive.calibration.ticksPerMeter")}</Text>
                    <InputNumber min={0.001} precision={3} style={{ width: "100%" }}
                        value={Number.isFinite(Number(values.ticks_per_meter)) ? Number(values.ticks_per_meter) : null}
                        onChange={(value) => onChange("ticks_per_meter", value)} />
                    <Text>{t("settingsMavrosDrive.calibration.wheelTrack")}</Text>
                    <InputNumber min={0.0001} precision={4} style={{ width: "100%" }}
                        value={Number.isFinite(Number(values.wheel_track)) ? Number(values.wheel_track) : null}
                        onChange={(value) => onChange("wheel_track", value)} />
                </Space>
            </Card>
            <MavrosCalibrationCard acceptPersistedValues={acceptPersistedValues} />
            <Card title={t("settingsMavrosDrive.odometry.title")}>
                <Descriptions size="small" column={1}>
                    <Descriptions.Item label={t("settingsMavrosDrive.odometry.source")}>{source?.message ?? t("common.unknown")}</Descriptions.Item>
                    <Descriptions.Item label={t("settingsMavrosDrive.odometry.status")}>{state}</Descriptions.Item>
                    <Descriptions.Item label={t("settingsMavrosDrive.odometry.leftTicks")}>{fields[prefix+"left_valid"] === "true" ? fields[prefix+"left_raw_ticks"] : t("common.unknown")}</Descriptions.Item>
                    <Descriptions.Item label={t("settingsMavrosDrive.odometry.rightTicks")}>{fields[prefix+"right_valid"] === "true" ? fields[prefix+"right_raw_ticks"] : t("common.unknown")}</Descriptions.Item>
                </Descriptions>
            </Card>
        </Space>
    );
};
