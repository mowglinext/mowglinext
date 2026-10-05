import React from "react";
import { Alert, Card, Descriptions, Tag } from "antd";
import { useTranslation } from "react-i18next";
import { HardwareBackendInfo } from "../../constants/hardwareBackends.ts";

export const HardwareBackendCard: React.FC<{ info: HardwareBackendInfo }> = ({ info }) => {
    const { t } = useTranslation();
    return (
        <Card size="small" title={t("settingsHardwareBackend.title")} style={{ marginBottom: 16 }}>
            <Descriptions size="small" column={1}>
                <Descriptions.Item label={t("settingsHardwareBackend.active")}>
                    <Tag color={info.backend === "mavros" ? "blue" : "green"}>
                        {t(`settingsHardwareBackend.names.${info.backend}`)}
                    </Tag>
                </Descriptions.Item>
            </Descriptions>
            {info.backend === "mavros" && (
                <Alert
                    type="info"
                    showIcon
                    message={t("settingsHardwareBackend.mavrosTitle")}
                    description={t("settingsHardwareBackend.mavrosPending")}
                />
            )}
        </Card>
    );
};
