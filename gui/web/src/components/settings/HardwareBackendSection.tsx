import React from "react";
import { Alert, Card, Descriptions, Typography } from "antd";
import { useTranslation } from "react-i18next";
import type { HardwareBackend } from "../../constants/hardwareBackends.ts";

const { Paragraph } = Typography;

type Props = {
    backend: HardwareBackend;
    /** The active backend's own settings cards (wiring, controller type). */
    children?: React.ReactNode;
};

/**
 * Which hardware bridge drives this robot, and the settings that exist only
 * for it (serial ports, motor-controller type). Settings MowgliNext already
 * has (charge limits, lift delays, ticks per metre…) stay in their usual
 * sections whatever the backend: the bridge maps them onto its hardware.
 */
export const HardwareBackendSection: React.FC<Props> = ({ backend, children }) => {
    const { t } = useTranslation();
    return (
        <div>
            <Card size="small" style={{ marginBottom: 16 }} data-testid="hardware-backend-card">
                <Descriptions size="small" column={1}>
                    <Descriptions.Item label={t("settingsHardwareBackend.active")}>
                        <strong data-testid="hardware-backend-name">
                            {t(`settingsHardwareBackend.names.${backend}`)}
                        </strong>
                    </Descriptions.Item>
                </Descriptions>
                <Paragraph type="secondary" style={{ margin: "8px 0 0" }}>
                    {t(`settingsHardwareBackend.descriptions.${backend}`)}
                </Paragraph>
                <Paragraph type="secondary" style={{ margin: "8px 0 0", fontSize: 12 }}>
                    {t("settingsHardwareBackend.choiceNote")}
                </Paragraph>
            </Card>
            {backend === "openmower" ? (
                <Alert
                    type="info"
                    showIcon
                    style={{ marginBottom: 16 }}
                    message={t("settingsHardwareBackend.openmowerRestartNote")}
                />
            ) : null}
            {children}
        </div>
    );
};
