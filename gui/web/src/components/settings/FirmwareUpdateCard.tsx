import { useState } from "react";
import { Alert, Button, Card, Modal, Space, Tag, Tooltip, Typography } from "antd";
import { useTranslation } from "react-i18next";
import { FlashBoardComponent } from "../FlashBoardComponent.tsx";
import { useAvailableFirmware } from "../../hooks/useAvailableFirmware.ts";
import { useHighLevelStatus } from "../../hooks/useHighLevelStatus.ts";
import type { FirmwareInventoryState } from "../../utils/versions.ts";

const { Text } = Typography;

export interface FirmwareUpdateCardProps {
    firmwareVersion?: string;
    protocolVersion?: number | string;
    state: FirmwareInventoryState;
    configuredModel?: string;
    advanced: boolean;
}

/**
 * Mainboard firmware card of the Updates page: what runs on the STM32, what
 * this installation would flash, and a Flash button that opens the flash flow
 * in place — no detour through the onboarding wizard.
 *
 * Flashing reboots the board (motors and blade stop), so the button is only
 * enabled on the charger, or when the running firmware is already
 * incompatible (the stack cannot drive the robot then anyway).
 */
export function FirmwareUpdateCard({
    firmwareVersion,
    protocolVersion,
    state,
    configuredModel,
    advanced,
}: FirmwareUpdateCardProps) {
    const { t } = useTranslation();
    const [flashOpen, setFlashOpen] = useState(false);
    const available = useAvailableFirmware();
    const { highLevelStatus } = useHighLevelStatus();
    const unknown = t("updates.unknown");

    const offer = available.data?.available ? available.data : null;
    const newer = !!offer?.fw_version && offer.fw_version !== firmwareVersion;
    const showFlash = state === "incompatible" || (!!offer && newer);
    const canFlash = highLevelStatus.is_charging === true || state === "incompatible";

    return (
        <Card title={t("updates.mainboard")} size="small">
            <Space direction="vertical" style={{ width: "100%" }}>
                <div className="installed-version-heading">
                    <Text code>{firmwareVersion || unknown}</Text>
                    <Tag color={state === "compatible" ? "success" : state === "incompatible" ? "error" : "default"}>
                        {t(`updates.firmwareStates.${state}`)}
                    </Tag>
                </div>
                <Text type="secondary">{t("updates.firmwareMeaning")}</Text>
                {offer && (
                    <Text>
                        {newer
                            ? t("updates.firmwareAvailable", { version: offer.fw_version, protocol: offer.protocol_version })
                            : t("updates.firmwareUpToDate", { version: offer.fw_version })}
                        {!offer.own_release && (
                            <Text type="secondary"> {t("updates.firmwareFromLatestStable", { release: offer.release })}</Text>
                        )}
                    </Text>
                )}
                {available.data && !available.data.board && (
                    <Text type="secondary">{t("updates.firmwareNoBoard")}</Text>
                )}
                {available.data?.board && !available.data.available && (
                    <Text type="secondary">{t("updates.firmwareNoPrebuilt", { board: available.data.board })}</Text>
                )}
                {available.error && <Alert type="warning" showIcon message={t("updates.firmwareCheckFailed")} description={available.error} />}
                {advanced && (
                    <dl>
                        <dt>{t("updates.protocol")}</dt>
                        <dd>{protocolVersion || unknown}</dd>
                        <dt>{t("updates.configuredModel")}</dt>
                        <dd>{configuredModel || unknown}</dd>
                        <dt>{t("updates.firmwareRelease")}</dt>
                        <dd>{offer?.release || unknown}</dd>
                        <dt>{t("updates.boardRevision")}</dt>
                        <dd>{t("updates.notReported")}</dd>
                    </dl>
                )}
                {showFlash && (
                    <Tooltip title={canFlash ? undefined : t("updates.firmwareFlashNeedsDock")}>
                        <Button
                            danger={state === "incompatible"}
                            type={state === "incompatible" ? "default" : "primary"}
                            disabled={!canFlash}
                            onClick={() => setFlashOpen(true)}
                        >
                            {offer?.fw_version
                                ? t("updates.firmwareFlashVersion", { version: offer.fw_version })
                                : t("mowgliNextPage.firmwareFlashCta")}
                        </Button>
                    </Tooltip>
                )}
            </Space>
            <Modal
                open={flashOpen}
                title={t("onboardingPage.flashFirmware")}
                footer={null}
                width={900}
                destroyOnHidden
                onCancel={() => setFlashOpen(false)}
            >
                <FlashBoardComponent
                    mowerModel={configuredModel}
                    onNext={() => {
                        setFlashOpen(false);
                        void available.refresh();
                    }}
                />
            </Modal>
        </Card>
    );
}
