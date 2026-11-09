import {Alert, Card, Space, Tag, Typography} from "antd";
import {useTranslation} from "react-i18next";
import {formatMavrosBoardIdentity, formatMavrosFirmwareVersion, mavrosAutopilotName,
    mavrosVehicleTypeName, useMavrosInfo} from "../../hooks/useMavrosInfo.ts";

const {Text} = Typography;

export function MavrosFirmwareCard() {
    const {t} = useTranslation();
    const {state, vehicle} = useMavrosInfo();
    const connected = state.connected === true;
    const firmware = formatMavrosFirmwareVersion(vehicle);
    const vehicleType = vehicle?.type == null ? null : t(`updates.mavrosVehicleTypes.${vehicle.type}`, {
        defaultValue: mavrosVehicleTypeName(vehicle.type) ?? t("updates.mavrosNotReported"),
    });
    return <Card title={t("updates.autopilotFirmware")} size="small" data-testid="mavros-firmware-card">
        <Space direction="vertical" style={{width: "100%"}}>
            <div className="installed-version-heading">
                <Text code>{firmware || t("updates.mavrosVersionUnavailable")}</Text>
                <Tag color={connected ? "success" : "default"}>{t(connected ? "updates.mavrosConnected" : "updates.mavrosDisconnected")}</Tag>
            </div>
            <dl>
                <dt>{t("updates.mavrosProvider")}</dt><dd>{mavrosAutopilotName(vehicle?.autopilot) || t("updates.mavrosNotReported")}</dd>
                <dt>{t("updates.mavrosVehicleType")}</dt><dd>{vehicleType || t("updates.mavrosNotReported")}</dd>
                <dt>{t("updates.mavrosBoardIdentity")}</dt><dd>{formatMavrosBoardIdentity(vehicle) || t("updates.mavrosNotReported")}</dd>
            </dl>
            <Alert type="info" showIcon message={t("updates.mavrosUpdateUnsupported")}/>
        </Space>
    </Card>;
}
