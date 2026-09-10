import {Button, Space, Tag, Typography} from "antd";
import {ReloadOutlined} from "@ant-design/icons";
import {useTranslation} from "react-i18next";
import dayjs from "dayjs";
import {soilVerdict, type SoilStatus} from "../../types/irrisense.ts";

const {Text} = Typography;

interface IrriSenseStatusLineProps {
    status: SoilStatus | null;
    onRefresh: () => void;
}

const VERDICT_TAG: Record<ReturnType<typeof soilVerdict>, "blue" | "green" | "default"> = {
    wet: "blue",
    dry: "green",
    unknown: "default",
};

/** Live verdict, its reason, when it was fetched and the per-zone breakdown. */
export function IrriSenseStatusLine({status, onRefresh}: IrriSenseStatusLineProps) {
    const {t} = useTranslation();
    const verdict = soilVerdict(status);
    const selectedZones = (status?.zones ?? []).filter((z) => z.selected && z.enabled);

    return (
        <Space direction="vertical" size={6} style={{width: "100%"}}>
            <div style={{display: "flex", alignItems: "center", gap: 10, flexWrap: "wrap"}}>
                <Tag color={VERDICT_TAG[verdict]} data-testid="irrisense-verdict" style={{margin: 0}}>
                    {t(`settingsIrriSense.verdict.${verdict}`)}
                </Tag>
                {status?.gardenName && <Text strong>{status.gardenName}</Text>}
                <Text type="secondary" style={{fontSize: 12}}>
                    {status?.fetchedAt
                        ? t("settingsIrriSense.fetchedAt", {value: dayjs(status.fetchedAt).format("YYYY-MM-DD HH:mm")})
                        : t("settingsIrriSense.neverFetched")}
                </Text>
                <Button size="small" icon={<ReloadOutlined/>} onClick={onRefresh} aria-label={t("settingsIrriSense.refresh")}>
                    {t("settingsIrriSense.refresh")}
                </Button>
            </div>
            {status?.reason && (
                <Text style={{fontSize: 12}} data-testid="irrisense-reason">{status.reason}</Text>
            )}
            {status?.error && (
                <Text type="warning" style={{fontSize: 12}}>{status.error}</Text>
            )}
            {selectedZones.length > 0 && (
                <Space size={[6, 6]} wrap>
                    {selectedZones.map((zone) => (
                        <Tag key={zone.id} color={zone.wet ? "blue" : undefined} style={{margin: 0}}>
                            {zone.label}: {zone.deficitMm.toFixed(1)} mm
                        </Tag>
                    ))}
                </Space>
            )}
        </Space>
    );
}
