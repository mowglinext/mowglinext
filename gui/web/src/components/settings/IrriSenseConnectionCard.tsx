import {useState} from "react";
import {Button, Card, Col, Form, Input, Row, Select, Space, Typography} from "antd";
import {ApiOutlined} from "@ant-design/icons";
import {useTranslation} from "react-i18next";
import type {IrriSenseGardenSummary, IrriSenseSettings, IrriSenseSettingsUpdate} from "../../types/irrisense.ts";

const {Text} = Typography;

interface IrriSenseConnectionCardProps {
    settings: IrriSenseSettings;
    draft: IrriSenseSettingsUpdate;
    onChange: (patch: IrriSenseSettingsUpdate) => void;
    gardens: IrriSenseGardenSummary[];
    testing: boolean;
    onTestConnection: () => void;
}

/**
 * Service URL, the write-only token, and the garden / zone pickers. The token
 * field is never pre-filled: the backend only ever returns a masked prefix,
 * so what the operator sees here is "set" or "not set", plus a box to paste a
 * new one into.
 */
export function IrriSenseConnectionCard({
    settings, draft, onChange, gardens, testing, onTestConnection,
}: IrriSenseConnectionCardProps) {
    const {t} = useTranslation();
    const [tokenInput, setTokenInput] = useState("");

    const baseUrl = draft.baseUrl ?? settings.baseUrl;
    const gardenId = draft.gardenId ?? settings.gardenId;
    const zoneIds = draft.zoneIds ?? settings.zoneIds;
    const tokenPending = draft.token !== undefined;
    const tokenCleared = draft.clearToken === true;
    const tokenState = tokenPending
        ? t("settingsIrriSense.tokenPending")
        : tokenCleared
            ? t("settingsIrriSense.tokenCleared")
            : settings.tokenSet
                ? t("settingsIrriSense.tokenSet", {masked: settings.tokenMasked})
                : t("settingsIrriSense.tokenNotSet");

    const selectedGarden = gardens.find((g) => g.id === gardenId);
    const gardenOptions = gardens.map((g) => ({value: g.id, label: `${g.name} (${g.zones.length})`}));
    if (gardenId && !selectedGarden) gardenOptions.push({value: gardenId, label: gardenId});
    const zoneOptions = (selectedGarden?.zones ?? []).map((z) => ({
        value: z.id,
        label: z.enabled ? z.label : t("settingsIrriSense.zoneDisabled", {label: z.label}),
    }));
    for (const id of zoneIds) {
        if (!zoneOptions.some((o) => o.value === id)) zoneOptions.push({value: id, label: id});
    }

    const applyToken = () => {
        const trimmed = tokenInput.trim();
        if (!trimmed) return;
        onChange({token: trimmed, clearToken: undefined});
        setTokenInput("");
    };

    return (
        <Card size="small" title={t("settingsIrriSense.connection")} style={{marginBottom: 16}}>
            <Form layout="vertical" size="small">
                <Form.Item label={t("settingsIrriSense.baseUrl")} tooltip={t("settingsIrriSense.baseUrlTooltip")}>
                    <Input
                        value={baseUrl}
                        onChange={(e) => onChange({baseUrl: e.target.value})}
                        placeholder="https://irrisense-cloud.fly.dev"
                        aria-label={t("settingsIrriSense.baseUrl")}
                    />
                </Form.Item>
                <Form.Item label={t("settingsIrriSense.token")} tooltip={t("settingsIrriSense.tokenTooltip")}>
                    <Space.Compact style={{width: "100%"}}>
                        <Input.Password
                            value={tokenInput}
                            onChange={(e) => setTokenInput(e.target.value)}
                            onPressEnter={applyToken}
                            placeholder={t("settingsIrriSense.tokenPlaceholder")}
                            aria-label={t("settingsIrriSense.token")}
                            autoComplete="off"
                        />
                        <Button onClick={applyToken} disabled={!tokenInput.trim()}>
                            {t("settingsIrriSense.tokenApply")}
                        </Button>
                        <Button
                            danger
                            disabled={!settings.tokenSet && !tokenPending}
                            onClick={() => { onChange({token: undefined, clearToken: true}); setTokenInput(""); }}
                        >
                            {t("settingsIrriSense.tokenClear")}
                        </Button>
                    </Space.Compact>
                    <Text type="secondary" style={{fontSize: 12}} data-testid="irrisense-token-state">{tokenState}</Text>
                </Form.Item>
                <Row gutter={[16, 0]}>
                    <Col xs={24} sm={12}>
                        <Form.Item label={t("settingsIrriSense.garden")} tooltip={t("settingsIrriSense.gardenTooltip")}>
                            <Select
                                value={gardenId || undefined}
                                onChange={(value) => onChange({gardenId: value, zoneIds: []})}
                                options={gardenOptions}
                                placeholder={t("settingsIrriSense.gardenPlaceholder")}
                                aria-label={t("settingsIrriSense.garden")}
                                allowClear
                                style={{width: "100%"}}
                            />
                        </Form.Item>
                    </Col>
                    <Col xs={24} sm={12}>
                        <Form.Item label={t("settingsIrriSense.zones")} tooltip={t("settingsIrriSense.zonesTooltip")}>
                            <Select
                                mode="multiple"
                                value={zoneIds}
                                onChange={(value) => onChange({zoneIds: value})}
                                options={zoneOptions}
                                placeholder={t("settingsIrriSense.zonesPlaceholder")}
                                aria-label={t("settingsIrriSense.zones")}
                                style={{width: "100%"}}
                            />
                        </Form.Item>
                    </Col>
                </Row>
                <Button icon={<ApiOutlined/>} onClick={onTestConnection} loading={testing}>
                    {t("settingsIrriSense.testConnection")}
                </Button>
            </Form>
        </Card>
    );
}
