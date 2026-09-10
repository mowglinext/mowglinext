import {Card, Col, Form, InputNumber, Row, Switch, Typography} from "antd";
import {useTranslation} from "react-i18next";
import type {IrriSenseSettings, IrriSenseSettingsUpdate} from "../../types/irrisense.ts";

const {Paragraph} = Typography;

interface IrriSenseRuleCardProps {
    settings: IrriSenseSettings;
    draft: IrriSenseSettingsUpdate;
    onChange: (patch: IrriSenseSettingsUpdate) => void;
}

/** The three thresholds of the wetness rule and the scheduler gate switch. */
export function IrriSenseRuleCard({settings, draft, onChange}: IrriSenseRuleCardProps) {
    const {t} = useTranslation();
    const wetDeficitMm = draft.wetDeficitMm ?? settings.wetDeficitMm;
    const dryAfterWateringHours = draft.dryAfterWateringHours ?? settings.dryAfterWateringHours;
    const maxStaleMinutes = draft.maxStaleMinutes ?? settings.maxStaleMinutes;
    const gateScheduler = draft.gateScheduler ?? settings.gateScheduler;

    return (
        <Card size="small" title={t("settingsIrriSense.rule")} style={{marginBottom: 16}}>
            <Paragraph type="secondary" style={{margin: "0 0 12px", fontSize: 12}}>
                {t("settingsIrriSense.ruleDescription")}
            </Paragraph>
            <Form layout="vertical" size="small">
                <Row gutter={[16, 0]}>
                    <Col xs={24} sm={8}>
                        <Form.Item label={t("settingsIrriSense.wetDeficit")} tooltip={t("settingsIrriSense.wetDeficitTooltip")}>
                            <InputNumber
                                value={wetDeficitMm}
                                onChange={(v) => onChange({wetDeficitMm: v ?? 0})}
                                min={0} step={0.5} addonAfter="mm" style={{width: "100%"}}
                                aria-label={t("settingsIrriSense.wetDeficit")}
                            />
                        </Form.Item>
                    </Col>
                    <Col xs={24} sm={8}>
                        <Form.Item label={t("settingsIrriSense.dryAfterWatering")} tooltip={t("settingsIrriSense.dryAfterWateringTooltip")}>
                            <InputNumber
                                value={dryAfterWateringHours}
                                onChange={(v) => onChange({dryAfterWateringHours: v ?? 0})}
                                min={0} step={0.5} addonAfter="h" style={{width: "100%"}}
                                aria-label={t("settingsIrriSense.dryAfterWatering")}
                            />
                        </Form.Item>
                    </Col>
                    <Col xs={24} sm={8}>
                        <Form.Item label={t("settingsIrriSense.maxStale")} tooltip={t("settingsIrriSense.maxStaleTooltip")}>
                            <InputNumber
                                value={maxStaleMinutes}
                                onChange={(v) => onChange({maxStaleMinutes: v ?? 1})}
                                min={1} step={10} addonAfter="min" style={{width: "100%"}}
                                aria-label={t("settingsIrriSense.maxStale")}
                            />
                        </Form.Item>
                    </Col>
                </Row>
                <div style={{display: "flex", justifyContent: "space-between", alignItems: "center", gap: 12}}>
                    <div>
                        <div style={{fontWeight: 600}}>{t("settingsIrriSense.gateScheduler")}</div>
                        <Paragraph type="secondary" style={{margin: "2px 0 0", fontSize: 12}}>
                            {t("settingsIrriSense.gateSchedulerDescription")}
                        </Paragraph>
                    </div>
                    <Switch
                        checked={gateScheduler}
                        onChange={(checked) => onChange({gateScheduler: checked})}
                        aria-label={t("settingsIrriSense.gateScheduler")}
                    />
                </div>
            </Form>
        </Card>
    );
}
