import {NumberWithAddon} from '../inputs/NumberWithAddon';
import React from "react";
import { Card, Col, Form, InputNumber, Row, Space, Switch, Tag, Typography } from "antd";
import { CloudOutlined } from "@ant-design/icons";
import { useTranslation } from "react-i18next";
import { useThemeMode } from "../../theme/ThemeContext.tsx";
import type {SettingsValues} from "../../types/settings.generated";
import { useExternalRainState, type ExternalRainState } from "../../hooks/useExternalRainState.ts";

const { Text, Paragraph } = Typography;

type Props = {
    values: SettingsValues;
    onChange: (key: string, value: number | boolean | null) => void;
};

const RAIN_MODES = [
    { value: 0, labelKey: "rainModeIgnoreLabel", descriptionKey: "rainModeIgnoreDescription", color: "#8c8c8c" },
    { value: 1, labelKey: "rainModeDockLabel", descriptionKey: "rainModeDockDescription", color: "#1890ff" },
    { value: 2, labelKey: "rainModeDockUntilDryLabel", descriptionKey: "rainModeDockUntilDryDescription", color: "#13c2c2" },
    { value: 3, labelKey: "rainModePauseAutoLabel", descriptionKey: "rainModePauseAutoDescription", color: "#722ed1" },
];

const EXTERNAL_RAIN_COLORS: Record<string, string> = {
    wet: "blue",
    dry: "green",
    expired: "orange",
    disabled: "default",
    no_data: "default",
};

const formatAge = (seconds: number | null | undefined): string => {
    if (seconds === null || seconds === undefined) return "";
    if (seconds < 90) return `${Math.max(0, Math.round(seconds))} s`;
    if (seconds < 5400) return `${Math.round(seconds / 60)} min`;
    return `${(seconds / 3600).toFixed(1)} h`;
};

/** One line of feedback: did the external verdict arrive, and is the mower counting it. */
const ExternalRainStatusLine: React.FC<{ state: ExternalRainState | null }> = ({ state }) => {
    const { t } = useTranslation();
    const key = state?.state ?? "no_data";
    return (
        <div style={{ marginBottom: 12 }} data-testid="external-rain-status">
            <Space size={8} wrap>
                <Text strong>{t("settingsRain.externalStatus")}</Text>
                <Tag color={EXTERNAL_RAIN_COLORS[key] ?? "default"}>{t(`settingsRain.externalState_${key}`)}</Tag>
                {state?.received && (
                    <Text type="secondary">{t("settingsRain.externalAge", { age: formatAge(state.age_s) })}</Text>
                )}
                {state?.active && state.expires_in_s !== null && state.expires_in_s !== undefined && (
                    <Text type="secondary">{t("settingsRain.externalExpiresIn", { age: formatAge(state.expires_in_s) })}</Text>
                )}
                {state?.onboard_wet && <Tag color="blue">{t("settingsRain.ownSensorWet")}</Tag>}
            </Space>
        </div>
    );
};

export const RainSection: React.FC<Props> = ({ values, onChange }) => {
    const { t } = useTranslation();
    const { colors } = useThemeMode();
    const externalRainState = useExternalRainState();
    const currentMode = values.rain_mode ?? 2;
    // Narrowed (not read as `any`), so these two fields add no lint warnings.
    const externalEnabled: boolean =
        typeof values.use_external_rain === "boolean" ? values.use_external_rain : true;
    const externalExpiry: number | undefined =
        typeof values.external_rain_max_age_minutes === "number"
            ? values.external_rain_max_age_minutes
            : undefined;

    return (
        <div>
            <Card size="small" style={{ marginBottom: 16 }}>
                <Space direction="vertical" size={12} style={{ width: "100%" }}>
                    <div>
                        <Text strong style={{ fontSize: 14 }}>
                            <CloudOutlined style={{ marginRight: 6 }} />
                            {t("settingsRain.rainBehavior")}
                        </Text>
                        <Paragraph type="secondary" style={{ margin: "4px 0 0" }}>
                            {t("settingsRain.rainBehaviorDescription")}
                        </Paragraph>
                    </div>

                    <Row gutter={[8, 8]}>
                        {RAIN_MODES.map((mode) => {
                            const isSelected = currentMode === mode.value;
                            return (
                                <Col xs={12} sm={6} key={mode.value}>
                                    <Card
                                        hoverable
                                        size="small"
                                        onClick={() => onChange("rain_mode", mode.value)}
                                        style={{
                                            border: isSelected
                                                ? `2px solid ${mode.color}`
                                                : `1px solid ${colors.border}`,
                                            background: isSelected ? `${mode.color}10` : undefined,
                                            height: "100%",
                                            cursor: "pointer",
                                        }}
                                        styles={{ body: { padding: "8px 10px" } }}
                                    >
                                        <Text strong style={{ fontSize: 12, color: isSelected ? mode.color : undefined }}>
                                            {t(`settingsRain.${mode.labelKey}`)}
                                        </Text>
                                        <br />
                                        <Text type="secondary" style={{ fontSize: 11 }}>
                                            {t(`settingsRain.${mode.descriptionKey}`)}
                                        </Text>
                                    </Card>
                                </Col>
                            );
                        })}
                    </Row>
                </Space>
            </Card>

            {currentMode > 0 && (
                <Card size="small" title={t("settingsRain.timing")} style={{ marginBottom: 16 }}>
                    <Form layout="vertical" size="small">
                        <Row gutter={[16, 0]}>
                            <Col xs={12}>
                                <Form.Item htmlFor="setting-rain_delay_minutes" data-setting-key="rain_delay_minutes" label={t("settingsRain.resumeDelay")} tooltip={t("settingsRain.resumeDelayTooltip")}>
                                    <NumberWithAddon aria-label={t("settingsRain.resumeDelay") + ", min"} aria-description={t("settingsRain.resumeDelayTooltip")}  id="setting-rain_delay_minutes"
                                        value={values.rain_delay_minutes}
                                        onChange={(v) => onChange("rain_delay_minutes", v)}
                                        min={0} max={240} step={5} precision={0}
                                        style={{ width: "100%" }} after="min"
                                    />
                                </Form.Item>
                            </Col>
                            <Col xs={12}>
                                <Form.Item htmlFor="setting-rain_debounce_sec" data-setting-key="rain_debounce_sec" label={t("settingsRain.debounce")} tooltip={t("settingsRain.debounceTooltip")}>
                                    <NumberWithAddon aria-label={t("settingsRain.debounce") + ", s"} aria-description={t("settingsRain.debounceTooltip")}  id="setting-rain_debounce_sec"
                                        value={values.rain_debounce_sec}
                                        onChange={(v) => onChange("rain_debounce_sec", v)}
                                        min={1} max={60} step={5} precision={0}
                                        style={{ width: "100%" }} after="s"
                                    />
                                </Form.Item>
                            </Col>
                        </Row>
                    </Form>
                </Card>
            )}

            {currentMode > 0 && (
                <Card size="small" title={t("settingsRain.externalSource")} style={{ marginBottom: 16 }}>
                    <Paragraph type="secondary" style={{ marginTop: 0 }}>
                        {t("settingsRain.externalSourceDescription")}
                    </Paragraph>
                    <ExternalRainStatusLine state={externalRainState} />
                    <Form layout="vertical" size="small">
                        <Row gutter={[16, 0]}>
                            <Col xs={12}>
                                <Form.Item htmlFor="setting-use_external_rain" data-setting-key="use_external_rain" label={t("settingsRain.useExternal")} tooltip={t("settingsRain.useExternalTooltip")}>
                                    <Switch
                                        id="setting-use_external_rain"
                                        aria-label={t("settingsRain.useExternal")}
                                        checked={externalEnabled}
                                        onChange={(v) => onChange("use_external_rain", v)}
                                    />
                                </Form.Item>
                            </Col>
                            <Col xs={12}>
                                <Form.Item htmlFor="setting-external_rain_max_age_minutes" data-setting-key="external_rain_max_age_minutes" label={t("settingsRain.externalExpiry")} tooltip={t("settingsRain.externalExpiryTooltip")}>
                                    <InputNumber aria-label={t("settingsRain.externalExpiry") + ", min"} aria-description={t("settingsRain.externalExpiryTooltip")} id="setting-external_rain_max_age_minutes"
                                        value={externalExpiry}
                                        onChange={(v) => onChange("external_rain_max_age_minutes", v)}
                                        disabled={!externalEnabled}
                                        min={0} max={1440} step={15} precision={0}
                                        style={{ width: "100%" }} addonAfter="min"
                                    />
                                </Form.Item>
                            </Col>
                        </Row>
                    </Form>
                </Card>
            )}
        </div>
    );
};
