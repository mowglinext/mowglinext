import {NumberWithAddon} from '../inputs/NumberWithAddon';
import {useMeasurementEditor} from "../robot/useMeasurementEditor";
import {DOCK_VISUAL_DEFAULTS} from "../robot/dockGeometry";
import {resolveCasterPosition} from "../../utils/casterPosition";
import {DEFAULT_MOWER_VISUAL} from "../../hooks/useMowerVisual";
import {MowerPreview} from "../robot/MowerPreview";
import React, { useState } from "react";
import { App, Card, Col, Form, Input, InputNumber, Row, Space, Tag, Typography } from "antd";
import { ToolOutlined, DownOutlined, UpOutlined } from "@ant-design/icons";
import { useTranslation } from "react-i18next";
import { useThemeMode } from "../../theme/ThemeContext.tsx";
import { MOWER_MODELS } from "../../constants/mowerModels.ts";
import { presetValuesForBackend } from "../../constants/hardwareBackends.ts";
import { SettingFieldLabel } from "./SettingFieldLabel.tsx";
import type {SettingsValues} from "../../types/settings.generated";

const { Text, Paragraph } = Typography;

type Props = {
    revealAdvanced?: boolean;
    values: SettingsValues;
    onChange: (key: string, value: any) => void;
    onBulkChange: (changes: Record<string, any>) => void;
    /** Settings whose default the robot's hardware backend replaces. */
    backendConfirmed?: boolean;
    backendDefaultOverrides?: Record<string, unknown>;
    isOverridden?: (key: string) => boolean;
    hasDefault?: (key: string) => boolean;
    onReset?: (key: string) => void;
};

export const HardwareSection: React.FC<Props> = ({
    values,
    revealAdvanced = false,
    onChange,
    onBulkChange,
    backendDefaultOverrides = {},
    backendConfirmed = false,
    isOverridden,
    hasDefault,
    onReset,
}) => {
    const { t } = useTranslation();
    const { colors } = useThemeMode();
    const { modal } = App.useApp();
    const [showAdvanced, setShowAdvanced] = useState(false);
    const measurementEditor = useMeasurementEditor(values,onChange);
    const selectedModel = values.mower_model || "YardForce500";

    const fieldLabel = (key: string, label: React.ReactNode) => (
        <SettingFieldLabel
            settingKey={key}
            label={label}
            overridden={isOverridden?.(key) ?? false}
            canReset={hasDefault?.(key) ?? false}
            onReset={onReset}
        />
    );

    const applyModelPreset = (model: string) => {
        if (!backendConfirmed) return;
        onChange("mower_model", model);
        const preset = MOWER_MODELS.find((m) => m.value === model);
        onChange("mower_visual_style", (preset?.appearance ?? DEFAULT_MOWER_VISUAL).style);
        if (preset?.defaults && Object.keys(preset.defaults).length > 0) {
            onBulkChange(presetValuesForBackend(preset.defaults, backendDefaultOverrides));
        }
    };

    const handleModelSelect = (model: string) => {
        if (!backendConfirmed) return;
        if (model === selectedModel) {
            return;
        }
        modal.confirm({
            title: t("settingsHardware.modelConfirmTitle"),
            content: t("settingsHardware.modelConfirmBody"),
            okText: t("settingsHardware.modelConfirmOk"),
            cancelText: t("settingsHardware.modelConfirmCancel"),
            onOk: () => applyModelPreset(model),
        });
    };

    return (
        <div className="hardware-settings">
            {/* Identity: the name the fleet view and the GUI show for this mower */}
            <Card size="small" style={{ marginBottom: 16 }}>
                <Form layout="vertical" size="small">
                    <Form.Item htmlFor="setting-robot_name" data-setting-key="robot_name"
                        label={fieldLabel("robot_name", t("settingsHardware.robotName"))}
                        tooltip={t("settingsHardware.robotNameTooltip")}
                        style={{ marginBottom: 0 }}
                    >
                        <Input id="setting-robot_name"
                            value={values.robot_name ?? ""}
                            onChange={(e) => onChange("robot_name", e.target.value)}
                            maxLength={32}
                            placeholder="mowgli"
                            style={{ maxWidth: 320 }}
                        />
                    </Form.Item>
                </Form>
            </Card>

            {/* Model selection */}
            <Card size="small" style={{ marginBottom: 16 }}>
                <Space direction="vertical" size={12} style={{ width: "100%" }}>
                    <div>
                        <Text strong style={{ fontSize: 14 }}>
                            <ToolOutlined style={{ marginRight: 6 }} />
                            {t("settingsHardware.robotModel")}
                        </Text>
                        <Paragraph type="secondary" style={{ margin: "4px 0 0" }}>
                            {t("settingsHardware.robotModelDescription")}
                        </Paragraph>
                    </div>
                    <Row gutter={[8, 8]} role="radiogroup" aria-label={t("settingsHardware.robotModel")}>
                        {MOWER_MODELS.map((model) => {
                            const isSelected = selectedModel === model.value;
                            return (
                                <Col xs={24} sm={12} lg={8} key={model.value}>
                                    <Card
                                        hoverable
                                        size="small"
                                        role="radio"
                                        tabIndex={0}
                                        aria-disabled={!backendConfirmed}
                                        aria-checked={isSelected}
                                        aria-label={t(model.label)}
                                        onKeyDown={(event) => {
                                            if (event.key === " " || event.key === "Enter") {
                                                event.preventDefault();
                                                handleModelSelect(model.value);
                                            }
                                        }}
                                        onClick={() => handleModelSelect(model.value)}
                                        style={{
                                            border: isSelected
                                                ? `2px solid ${colors.primary}`
                                                : `1px solid ${colors.border}`,
                                            background: isSelected ? colors.primaryBg : undefined,
                                            height: "100%",
                                            cursor: "pointer",
                                        }}
                                        styles={{ body: { padding: "8px 12px" } }}
                                    >
                                        <Space direction="vertical" size={2} style={{ width: "100%" }}>
                                            <Space size={4}>
                                                <Text strong style={{ fontSize: 14, wordBreak: "normal" }}>{isSelected ? "✓ " : ""}{t(model.label)}</Text>
                                            </Space>
                                            {model.tag && <Tag color="green">{t(model.tag)}</Tag>}
                                            <Text type="secondary" style={{ fontSize: 11 }}>
                                                {t(model.description)}
                                            </Text>
                                        </Space>
                                    </Card>
                                </Col>
                            );
                        })}
                    </Row>
                </Space>
            </Card>

            <MowerPreview values={values} editor={measurementEditor} onStyleChange={style => onChange("mower_visual_style", style)} onColourChange={colour => onChange("mower_visual_colour", colour)}/>

            {/* Essential parameters (always visible) */}
            <Card
                size="small"
                title={t("settingsHardware.wheelsAndBlade")}
                style={{ marginBottom: 16 }}
            >
                <Form layout="vertical" size="small">
                    <Row gutter={[16, 0]}>
                        <Col xs={12} sm={8} lg={6}>
                            <Form.Item htmlFor="setting-wheel_radius" data-setting-key="wheel_radius" label={fieldLabel("wheel_radius", t("settingsHardware.wheelRadius"))} tooltip={t("settingsHardware.wheelRadiusTooltip")}>
                                <NumberWithAddon {...measurementEditor.fieldProps("wheel_radius")} aria-label={t("settingsHardware.wheelRadius") + ", m"} aria-description={t("settingsHardware.wheelRadiusTooltip")}  id="setting-wheel_radius"
                                    value={values.wheel_radius}
                                    onChange={(v) => onChange("wheel_radius", v)}
                                    step={0.001} precision={5} style={{ width: "100%" }}
                                    after="m"
                                />
                            </Form.Item>
                        </Col>
                        <Col xs={12} sm={8} lg={6}>
                            <Form.Item htmlFor="setting-wheel_track" data-setting-key="wheel_track" label={fieldLabel("wheel_track", t("settingsHardware.wheelTrack"))} tooltip={t("settingsHardware.wheelTrackTooltip")}>
                                <NumberWithAddon {...measurementEditor.fieldProps("wheel_track")} aria-label={t("settingsHardware.wheelTrack") + ", m"} aria-description={t("settingsHardware.wheelTrackTooltip")}  id="setting-wheel_track"
                                    value={values.wheel_track}
                                    onChange={(v) => onChange("wheel_track", v)}
                                    step={0.005} precision={3} style={{ width: "100%" }}
                                    after="m"
                                />
                            </Form.Item>
                        </Col>
                        <Col xs={12} sm={8} lg={6}>
                            <Form.Item htmlFor="setting-blade_radius" data-setting-key="blade_radius" label={fieldLabel("blade_radius", t("settingsHardware.bladeRadius"))} tooltip={t("settingsHardware.bladeRadiusTooltip")}>
                                <NumberWithAddon {...measurementEditor.fieldProps("blade_radius")} aria-label={t("settingsHardware.bladeRadius") + ", m"} aria-description={t("settingsHardware.bladeRadiusTooltip")}  id="setting-blade_radius"
                                    value={values.blade_radius}
                                    onChange={(v) => onChange("blade_radius", v)}
                                    step={0.01} precision={3} style={{ width: "100%" }}
                                    after="m"
                                />
                            </Form.Item>
                        </Col>
                        <Col xs={12} sm={8} lg={6}>
                            <Form.Item htmlFor="setting-tool_width" data-setting-key="tool_width" label={fieldLabel("tool_width", t("settingsHardware.toolWidth"))} tooltip={t("settingsHardware.toolWidthTooltip")}>
                                <NumberWithAddon {...measurementEditor.fieldProps("tool_width")} aria-label={t("settingsHardware.toolWidth") + ", m"} aria-description={t("settingsHardware.toolWidthTooltip")}  id="setting-tool_width"
                                    value={values.tool_width}
                                    onChange={(v) => onChange("tool_width", v)}
                                    step={0.01} precision={3} style={{ width: "100%" }}
                                    after="m"
                                />
                            </Form.Item>
                        </Col>
                        <Col xs={12} sm={8} lg={6}>
                            <Form.Item htmlFor="setting-ticks_per_meter" data-setting-key="ticks_per_meter" label={fieldLabel("ticks_per_meter", t("settingsHardware.encoderTicksPerMeter"))} tooltip={t("settingsHardware.encoderTicksPerMeterTooltip")}>
                                <InputNumber aria-label={t("settingsHardware.encoderTicksPerMeter")} aria-description={t("settingsHardware.encoderTicksPerMeterTooltip")}  id="setting-ticks_per_meter"
                                    value={values.ticks_per_meter}
                                    onChange={(v) => onChange("ticks_per_meter", v)}
                                    step={0.001} style={{ width: "100%" }}
                                />
                            </Form.Item>
                        </Col>
                    </Row>
                </Form>
            </Card>

            {/* Advanced: chassis dimensions */}
            <Card
                size="small"
                title={
                    <Space
                        style={{ cursor: "pointer", userSelect: "none" }}
                        onClick={() => setShowAdvanced(!showAdvanced)}
                    >
                        <span>{t("settingsHardware.chassisAndGeometry")}</span>
                        <Tag color="default" style={{ fontSize: 10 }}>{t("settingsHardware.advanced")}</Tag>
                        {(showAdvanced || revealAdvanced) ? <UpOutlined style={{ fontSize: 10 }} /> : <DownOutlined style={{ fontSize: 10 }} />}
                    </Space>
                }
                style={{ marginBottom: 16 }}
                styles={{ body: { display: (showAdvanced || revealAdvanced) ? undefined : "none" } }}
            >
                <Paragraph type="secondary">{t("settingsHardware.geometryMeasurementHelp")}</Paragraph>
                <Form layout="vertical" size="small">
                    <Row gutter={[16, 0]}>
                        <Col xs={12} sm={8} lg={6}>
                            <Form.Item htmlFor="setting-chassis_length" data-setting-key="chassis_length" label={fieldLabel("chassis_length", t("settingsHardware.chassisLength"))} tooltip={t("settingsHardware.chassisLengthTooltip")}>
                                <NumberWithAddon {...measurementEditor.fieldProps("chassis_length")} aria-label={t("settingsHardware.chassisLength") + ", m"} aria-description={t("settingsHardware.chassisLengthTooltip")} id="setting-chassis_length"
                                    value={values.chassis_length}
                                    onChange={(v) => onChange("chassis_length", v)}
                                    step={0.01} precision={3} style={{ width: "100%" }}
                                    after="m"
                                />
                            </Form.Item>
                        </Col>
                        <Col xs={12} sm={8} lg={6}>
                            <Form.Item htmlFor="setting-chassis_width" data-setting-key="chassis_width" label={fieldLabel("chassis_width", t("settingsHardware.chassisWidth"))} tooltip={t("settingsHardware.chassisWidthTooltip")}>
                                <NumberWithAddon {...measurementEditor.fieldProps("chassis_width")} aria-label={t("settingsHardware.chassisWidth") + ", m"} aria-description={t("settingsHardware.chassisWidthTooltip")} id="setting-chassis_width"
                                    value={values.chassis_width}
                                    onChange={(v) => onChange("chassis_width", v)}
                                    step={0.01} precision={3} style={{ width: "100%" }}
                                    after="m"
                                />
                            </Form.Item>
                        </Col>
                        <Col xs={12} sm={8} lg={6}>
                            <Form.Item htmlFor="setting-chassis_height" data-setting-key="chassis_height" label={fieldLabel("chassis_height", t("settingsHardware.chassisHeight"))} tooltip={t("settingsHardware.chassisHeightTooltip")}>
                                <NumberWithAddon {...measurementEditor.fieldProps("chassis_height")} aria-label={t("settingsHardware.chassisHeight") + ", m"} aria-description={t("settingsHardware.chassisHeightTooltip")} id="setting-chassis_height"
                                    value={values.chassis_height}
                                    onChange={(v) => onChange("chassis_height", v)}
                                    step={0.01} precision={3} style={{ width: "100%" }}
                                    after="m"
                                />
                            </Form.Item>
                        </Col>
                        <Col xs={12} sm={8} lg={6}>
                            <Form.Item htmlFor="setting-chassis_z_offset" data-setting-key="chassis_z_offset"
                                label={fieldLabel("chassis_z_offset", t("settingsHardware.chassisZOffset"))}
                                tooltip={t("settingsHardware.chassisZOffsetTooltip")}>
                                <NumberWithAddon {...measurementEditor.fieldProps("chassis_z_offset")} id="setting-chassis_z_offset" aria-label={t("settingsHardware.chassisZOffset") + ", m"}
                                    aria-description={t("settingsHardware.chassisZOffsetTooltip")}
                                    value={values.chassis_z_offset == null ? undefined : Number(values.chassis_z_offset)}
                                    onChange={v=>{if(v != null) onChange("chassis_z_offset",v);}}
                                    min={-.3} max={.3} step={.005} precision={3} style={{width:"100%"}} after="m"/>
                            </Form.Item>
                        </Col>
                        <Col xs={12} sm={8} lg={6}>
                            <Form.Item htmlFor="setting-chassis_center_x" data-setting-key="chassis_center_x" label={fieldLabel("chassis_center_x", t("settingsHardware.chassisCenterX"))} tooltip={t("settingsHardware.chassisCenterXTooltip")}>
                                <NumberWithAddon {...measurementEditor.fieldProps("chassis_center_x")} aria-label={t("settingsHardware.chassisCenterX") + ", m"} aria-description={t("settingsHardware.chassisCenterXTooltip")}  id="setting-chassis_center_x"
                                    value={values.chassis_center_x}
                                    onChange={(v) => onChange("chassis_center_x", v)}
                                    step={0.01} precision={3} style={{ width: "100%" }}
                                    after="m"
                                />
                            </Form.Item>
                        </Col>
                        <Col xs={12} sm={8} lg={6}>
                            <Form.Item htmlFor="setting-chassis_mass_kg" data-setting-key="chassis_mass_kg" label={fieldLabel("chassis_mass_kg", t("settingsHardware.mass"))} tooltip={t("settingsHardware.massTooltip")}>
                                <NumberWithAddon aria-label={t("settingsHardware.mass") + ", kg"} aria-description={t("settingsHardware.massTooltip")} id="setting-chassis_mass_kg"
                                    value={values.chassis_mass_kg}
                                    onChange={(v) => onChange("chassis_mass_kg", v)}
                                    step={0.5} precision={2} style={{ width: "100%" }}
                                    after="kg"
                                />
                            </Form.Item>
                        </Col>
                        <Col xs={12} sm={8} lg={6}>
                            <Form.Item htmlFor="setting-wheel_width" data-setting-key="wheel_width" label={fieldLabel("wheel_width", t("settingsHardware.wheelWidth"))} tooltip={t("settingsHardware.wheelWidthTooltip")}>
                                <NumberWithAddon {...measurementEditor.fieldProps("wheel_width")} aria-label={t("settingsHardware.wheelWidth") + ", m"} aria-description={t("settingsHardware.wheelWidthTooltip")} id="setting-wheel_width"
                                    value={values.wheel_width}
                                    onChange={(v) => onChange("wheel_width", v)}
                                    step={0.005} precision={3} style={{ width: "100%" }}
                                    after="m"
                                />
                            </Form.Item>
                        </Col>
                        <Col xs={12} sm={8} lg={6}>
                            <Form.Item htmlFor="setting-wheel_x_offset" data-setting-key="wheel_x_offset" label={fieldLabel("wheel_x_offset", t("settingsHardware.wheelXOffset"))} tooltip={t("settingsHardware.wheelXOffsetTooltip")}>
                                <NumberWithAddon {...measurementEditor.fieldProps("wheel_x_offset")} aria-label={t("settingsHardware.wheelXOffset") + ", m"} aria-description={t("settingsHardware.wheelXOffsetTooltip")}  id="setting-wheel_x_offset"
                                    value={values.wheel_x_offset}
                                    onChange={(v) => onChange("wheel_x_offset", v)}
                                    step={0.01} precision={3} style={{ width: "100%" }}
                                    after="m"
                                />
                            </Form.Item>
                        </Col>
                        <Col xs={12} sm={8} lg={6}>
                            <Form.Item htmlFor="setting-blade_x_offset" data-setting-key="blade_x_offset" label={fieldLabel("blade_x_offset", t("settingsHardware.bladeXOffset"))} tooltip={t("settingsHardware.bladeXOffsetTooltip")}>
                                <NumberWithAddon {...measurementEditor.fieldProps("blade_x_offset")} id="setting-blade_x_offset"
                                    aria-label={t("settingsHardware.bladeXOffset") + ", m"}
                                    value={typeof values.blade_x_offset === "number" ? values.blade_x_offset : 0} onChange={v => onChange("blade_x_offset", v)}
                                    step={0.001} precision={4} style={{width: "100%"}} after="m"/>
                            </Form.Item>
                        </Col>
                        <Col xs={12} sm={8} lg={6}>
                            <Form.Item htmlFor="setting-blade_y_offset" data-setting-key="blade_y_offset" label={fieldLabel("blade_y_offset", t("settingsHardware.bladeYOffset"))} tooltip={t("settingsHardware.bladeYOffsetTooltip")}>
                                <NumberWithAddon {...measurementEditor.fieldProps("blade_y_offset")} id="setting-blade_y_offset"
                                    aria-label={t("settingsHardware.bladeYOffset") + ", m"}
                                    value={typeof values.blade_y_offset === "number" ? values.blade_y_offset : 0} onChange={v => onChange("blade_y_offset", v)}
                                    step={0.001} precision={4} style={{width: "100%"}} after="m"/>
                            </Form.Item>
                        </Col>
                        <Col xs={12} sm={8} lg={6}>
                            <Form.Item htmlFor="setting-caster_radius" data-setting-key="caster_radius" label={fieldLabel("caster_radius", t("settingsHardware.casterRadius"))} tooltip={t("settingsHardware.casterRadiusTooltip")}>
                                <NumberWithAddon {...measurementEditor.fieldProps("caster_radius")} aria-label={t("settingsHardware.casterRadius") + ", m"} aria-description={t("settingsHardware.casterRadiusTooltip")} id="setting-caster_radius"
                                    value={values.caster_radius}
                                    onChange={(v) => onChange("caster_radius", v)}
                                    step={0.005} precision={3} style={{ width: "100%" }}
                                    after="m"
                                />
                            </Form.Item>
                        </Col>
                        <Col xs={12} sm={8} lg={6}>
                            <Form.Item htmlFor="setting-caster_track" data-setting-key="caster_track" label={fieldLabel("caster_track", t("settingsHardware.casterTrack"))} tooltip={t("settingsHardware.casterTrackTooltip")}>
                                <NumberWithAddon {...measurementEditor.fieldProps("caster_track")} aria-label={t("settingsHardware.casterTrack") + ", m"} aria-description={t("settingsHardware.casterTrackTooltip")} id="setting-caster_track"
                                    value={values.caster_track}
                                    onChange={(v) => onChange("caster_track", v)}
                                    step={0.01} precision={3} style={{ width: "100%" }}
                                    after="m"
                                />
                            </Form.Item>
                        </Col>
                        <Col xs={12} sm={8} lg={6}>
                            <Form.Item htmlFor="setting-caster_x_offset" data-setting-key="caster_x_offset"
                                label={fieldLabel("caster_x_offset", t("settingsHardware.casterXOffset"))}
                                tooltip={t("settingsHardware.casterXOffsetTooltip")}>
                                <NumberWithAddon {...measurementEditor.fieldProps("caster_x_offset")} id="setting-caster_x_offset" aria-label={t("settingsHardware.casterXOffset") + ", m"}
                                    aria-description={t("settingsHardware.casterXOffsetTooltip")}
                                    value={resolveCasterPosition(values)} min={-.99} max={2} step={.01} precision={3}
                                    onChange={v=>{if(v != null) onChange("caster_x_offset",v);}}
                                    style={{width:"100%"}} after="m"/>
                            </Form.Item>
                        </Col>
                    </Row>
                    <Typography.Title level={5}>{t("settingsHardware.dockVisualTitle")}</Typography.Title>
                    <Paragraph type="secondary">{t("settingsHardware.dockVisualHelp")}</Paragraph>
                    <Row gutter={[16, 0]}>
                        {(["dock_visual_length", "dock_visual_width", "dock_visual_x_offset", "dock_visual_y_offset"] as const).map(key => (
                            <Col xs={12} sm={8} lg={6} key={key}>
                                <Form.Item htmlFor={`setting-${key}`} data-setting-key={key}
                                    label={fieldLabel(key, t(`settingsHardware.${key}`))}
                                    tooltip={t(`settingsHardware.${key}Tooltip`)}>
                                    <NumberWithAddon {...measurementEditor.fieldProps(key)} id={`setting-${key}`} aria-label={t(`settingsHardware.${key}`) + ", m"}
                                        aria-description={t(`settingsHardware.${key}Tooltip`)}
                                        value={values[key] ?? DOCK_VISUAL_DEFAULTS[key]}
                                        min={key.endsWith("_offset") ? -2 : .1}
                                        max={key.endsWith("_offset") ? 2 : 3}
                                        step={.01} precision={3} style={{width:"100%"}} after="m"
                                        onChange={v => {if (v != null) onChange(key, v);}}/>
                                </Form.Item>
                            </Col>
                        ))}
                    </Row>
                </Form>
            </Card>
        </div>
    );
};
