import React from "react";
import { Alert, Card, Col, Form, InputNumber, Row, Select, Switch, Typography } from "antd";
import { useTranslation } from "react-i18next";
import { SettingFieldLabel } from "./SettingFieldLabel.tsx";
import type { SettingsFieldGroup, SettingsFieldSpec } from "./settingsFieldGroups.ts";

const { Paragraph } = Typography;

type SettingValue = string | number | boolean | null | undefined;

export interface SettingsFieldCardProps {
    group: SettingsFieldGroup;
    values: Record<string, SettingValue>;
    onChange: (key: string, value: SettingValue) => void;
    isOverridden?: (key: string) => boolean;
    hasDefault?: (key: string) => boolean;
    onReset?: (key: string) => void;
}

const asBool = (value: SettingValue): boolean => value === true || value === "true";

/**
 * Renders one declarative group of mowgli_robot.yaml settings (see
 * settingsFieldGroups.ts) with the same default-awareness affordances as the
 * hand-written sections: overridden dot + reset-to-default on every label.
 */
export const SettingsFieldCard: React.FC<SettingsFieldCardProps> = ({
    group,
    values,
    onChange,
    isOverridden,
    hasDefault,
    onReset,
}) => {
    const { t } = useTranslation();

    const renderInput = (field: SettingsFieldSpec) => {
        const value = values[field.key];
        const label = t(`settingsFields.${field.key}.label`);
        switch (field.kind) {
            case "switch":
                return (
                    <Switch
                        checked={asBool(value)}
                        onChange={(checked) => onChange(field.key, checked)}
                        aria-label={label}
                    />
                );
            case "select":
                return (
                    <Select
                        value={typeof value === "number" ? value : undefined}
                        onChange={(next: number) => onChange(field.key, next)}
                        options={field.options.map((option) => ({
                            value: option,
                            label: option > 0 ? `+${option}` : `${option}`,
                        }))}
                        aria-label={label}
                        style={{ width: "100%" }}
                    />
                );
            case "number":
                return (
                    <InputNumber
                        value={typeof value === "number" ? value : null}
                        onChange={(next) => onChange(field.key, next)}
                        min={field.min}
                        max={field.max}
                        step={field.step}
                        precision={field.precision}
                        addonAfter={field.unit}
                        aria-label={label}
                        style={{ width: "100%" }}
                    />
                );
        }
    };

    return (
        <Card
            size="small"
            title={t(`settingsFieldGroups.${group.id}.title`)}
            style={{ marginTop: 16, marginBottom: 16 }}
            data-testid={`settings-field-group-${group.id}`}
        >
            <Paragraph type="secondary" style={{ margin: "0 0 12px", fontSize: 12 }}>
                {t(`settingsFieldGroups.${group.id}.description`)}
            </Paragraph>
            {group.isSafetyRelevant ? (
                <Alert
                    type="warning"
                    showIcon
                    style={{ marginBottom: 12 }}
                    message={t("settingsFieldGroups.safetyWarning")}
                />
            ) : null}
            <Form layout="vertical" size="small">
                <Row gutter={[16, 0]}>
                    {group.fields.map((field) => (
                        <Col xs={24} sm={12} md={8} key={field.key}>
                            <Form.Item
                                label={
                                    <SettingFieldLabel
                                        settingKey={field.key}
                                        label={t(`settingsFields.${field.key}.label`)}
                                        overridden={isOverridden?.(field.key) ?? false}
                                        canReset={hasDefault?.(field.key) ?? false}
                                        onReset={onReset}
                                    />
                                }
                                tooltip={t(`settingsFields.${field.key}.tooltip`)}
                            >
                                {renderInput(field)}
                            </Form.Item>
                        </Col>
                    ))}
                </Row>
            </Form>
        </Card>
    );
};
