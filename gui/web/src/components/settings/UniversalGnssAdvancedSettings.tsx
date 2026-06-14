import React, { useState } from "react";
import { Alert, Card, Form, Input, InputNumber, Select, Typography } from "antd";
import {
    GNSS_ADVANCED_SETTINGS_BY_FAMILY,
    GNSS_CUSTOM_OPTION_VALUE,
    inferPresetTextSelection,
    normalizeGnssSignalGroup,
    normalizeGnssString,
} from "./gnssConfig.ts";

const { Paragraph } = Typography;

type Props = {
    receiverFamily: unknown;
    values: Record<string, any>;
    onChange: (key: string, value: any) => void;
};

export const UniversalGnssAdvancedSettings: React.FC<Props> = ({
    receiverFamily,
    values,
    onChange,
}) => {
    const family = normalizeGnssString(receiverFamily).toLowerCase();
    const definition = GNSS_ADVANCED_SETTINGS_BY_FAMILY[family];
    const [customSelections, setCustomSelections] = useState<Record<string, boolean>>({});

    if (!definition) {
        return null;
    }

    return (
        <Card size="small" title={definition.title} style={{ marginBottom: 16 }}>
            <Paragraph type="secondary" style={{ marginTop: 0 }}>
                {definition.description}
            </Paragraph>

            <Form layout="vertical" size="small">
                {definition.fields.map((field) => {
                    if (field.kind === "presetText") {
                        const rawValue = field.key === "gnss_signal_group"
                            ? normalizeGnssSignalGroup(values[field.key])
                            : normalizeGnssString(values[field.key]);
                        const inferredPreset = inferPresetTextSelection(field, rawValue);
                        const customSelected = customSelections[field.key] || inferredPreset === GNSS_CUSTOM_OPTION_VALUE;
                        const selectedPreset = customSelected ? GNSS_CUSTOM_OPTION_VALUE : inferredPreset;
                        const selectedOption = field.options.find((option) => option.value === selectedPreset);

                        return (
                            <div key={field.key}>
                                <Form.Item label={field.label} tooltip={field.tooltip}>
                                    <Select
                                        value={selectedPreset}
                                        onChange={(preset) => {
                                            if (preset === GNSS_CUSTOM_OPTION_VALUE) {
                                                setCustomSelections((prev) => ({ ...prev, [field.key]: true }));
                                                return;
                                            }
                                            setCustomSelections((prev) => ({ ...prev, [field.key]: false }));
                                            onChange(field.key, preset);
                                        }}
                                        options={[
                                            ...field.options.map((option) => ({
                                                value: option.value,
                                                label: option.label,
                                            })),
                                            {
                                                value: GNSS_CUSTOM_OPTION_VALUE,
                                                label: field.customOptionLabel,
                                            },
                                        ]}
                                    />
                                </Form.Item>

                                <Form.Item label={field.rawLabel ?? `${field.label} Raw Value`}>
                                    <Input
                                        value={rawValue}
                                        onChange={(event) => {
                                            const nextValue = field.key === "gnss_signal_group"
                                                ? normalizeGnssSignalGroup(event.target.value)
                                                : normalizeGnssString(event.target.value);
                                            onChange(field.key, nextValue);
                                        }}
                                        placeholder={field.customPlaceholder}
                                    />
                                </Form.Item>

                                {selectedOption?.description && (
                                    <Paragraph type="secondary" style={{ marginTop: -8, fontSize: 12 }}>
                                        {selectedOption.description}
                                    </Paragraph>
                                )}

                                {field.helpText && (
                                    <Alert
                                        type="info"
                                        showIcon
                                        style={{ marginBottom: 12 }}
                                        message={field.helpText}
                                    />
                                )}
                            </div>
                        );
                    }

                    if (field.kind === "text") {
                        return (
                            <div key={field.key}>
                                <Form.Item label={field.label} tooltip={field.tooltip}>
                                    <Input
                                        value={normalizeGnssString(values[field.key])}
                                        onChange={(event) => onChange(field.key, normalizeGnssString(event.target.value))}
                                        placeholder={field.placeholder}
                                    />
                                </Form.Item>
                                {field.helpText && (
                                    <Alert
                                        type="info"
                                        showIcon
                                        style={{ marginBottom: 12 }}
                                        message={field.helpText}
                                    />
                                )}
                            </div>
                        );
                    }

                    if (field.kind === "number") {
                        return (
                            <div key={field.key}>
                                <Form.Item label={field.label} tooltip={field.tooltip}>
                                    <InputNumber
                                        value={values[field.key]}
                                        onChange={(value) => onChange(field.key, value)}
                                        placeholder={field.placeholder}
                                        min={field.min}
                                        step={field.step}
                                        addonAfter={field.addonAfter}
                                        style={{ width: "100%" }}
                                    />
                                </Form.Item>
                                {field.helpText && (
                                    <Alert
                                        type="info"
                                        showIcon
                                        style={{ marginBottom: 12 }}
                                        message={field.helpText}
                                    />
                                )}
                            </div>
                        );
                    }

                    return (
                        <div key={field.key}>
                            <Form.Item label={field.label} tooltip={field.tooltip}>
                                <Select
                                    value={normalizeGnssString(values[field.key])}
                                    onChange={(value) => onChange(field.key, value)}
                                    options={field.options.map((option) => ({
                                        value: option.value,
                                        label: option.label,
                                    }))}
                                />
                            </Form.Item>
                            {field.helpText && (
                                <Alert
                                    type="info"
                                    showIcon
                                    style={{ marginBottom: 12 }}
                                    message={field.helpText}
                                />
                            )}
                        </div>
                    );
                })}
            </Form>
        </Card>
    );
};
