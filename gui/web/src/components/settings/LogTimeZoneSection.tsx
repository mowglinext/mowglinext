import React from "react";
import {Card, Radio, Space, Typography} from "antd";
import {useTranslation} from "react-i18next";
import {useThemeMode} from "../../theme/ThemeContext.tsx";
import {useTimeFormat} from "../../hooks/useTimeFormat.tsx";
import type {TimeZoneMode} from "../../utils/logTime.ts";

const {Paragraph, Text} = Typography;

const TIME_ZONE_OPTIONS: TimeZoneMode[] = ['local', 'utc'];

/**
 * Chooses the zone every log line and `*_at` timestamp is rendered in. This is
 * a browser-local preference (localStorage), not a robot setting — which is why
 * it lives in the "appearance" section next to the display mode rather than in
 * `mowgli_robot.yaml`.
 */
export const LogTimeZoneSection: React.FC = () => {
    const {t} = useTranslation();
    const {colors} = useThemeMode();
    const {timeZoneMode, setTimeZoneMode, zoneLabel} = useTimeFormat();

    return (
        <Card size="small">
            <Space direction="vertical" size={14} style={{width: "100%"}}>
                <div>
                    <Text strong style={{fontSize: 14}}>{t("logTime.title")}</Text>
                    <Paragraph type="secondary" style={{margin: "4px 0 0"}}>
                        {t("logTime.description")}
                    </Paragraph>
                </div>

                <Radio.Group
                    value={timeZoneMode}
                    onChange={(event) => setTimeZoneMode(event.target.value as TimeZoneMode)}
                    aria-label={t("logTime.title")}
                    style={{display: "grid", gridTemplateColumns: "repeat(auto-fit, minmax(190px, 1fr))", gap: 10}}
                >
                    {TIME_ZONE_OPTIONS.map((mode) => {
                        const selected = timeZoneMode === mode;
                        return (
                            <Radio
                                key={mode}
                                value={mode}
                                style={{
                                    alignItems: "flex-start",
                                    minHeight: 76,
                                    marginInlineEnd: 0,
                                    padding: "12px 14px",
                                    borderRadius: 12,
                                    border: `1px solid ${selected ? colors.accent : colors.border}`,
                                    background: selected ? colors.primaryBg : colors.bgElevated,
                                }}
                            >
                                <span style={{display: "inline-flex", flexDirection: "column", gap: 4, paddingLeft: 4}}>
                                    <Text strong style={{color: colors.text}}>{t(`logTime.${mode}.label`)}</Text>
                                    <Text type="secondary" style={{fontSize: 12, lineHeight: 1.45}}>
                                        {t(`logTime.${mode}.description`)}
                                    </Text>
                                </span>
                            </Radio>
                        );
                    })}
                </Radio.Group>

                <Text type="secondary" style={{fontSize: 12}}>
                    {t("logTime.zoneBadge", {zone: zoneLabel})}
                </Text>
            </Space>
        </Card>
    );
};
