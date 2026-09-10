import React from "react";
import { Alert, Card, Typography } from "antd";
import { WarningOutlined } from "@ant-design/icons";
import { useTranslation } from "react-i18next";

const { Paragraph } = Typography;

type Props = {
    values: Record<string, any>;
    onChange: (key: string, value: any) => void;
};

/**
 * Safety section — deliberately read-only.
 *
 * Issue #195: this section used to render a "Motor Temperature Limits" card
 * (motor_temp_high_c / motor_temp_low_c, "Stop Above" / "Resume Below") that
 * promised a thermal blade cutoff implemented by NO layer of the stack. The
 * firmware only MEASURES blade temperature (adc.c) and REPORTS it
 * (cpp_main.cpp); no ROS2 node ever read either key. The card was removed.
 *
 * Nothing was put in its place on purpose. The remaining keys the section
 * claims (see SECTION_DEFINITIONS in useSettingsManager.ts) are
 * lift_recovery_mode and lift_blade_resume_delay_sec, and neither is exposed:
 *
 *  - lift_recovery_mode is safety-adjacent. In hardware_bridge_node.cpp it
 *    SUPPRESSES the ROS2 lift emergency while the firmware LIFT bit is set and
 *    repeatedly requests a firmware emergency-latch release. Turning it on
 *    from a settings page is not a cosmetic preference, and it was never
 *    GUI-editable before. Changing that is out of scope for #195.
 *  - lift_blade_resume_delay_sec only has an effect INSIDE that recovery path
 *    (blade_was_enabled_before_lift_ is set only in the lift_recovery_mode_
 *    branch), so on a default robot it is inert — exposing it alone would be
 *    the same "control that does nothing" bug #195 asked us to remove.
 *
 * Both keys stay listed in the safety section's `keys` array so they keep
 * falling outside AdvancedSection's free-form editor, exactly as before.
 */
export const SafetySection: React.FC<Props> = () => {
    const { t } = useTranslation();
    return (
        <div>
            <Alert
                type="warning"
                showIcon
                icon={<WarningOutlined />}
                message={t("settingsSafety.alertMessage")}
                description={t("settingsSafety.alertDescription")}
                style={{ marginBottom: 16 }}
            />

            <Card size="small" title={t("settingsSafety.firmwareOwnedTitle")} style={{ marginBottom: 16 }}>
                <Paragraph type="secondary" style={{ fontSize: 12, marginBottom: 8 }}>
                    {t("settingsSafety.firmwareOwnedDescription")}
                </Paragraph>
                <Paragraph type="secondary" style={{ fontSize: 12, marginBottom: 0 }}>
                    {t("settingsSafety.temperatureThresholdsNote")}
                </Paragraph>
            </Card>

            {/* max_obstacle_avoidance_distance moved to the Obstacles
                section (ObstaclesSection.tsx) alongside the other
                obstacle-avoidance knobs. */}
        </div>
    );
};
