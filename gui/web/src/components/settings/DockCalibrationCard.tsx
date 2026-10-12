import React from "react";
import {useTranslation} from "react-i18next";
import { Alert, Button, Card, Progress, Space, Typography } from "antd";
import { AimOutlined } from "@ant-design/icons";
import { useThemeMode } from "../../theme/ThemeContext.tsx";
import {
    useDockCalibration,
} from "../../hooks/useDockCalibration.ts";

const { Text, Paragraph } = Typography;

/**
 * DockCalibrationCard — the ONE unified one-click dock calibration surface.
 * Replaces the old separate IMU-yaw modal + onboarding ImuYawStep. Robot must
 * be on the dock (charging) with RTK-Fixed. The node drives: reverse off the
 * dock (monitoring COG), coherence-gate the heading, re-dock until charging,
 * then persist the dock pose. Blade stays OFF throughout.
 */
export const DockCalibrationCard: React.FC = () => {
    const {t, i18n} = useTranslation();
    const number = (n: number) => n.toLocaleString(i18n.language, {maximumFractionDigits: 2});
    const { colors } = useThemeMode();
    const { status, start, starting, startError, running, done } = useDockCalibration();

    const phase = status?.phase ?? 6;
    const progressPct = Math.round((status?.progress ?? 0) * 100);
    const showResult = done && !!status;
    const success = status?.success ?? false;
    const retry = status?.retry_reason ?? 0;

    return (
        <Card size="small" style={{ marginBottom: 16 }}>
            <Space direction="vertical" size={12} style={{ width: "100%" }}>
                <div>
                    <Text strong className="mn-display" style={{ fontSize: 14, color: colors.text }}>
                        <AimOutlined style={{ marginRight: 6, color: colors.primary }} />
                        {t('dockCalibration.title')}
                    </Text>
                    <Paragraph type="secondary" style={{ margin: "4px 0 12px" }}>
                        {t('dockCalibration.intro')}
                    </Paragraph>
                    <Alert type="warning" showIcon message={t('dockCalibration.motionWarning')} />
                    <details style={{marginTop: 12}}>
                        <summary>{t('dockCalibration.details')}</summary>
                        <Paragraph type="secondary">{t('dockCalibration.detail')}</Paragraph>
                    </details>
                </div>

                <Button
                    type="primary"
                    icon={<AimOutlined />}
                    loading={starting || running}
                    disabled={running}
                    onClick={start}
                >
                    {t(running ? 'dockCalibration.running' : 'dockCalibration.start')}
                </Button>

                {startError && !running && (
                    <Alert
                        type="error"
                        showIcon
                        message={t('dockCalibration.startError')}
                        description={startError}
                    />
                )}

                {(running || (status && status.phase !== 6)) && !showResult && (
                    <div>
                        <Text style={{ color: colors.text }}>{t(`dockCalibration.phases.${phase}`)}</Text>
                        <Progress percent={progressPct} status={running ? "active" : "normal"} />
                        {status?.message && (
                            <div>
                                <Text type="secondary" style={{ fontSize: 12 }}>
                                    {status.message}
                                </Text>
                            </div>
                        )}
                        {status && (
                            <Text type="secondary" style={{ fontSize: 12 }}>
                                {t('dockCalibration.measurements', {spread: number(status.cog_std_deg), distance: number(status.displacement_m), charge: t(status.charging ? 'dockCalibration.charging' : 'dockCalibration.notCharging')})}
                            </Text>
                        )}
                    </div>
                )}

                {showResult && success && retry === 0 && (
                    <Alert
                        type="success"
                        showIcon
                        message={t('dockCalibration.success')}
                        description={status?.message}
                    />
                )}
                {showResult && success && retry !== 0 && (
                    <Alert
                        type="warning"
                        showIcon
                        message={t('dockCalibration.notDocked')}
                        description={status?.message}
                    />
                )}
                {showResult && !success && (
                    <Alert
                        type="error"
                        showIcon
                        message={t('dockCalibration.failed')}
                        description={
                            <>
                                <div>{status?.message}</div>
                                {retry > 0 && retry <= 7 && <div>{t(`dockCalibration.retries.${retry}`)}</div>}
                            </>
                        }
                    />
                )}
            </Space>
        </Card>
    );
};
