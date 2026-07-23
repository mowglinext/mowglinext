import React from "react";
import { Alert, Button, Card, Progress, Space, Typography } from "antd";
import { AimOutlined } from "@ant-design/icons";
import { useThemeMode } from "../../theme/ThemeContext.tsx";
import {
    PHASE_LABELS,
    RETRY_LABELS,
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
    const { colors } = useThemeMode();
    const { status, start, starting, running, done } = useDockCalibration();

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
                        Dock calibration (one-click)
                    </Text>
                    <Paragraph type="secondary" style={{ margin: "4px 0 0" }}>
                        Robot on the dock (charging) with RTK-Fixed. It reverses in a straight
                        line, checks the GPS course heading, then re-docks and saves the dock
                        pose. The blade stays off the whole time.
                    </Paragraph>
                </div>

                <Button
                    type="primary"
                    icon={<AimOutlined />}
                    loading={starting || running}
                    disabled={running}
                    onClick={start}
                >
                    {running ? "Calibrating…" : "Start dock calibration"}
                </Button>

                {(running || (status && status.phase !== 6)) && !showResult && (
                    <div>
                        <Text style={{ color: colors.text }}>{PHASE_LABELS[phase] ?? "…"}</Text>
                        <Progress percent={progressPct} status={running ? "active" : "normal"} />
                        {status && (
                            <Text type="secondary" style={{ fontSize: 12 }}>
                                COG σ {status.cog_std_deg.toFixed(2)}° · moved{" "}
                                {status.displacement_m.toFixed(2)} m ·{" "}
                                {status.charging ? "charging" : "not charging"}
                            </Text>
                        )}
                    </div>
                )}

                {showResult && success && (
                    <Alert
                        type="success"
                        showIcon
                        message="Dock calibrated"
                        description={status?.message}
                    />
                )}
                {showResult && !success && (
                    <Alert
                        type="error"
                        showIcon
                        message="Dock calibration failed"
                        description={
                            <>
                                <div>{status?.message}</div>
                                {RETRY_LABELS[retry] && <div>{RETRY_LABELS[retry]}</div>}
                            </>
                        }
                    />
                )}
            </Space>
        </Card>
    );
};
