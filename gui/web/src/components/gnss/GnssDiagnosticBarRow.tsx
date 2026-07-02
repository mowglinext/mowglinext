import React from "react";
import { Space, Typography } from "antd";

const { Text } = Typography;

type Props = {
    label: React.ReactNode;
    value: React.ReactNode;
    ratio?: number;
    status?: React.ReactNode;
    helperText?: React.ReactNode;
    barColor?: string;
    testId?: string;
};

function clampRowRatio(ratio: number | undefined): number | undefined {
    if (ratio === undefined || !Number.isFinite(ratio)) {
        return undefined;
    }
    return Math.min(1, Math.max(0, ratio));
}

export const GnssDiagnosticBarRow: React.FC<Props> = ({
    label,
    value,
    ratio,
    status,
    helperText,
    barColor = "#1677ff",
    testId,
}) => {
    const clampedRatio = clampRowRatio(ratio);
    const barPercent = clampedRatio !== undefined ? `${(clampedRatio * 100).toFixed(1)}%` : undefined;

    return (
        <div style={{ display: "grid", gap: 4 }}>
            <div style={{ alignItems: "baseline", display: "flex", gap: 12, justifyContent: "space-between" }}>
                <Space size={8} wrap>
                    <Text strong>{label}</Text>
                    {status}
                </Space>
                <Text style={{ fontVariantNumeric: "tabular-nums" }}>{value}</Text>
            </div>
            {barPercent && (
                <div
                    aria-hidden="true"
                    data-testid={testId}
                    style={{
                        background: "rgba(5, 5, 5, 0.06)",
                        borderRadius: 999,
                        height: 8,
                        overflow: "hidden",
                        width: "100%",
                    }}
                >
                    <div
                        data-testid={testId ? `${testId}-fill` : undefined}
                        style={{
                            background: barColor,
                            borderRadius: 999,
                            height: "100%",
                            transition: "width 160ms ease-out",
                            width: barPercent,
                        }}
                    />
                </div>
            )}
            {helperText && (
                <Text type="secondary" style={{ fontSize: 12 }}>
                    {helperText}
                </Text>
            )}
        </div>
    );
};
