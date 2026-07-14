import {Col, Statistic, Typography} from "antd";
import type {ReactNode} from "react";
import {useThemeMode} from "../theme/ThemeContext.tsx";

/**
 * A single diagnostics telemetry cell: a titled statistic with an optional
 * secondary hint underneath. Consolidates the ~30 repeated
 * Col + Statistic + secondary-hint blocks across DiagnosticsPage into one
 * component with a consistent null placeholder ('—') and a `tone` → color map.
 */

export type TelemetryTone = "default" | "ok" | "warn" | "danger";

export const TELEMETRY_PLACEHOLDER = "—";

export interface TelemetryStatProps {
    title: ReactNode;
    /** null/undefined renders the shared placeholder. */
    value: number | string | null | undefined;
    /** Decimal places — only applied to numeric, non-placeholder values. */
    precision?: number;
    /** Suffix (e.g. "°", "cm") — omitted when the value is a placeholder. */
    suffix?: string;
    /** Secondary hint line below the statistic. */
    hint?: ReactNode;
    tone?: TelemetryTone;
    /** Larger value font used by the fusion-graph / ICP monitor cells. */
    large?: boolean;
    /** antd Col span props. */
    span?: number;
    xs?: number;
    md?: number;
    lg?: number;
}

export function TelemetryStat({
    title,
    value,
    precision,
    suffix,
    hint,
    tone = "default",
    large = false,
    span,
    xs,
    md,
    lg,
}: TelemetryStatProps) {
    const {colors} = useThemeMode();

    const isPlaceholder = value === null || value === undefined;
    const toneColor =
        tone === "danger" ? colors.danger :
        tone === "warn" ? colors.warning :
        tone === "ok" ? colors.success :
        undefined;

    const valueStyle = {
        ...(large ? {fontSize: 18} : {}),
        ...(toneColor ? {color: toneColor} : {}),
    };

    return (
        <Col span={span} xs={xs} md={md} lg={lg}>
            <Statistic
                title={title}
                value={isPlaceholder ? TELEMETRY_PLACEHOLDER : value}
                precision={isPlaceholder || typeof value !== "number" ? undefined : precision}
                suffix={isPlaceholder ? undefined : suffix}
                valueStyle={Object.keys(valueStyle).length > 0 ? valueStyle : undefined}
            />
            {hint != null && hint !== "" && (
                <Typography.Text type="secondary" style={{fontSize: 11}}>
                    {hint}
                </Typography.Text>
            )}
        </Col>
    );
}

export default TelemetryStat;
