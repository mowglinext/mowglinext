import React, { useMemo } from "react";
import { Card, Col, Form, InputNumber, Row, Space, Switch, Typography } from "antd";
import { ScissorOutlined } from "@ant-design/icons";
import { useThemeMode } from "../../theme/ThemeContext.tsx";

const { Text, Paragraph } = Typography;

type Props = {
    values: Record<string, any>;
    onChange: (key: string, value: any) => void;
};

/** Mini SVG preview showing strip pattern based on path_spacing and tool_width */
const StripPreview: React.FC<{ pathSpacing: number; toolWidth: number; headlandWidth: number }> = ({
    pathSpacing,
    toolWidth,
    headlandWidth,
}) => {
    const { colors, mode } = useThemeMode();
    const w = 200;
    const h = 140;
    const margin = 10;

    const strips = useMemo(() => {
        const lines: React.ReactNode[] = [];
        if (pathSpacing <= 0 || toolWidth <= 0) return lines;

        // Scale: 1m = 120px
        const scale = 120;
        const spacingPx = pathSpacing * scale;
        const widthPx = toolWidth * scale;
        const headlandPx = headlandWidth * scale;
        const areaW = w - 2 * margin;
        const areaH = h - 2 * margin;

        // Draw area boundary
        lines.push(
            <rect
                key="area"
                x={margin} y={margin}
                width={areaW} height={areaH}
                fill="none"
                stroke={mode === "dark" ? "#555" : "#ccc"}
                strokeWidth={1}
                strokeDasharray="3 2"
            />
        );

        // Draw headland
        if (headlandPx > 0) {
            lines.push(
                <rect
                    key="headland"
                    x={margin + headlandPx} y={margin + headlandPx}
                    width={areaW - 2 * headlandPx} height={areaH - 2 * headlandPx}
                    fill="none"
                    stroke={mode === "dark" ? "#4a6" : "#8c8"}
                    strokeWidth={0.5}
                    strokeDasharray="2 2"
                />
            );
        }

        // Draw strips
        const startX = margin + headlandPx + widthPx / 2;
        const endX = margin + areaW - headlandPx;
        let x = startX;
        let i = 0;
        while (x < endX && i < 20) {
            const stripColor = mode === "dark" ? "rgba(44, 199, 107, 0.3)" : "rgba(27, 157, 82, 0.2)";
            const lineColor = mode === "dark" ? "#2CC76B" : "#1B9D52";

            // Strip width (blade coverage)
            lines.push(
                <rect
                    key={`strip-${i}`}
                    x={x - widthPx / 2}
                    y={margin + headlandPx}
                    width={widthPx}
                    height={areaH - 2 * headlandPx}
                    fill={stripColor}
                    rx={1}
                />
            );

            // Centre line (actual path)
            lines.push(
                <line
                    key={`line-${i}`}
                    x1={x} y1={margin + headlandPx + 2}
                    x2={x} y2={margin + areaH - headlandPx - 2}
                    stroke={lineColor}
                    strokeWidth={1}
                />
            );

            x += spacingPx;
            i++;
        }

        // Overlap indicator
        if (spacingPx < widthPx && spacingPx > 0) {
            lines.push(
                <text key="overlap-label" x={w / 2} y={h - 3} textAnchor="middle" fontSize={8}
                    fill={mode === "dark" ? "#aaa" : "#666"} fontFamily="monospace">
                    overlap: {((toolWidth - pathSpacing) * 100).toFixed(0)}%
                </text>
            );
        }

        return lines;
    }, [pathSpacing, toolWidth, headlandWidth, mode]);

    return (
        <div style={{
            background: mode === "dark" ? "#1a1a1a" : "#fafafa",
            border: `1px solid ${colors.border}`,
            borderRadius: 8,
            padding: 4,
            display: "flex",
            justifyContent: "center",
        }}>
            <svg width={w} height={h} viewBox={`0 0 ${w} ${h}`}>
                {strips}
            </svg>
        </div>
    );
};

export const MowingSection: React.FC<Props> = ({ values, onChange }) => {
    // F2C swath spacing == tool_width (Robot::setCovWidth). The
    // preview shows blade swaths spaced by tool_width — adjacent
    // strips tile exactly, no overlap or gap.
    const pathSpacing = values.tool_width ?? 0.18;
    const toolWidth = values.tool_width ?? 0.18;
    const headlandWidth = values.headland_width ?? 0.18;

    return (
        <div>
            {/* Blade toggle + speeds */}
            <Card size="small" style={{ marginBottom: 16 }}>
                <Space direction="vertical" size={12} style={{ width: "100%" }}>
                    <div style={{ display: "flex", justifyContent: "space-between", alignItems: "center" }}>
                        <div>
                            <Text strong style={{ fontSize: 14 }}>
                                <ScissorOutlined style={{ marginRight: 6 }} />
                                Mowing Motor
                            </Text>
                            <Paragraph type="secondary" style={{ margin: "4px 0 0" }}>
                                Disable for testing without the blade spinning.
                            </Paragraph>
                        </div>
                        <Switch
                            checked={values.mowing_enabled ?? true}
                            onChange={(v) => onChange("mowing_enabled", v)}
                        />
                    </div>
                    <Form layout="vertical" size="small">
                        <Row gutter={[16, 0]}>
                            <Col xs={12} sm={8}>
                                <Form.Item label="Mowing Speed" tooltip="Speed during mowing strips (m/s)">
                                    <InputNumber
                                        value={values.mowing_speed}
                                        onChange={(v) => onChange("mowing_speed", v)}
                                        min={0.05} max={0.6} step={0.05} precision={2}
                                        style={{ width: "100%" }} addonAfter="m/s"
                                    />
                                </Form.Item>
                            </Col>
                            <Col xs={12} sm={8}>
                                <Form.Item label="Transit Speed" tooltip="Speed when going to/from dock or between zones">
                                    <InputNumber
                                        value={values.transit_speed}
                                        onChange={(v) => onChange("transit_speed", v)}
                                        min={0.05} max={0.6} step={0.05} precision={2}
                                        style={{ width: "100%" }} addonAfter="m/s"
                                    />
                                </Form.Item>
                            </Col>
                        </Row>
                    </Form>
                </Space>
            </Card>

            {/* Path pattern with visual preview */}
            <Card size="small" title="Mowing Pattern" style={{ marginBottom: 16 }}>
                <Row gutter={[16, 16]}>
                    <Col xs={24} lg={14}>
                        <Form layout="vertical" size="small">
                            <Row gutter={[16, 0]}>
                                <Col xs={12}>
                                    <Form.Item label="Headland Width" tooltip="Width of perimeter strip mowed before the inner field (F2C ConstHL inset). Wired to coverage_server.default_headland_width.">
                                        <InputNumber
                                            value={values.headland_width}
                                            onChange={(v) => onChange("headland_width", v)}
                                            min={0} max={1.0} step={0.05} precision={2}
                                            style={{ width: "100%" }} addonAfter="m"
                                        />
                                    </Form.Item>
                                </Col>
                                <Col xs={12}>
                                    <Form.Item label="Min Turning Radius" tooltip="F2C Robot::setMinTurningRadius — bounds the Dubins arc radius between swaths. Diff-drive mowers: keep small (~0.05 m). Wired to coverage_server.min_turning_radius.">
                                        <InputNumber
                                            value={values.min_turning_radius}
                                            onChange={(v) => onChange("min_turning_radius", v)}
                                            min={0.05} max={1.0} step={0.05} precision={2}
                                            style={{ width: "100%" }} addonAfter="m"
                                        />
                                    </Form.Item>
                                </Col>
                                <Col xs={12}>
                                    <Form.Item label="Mow Angle" tooltip="Mowing angle offset in degrees (-1 = auto)">
                                        <InputNumber
                                            value={values.mow_angle_offset_deg}
                                            onChange={(v) => onChange("mow_angle_offset_deg", v)}
                                            min={-1} max={180} step={5} precision={1}
                                            style={{ width: "100%" }} addonAfter="deg"
                                        />
                                    </Form.Item>
                                </Col>
                                <Col xs={12}>
                                    <Form.Item label="Angle Increment" tooltip="Rotate mowing angle each full pass">
                                        <InputNumber
                                            value={values.mow_angle_increment_deg}
                                            onChange={(v) => onChange("mow_angle_increment_deg", v)}
                                            min={0} max={90} step={5} precision={1}
                                            style={{ width: "100%" }} addonAfter="deg"
                                        />
                                    </Form.Item>
                                </Col>
                            </Row>
                        </Form>
                    </Col>
                    <Col xs={24} lg={10}>
                        <Text type="secondary" style={{ fontSize: 11, display: "block", marginBottom: 6 }}>
                            Strip preview (spacing vs. blade coverage)
                        </Text>
                        <StripPreview
                            pathSpacing={pathSpacing}
                            toolWidth={toolWidth}
                            headlandWidth={headlandWidth}
                        />
                    </Col>
                </Row>
            </Card>

            {/* Outline passes */}
            <Card size="small" title="Perimeter (Outline)" style={{ marginBottom: 16 }}>
                <Form layout="vertical" size="small">
                    <Row gutter={[16, 0]}>
                        <Col xs={12} sm={8}>
                            <Form.Item label="Outline Passes" tooltip="Number of perimeter passes before fill">
                                <InputNumber
                                    value={values.outline_passes}
                                    onChange={(v) => onChange("outline_passes", v)}
                                    min={0} max={5} step={1} precision={0}
                                    style={{ width: "100%" }}
                                />
                            </Form.Item>
                        </Col>
                        <Col xs={12} sm={8}>
                            <Form.Item label="Outline Offset" tooltip="Inward offset from boundary">
                                <InputNumber
                                    value={values.outline_offset}
                                    onChange={(v) => onChange("outline_offset", v)}
                                    min={0} max={0.5} step={0.01} precision={3}
                                    style={{ width: "100%" }} addonAfter="m"
                                />
                            </Form.Item>
                        </Col>
                        <Col xs={12} sm={8}>
                            <Form.Item label="Outline Overlap" tooltip="Overlap between outline and fill">
                                <InputNumber
                                    value={values.outline_overlap}
                                    onChange={(v) => onChange("outline_overlap", v)}
                                    min={0} max={0.2} step={0.01} precision={3}
                                    style={{ width: "100%" }} addonAfter="m"
                                />
                            </Form.Item>
                        </Col>
                    </Row>
                </Form>
            </Card>
        </div>
    );
};
