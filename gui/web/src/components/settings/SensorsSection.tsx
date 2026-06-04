import React from "react";
import { Card, Switch, Typography } from "antd";
import { RadarChartOutlined } from "@ant-design/icons";
import { RobotComponentEditor } from "../RobotComponentEditor.tsx";

const { Text, Paragraph } = Typography;

type Props = {
    values: Record<string, any>;
    onChange: (key: string, value: any) => void;
};

export const SensorsSection: React.FC<Props> = ({ values, onChange }) => {
    // fusion_graph is the sole localizer and always runs (the use_fusion_graph
    // launch flag was removed), so the LiDAR toggle drives the scan-factor
    // gates that ARE consumed by fusion_graph.launch.py: use_scan_matching and
    // use_loop_closure. With no LiDAR there are no scans to match, so both are
    // forced off. Operators can still fine-tune them in the Localization tab.
    const handleLidarToggle = (enabled: boolean) => {
        onChange("lidar_enabled", enabled);
        onChange("use_scan_matching", enabled);
        onChange("use_loop_closure", enabled);
    };

    return (
        <div>
            {/* LiDAR toggle */}
            <Card size="small" style={{ marginBottom: 16 }}>
                <div style={{ display: "flex", justifyContent: "space-between", alignItems: "center" }}>
                    <div>
                        <Text strong style={{ fontSize: 14 }}>
                            <RadarChartOutlined style={{ marginRight: 6 }} />
                            LiDAR Sensor
                        </Text>
                        <Paragraph type="secondary" style={{ margin: "4px 0 0" }}>
                            Enable if your robot has a LiDAR. Also flips
                            {" "}<Text code>use_scan_matching</Text> and{" "}
                            <Text code>use_loop_closure</Text> so the factor graph fuses LiDAR
                            scans. Fine-tune those in the Localization tab.
                        </Paragraph>
                    </div>
                    <Switch
                        checked={values.lidar_enabled ?? false}
                        onChange={handleLidarToggle}
                    />
                </div>
            </Card>

            {/* Sensor placement visual editor */}
            <RobotComponentEditor values={values} onChange={onChange} />
        </div>
    );
};
