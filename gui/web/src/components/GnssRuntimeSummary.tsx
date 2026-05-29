import React from "react";
import { Alert, Card, Col, Row, Space, Tag, Typography } from "antd";
import type { GnssCrossCheck } from "../hooks/useDiagnosticsSnapshot.ts";
import { GNSS_BACKEND_OPTIONS, hasRuntimeGnss, isRuntimeUnicore } from "../utils/gnssRuntime.ts";

const { Text } = Typography;

type Props = {
    gnss?: GnssCrossCheck | null;
    title?: string;
};

const backendLabel = (backend: string | undefined | null) =>
    GNSS_BACKEND_OPTIONS.find((option) => option.value === (backend ?? "").trim().toLowerCase())?.label ??
    (backend || "Unknown GNSS backend");

export const GnssRuntimeSummary: React.FC<Props> = ({ gnss, title = "Runtime GNSS" }) => {
    if (!hasRuntimeGnss(gnss)) {
        return null;
    }

    const runtimeManaged = isRuntimeUnicore(gnss);

    return (
        <Card size="small" title={title} style={{ marginBottom: 16 }}>
            <Space wrap style={{ marginBottom: 12 }}>
                <Text type="secondary" style={{ fontSize: 12 }}>Backend</Text>
                <Tag color="processing">{backendLabel(gnss.backend)}</Tag>
                {gnss.protocol && <Tag>{gnss.protocol}</Tag>}
                {gnss.connection && <Tag>{gnss.connection}</Tag>}
            </Space>

            <Row gutter={[12, 8]}>
                <Col xs={24} sm={12}>
                    <Text type="secondary" style={{ fontSize: 11 }}>Port</Text>
                    <br />
                    <Text code style={{ fontSize: 11 }}>{gnss.port || "—"}</Text>
                </Col>
                <Col xs={24} sm={12}>
                    <Text type="secondary" style={{ fontSize: 11 }}>Baud</Text>
                    <br />
                    <Text code style={{ fontSize: 11 }}>{gnss.baud || "—"}</Text>
                </Col>
                {gnss.by_id && (
                    <Col span={24}>
                        <Text type="secondary" style={{ fontSize: 11 }}>USB by-id</Text>
                        <br />
                        <Text code style={{ fontSize: 11 }}>{gnss.by_id}</Text>
                    </Col>
                )}
                {gnss.frame_id && (
                    <Col span={24}>
                        <Text type="secondary" style={{ fontSize: 11 }}>Frame ID</Text>
                        <br />
                        <Text code style={{ fontSize: 11 }}>{gnss.frame_id}</Text>
                    </Col>
                )}
            </Row>

            {runtimeManaged && (
                <Alert
                    type="info"
                    showIcon
                    style={{ marginTop: 12 }}
                    message="Unicore backend is selected by the installer/runtime"
                    description="This GUI keeps the GNSS contract backend-agnostic and does not rewrite docker/.env. Protocol and serial fields below are shown from the active runtime contract and stay read-only here."
                />
            )}
        </Card>
    );
};
