import {useCallback, useEffect, useRef, useState} from "react";
import {Alert, Button, Card, Col, Row, Select, Space, Tag, Typography} from "antd";
import {ReloadOutlined} from "@ant-design/icons";
import {useTranslation} from "react-i18next";
import {ContentType} from "../../api/Api.ts";
import {mowingAreaIndex} from "../../utils/mapAreaIndex.ts";
import {useApi} from "../../hooks/useApi.ts";
import {useMowingMap} from "../../hooks/useMowingMap.ts";

type Orientation = {
    success: boolean;
    enabled: boolean;
    base_angle_deg: number;
    current_active: boolean;
    current_perpendicular: boolean;
    next_perpendicular: boolean;
};

function AreaOrientation({index, name, angle}: {index: number; name: string; angle: number}) {
    const {t} = useTranslation();
    const api = useApi();
    const [state, setState] = useState<Orientation>();
    const [error, setError] = useState(false);
    const [busy, setBusy] = useState(false);
    const saving = useRef(false);
    const generation = useRef(0);
    const baseAngle = state?.base_angle_deg ?? angle;
    const label = (perpendicular: boolean) => baseAngle < 0
        ? t(perpendicular ? "crossHatch.autoCross" : "crossHatch.autoBase")
        : `${((baseAngle + (perpendicular ? 90 : 0)) % 180 + 180) % 180}°`;
    const request = useCallback(async (next?: boolean) => {
        if (saving.current) return;
        const id = ++generation.current;
        saving.current = next !== undefined;
        setBusy(true);
        try {
            const response = await api.request<Orientation, {error: string}>({
                path: "/mowglinext/call/coverage_orientation", method: "POST",
                type: ContentType.Json, format: "json",
                body: {area_index: index, set_next: next !== undefined, perpendicular: next ?? false},
            });
            if (response.error || !response.data?.success) throw new Error("unavailable");
            if (id === generation.current) { setState(response.data); setError(false); }
        } catch {
            if (id === generation.current) setError(true);
        } finally {
            if (id === generation.current) { setBusy(false); saving.current = false; }
        }
    }, [api, index]);
    useEffect(() => {
        const pending = generation;
        void request();
        const timer = window.setInterval(() => { void request(); }, 10000);
        return () => { window.clearInterval(timer); pending.current++; };
    }, [request]);
    return <Card size="small" data-testid={`cross-hatch-area-${index}`}>
        <Row gutter={[12, 12]} align="middle">
            <Col xs={24} sm={8}><Typography.Text strong>{name}</Typography.Text></Col>
            <Col xs={24} sm={7}>
                {state?.current_active
                    ? <Tag color="blue">{t("crossHatch.current", {angle: label(state.current_perpendicular)})}</Tag>
                    : <Typography.Text type="secondary">{t("crossHatch.noActive")}</Typography.Text>}
            </Col>
            <Col xs={24} sm={9}>
                <Space>
                    <Typography.Text>{t("crossHatch.next")}</Typography.Text>
                    <Select aria-label={t("crossHatch.nextFor", {area: name})}
                        style={{minWidth: 130}} disabled={!state || busy || error}
                        loading={busy} value={state ? (state.next_perpendicular ? 90 : 0) : undefined}
                        onChange={(value: number) => { void request(value === 90); }}
                        options={[{value: 0, label: label(false)}, {value: 90, label: label(true)}]}/>
                    <Button aria-label={t("crossHatch.refresh", {area: name})} icon={<ReloadOutlined/>}
                        disabled={busy} onClick={() => { void request(); }}/>
                </Space>
            </Col>
        </Row>
        {error && <Alert style={{marginTop: 8}} type="error" showIcon message={t("crossHatch.unavailable")}/>}
        {state && !state.enabled && <Typography.Text type="secondary">{t("crossHatch.restart")}</Typography.Text>}
    </Card>;
}

export function CrossHatchSettings({angle}: {angle: number}) {
    const {t} = useTranslation();
    const map = useMowingMap();
    const areas = (map.working_area ?? []).map((area, position) => ({area, index: mowingAreaIndex(map, position)}))
        .filter(({area}) => !area.is_navigation_area);
    return <Card title={t("crossHatch.title")} data-testid="cross-hatch-settings" style={{marginTop: 16}}>
        <Typography.Paragraph type="secondary">{t("crossHatch.help")}</Typography.Paragraph>
        <Space direction="vertical" style={{width: "100%"}}>
            {areas.length ? areas.map(({area, index}, position) => index === undefined
                ? <Alert key={position} type="warning" message={t("crossHatch.areaUnavailable")}/>
                : <AreaOrientation key={index} index={index}
                name={area.name || t("crossHatch.area", {index: index + 1})} angle={angle}/>)
                : <Typography.Text type="secondary">{t("crossHatch.noAreas")}</Typography.Text>}
        </Space>
    </Card>;
}
