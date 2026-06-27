import {useState} from "react";
import {Alert, InputNumber, Modal, Space, Statistic, Table, Tag, Typography} from "antd";
import {useTranslation} from "react-i18next";
import type {ImportOpenMowerSummary} from "../hooks/useMapFiles.ts";

interface ImportOpenMowerModalProps {
    preview: ImportOpenMowerSummary | null;
    /**
     * Apply confirmation. The hook re-POSTs the stashed file body with
     * `apply: true` (and the OM datum, when set); this modal owns the
     * loading + error UI but not the fetch itself. Returning a rejected
     * promise here surfaces the error inline (we don't auto-close — the
     * user can read the message and cancel).
     */
    onApply: (omDatumLat?: number, omDatumLon?: number) => Promise<void>;
    /**
     * Re-run the preview with the OpenMower datum. The modal calls this
     * after the user fills (or clears) both datum fields; the returned
     * summary replaces the current preview so the datum-shift alert, dock
     * pose, and warnings reflect the reprojected geometry.
     */
    onReproject: (omDatumLat?: number, omDatumLon?: number) => Promise<ImportOpenMowerSummary>;
    onClose: () => void;
}

/**
 * Confirmation modal shown after an OpenMower map.json has been parsed
 * server-side. Renders a summary of what *would* be written if the
 * user confirms, then runs the live write path on Apply via the
 * `onApply` callback wired from MapPage / useMapFiles.
 */
export function ImportOpenMowerModal({preview, onApply, onReproject, onClose}: ImportOpenMowerModalProps) {
    const {t} = useTranslation();
    const open = preview !== null;
    const summary = preview;
    const [applying, setApplying] = useState(false);
    const [applyError, setApplyError] = useState<string | null>(null);
    // OpenMower datum (OM_DATUM_LAT/LONG) the source map was anchored at.
    // Both must be filled for the server to reproject; either left empty
    // ⇒ identity copy (offset import) + a warning from the backend.
    const [omLat, setOmLat] = useState<number | null>(null);
    const [omLon, setOmLon] = useState<number | null>(null);
    const [reprojecting, setReprojecting] = useState(false);

    const datumPair = (lat: number | null, lon: number | null): [number?, number?] =>
        lat !== null && lon !== null ? [lat, lon] : [undefined, undefined];

    const handleOk = async () => {
        setApplying(true);
        setApplyError(null);
        try {
            const [lat, lon] = datumPair(omLat, omLon);
            await onApply(lat, lon);
            // Reset local state before the parent clears `preview`, so
            // the next import session starts clean.
            setApplying(false);
            setOmLat(null);
            setOmLon(null);
            onClose();
        } catch (e: any) {
            setApplyError(e?.message ?? String(e));
            setApplying(false);
        }
    };

    // Re-preview with the current datum fields. Wired to the inputs'
    // onBlur / onPressEnter so the operator can tweak the datum and see
    // the reprojected preview without an extra button.
    const handleReproject = async () => {
        setApplyError(null);
        setReprojecting(true);
        try {
            const [lat, lon] = datumPair(omLat, omLon);
            await onReproject(lat, lon);
        } catch (e: any) {
            setApplyError(e?.message ?? String(e));
        } finally {
            setReprojecting(false);
        }
    };

    const handleCancel = () => {
        if (applying) return;
        setApplyError(null);
        setOmLat(null);
        setOmLon(null);
        onClose();
    };

    const noMowAreas = (summary?.mowing_areas ?? 0) === 0;

    return (
        <Modal
            title={t('mapImportOpenMower.title')}
            open={open}
            onCancel={handleCancel}
            onOk={handleOk}
            okText={applying ? t('mapImportOpenMower.applying') : t('mapImportOpenMower.applyReplaces')}
            okButtonProps={{
                disabled: applying || reprojecting || noMowAreas,
                loading: applying,
                danger: true,
            }}
            cancelButtonProps={{disabled: applying}}
            cancelText={t('mapImportOpenMower.cancel')}
            width={720}
            maskClosable={!applying}
            closable={!applying}
        >
            {!summary ? null : (
                <div style={{display: "flex", flexDirection: "column", gap: 16}}>
                    <Alert
                        type="warning"
                        showIcon
                        message={t('mapImportOpenMower.replaceWarningMessage')}
                        description={t('mapImportOpenMower.replaceWarningDescription')}
                    />

                    <div>
                        <Typography.Text strong>{t('mapImportOpenMower.omDatumTitle')}</Typography.Text>
                        <Typography.Paragraph type="secondary" style={{marginTop: 4, marginBottom: 8}}>
                            {t('mapImportOpenMower.omDatumHelp')}
                        </Typography.Paragraph>
                        <Space wrap>
                            <InputNumber
                                addonBefore={t('mapImportOpenMower.omDatumLat')}
                                value={omLat}
                                onChange={(v) => setOmLat(typeof v === "number" ? v : null)}
                                onBlur={handleReproject}
                                onPressEnter={handleReproject}
                                disabled={applying || reprojecting}
                                step={0.000001}
                                style={{width: 280}}
                                placeholder="OM_DATUM_LAT"
                            />
                            <InputNumber
                                addonBefore={t('mapImportOpenMower.omDatumLon')}
                                value={omLon}
                                onChange={(v) => setOmLon(typeof v === "number" ? v : null)}
                                onBlur={handleReproject}
                                onPressEnter={handleReproject}
                                disabled={applying || reprojecting}
                                step={0.000001}
                                style={{width: 280}}
                                placeholder="OM_DATUM_LONG"
                            />
                        </Space>
                    </div>

                    <div style={{display: "flex", gap: 32, flexWrap: "wrap"}}>
                        <Statistic title={t('mapImportOpenMower.mowingAreas')} value={summary.mowing_areas} />
                        <Statistic title={t('mapImportOpenMower.navigationAreas')} value={summary.navigation_areas} />
                        <Statistic title={t('mapImportOpenMower.obstaclesReparented')} value={summary.obstacles} />
                        {summary.orphan_obstacles > 0 ? (
                            <Statistic title={t('mapImportOpenMower.orphanObstaclesDropped')} value={summary.orphan_obstacles} valueStyle={{color: "#cf1322"}} />
                        ) : null}
                    </div>

                    {summary.dock_pose ? (
                        <div>
                            <Typography.Text strong>{t('mapImportOpenMower.dockPose')}</Typography.Text>
                            <div style={{display: "flex", gap: 24, marginTop: 4}}>
                                <Statistic title={t('mapImportOpenMower.xMeters')} value={summary.dock_pose.x} precision={3} />
                                <Statistic title={t('mapImportOpenMower.yMeters')} value={summary.dock_pose.y} precision={3} />
                                <Statistic title={t('mapImportOpenMower.yawRad')} value={summary.dock_pose.yaw_rad} precision={4} />
                            </div>
                        </div>
                    ) : (
                        <Alert type="warning" message={t('mapImportOpenMower.noDockingStation')} />
                    )}

                    {(summary.datum_shift_east_m !== 0 || summary.datum_shift_north_m !== 0) ? (
                        <Alert
                            type="info"
                            message={t('mapImportOpenMower.datumShift', {east: summary.datum_shift_east_m.toFixed(2), north: summary.datum_shift_north_m.toFixed(2)})}
                            description={t('mapImportOpenMower.datumShiftDescription')}
                        />
                    ) : null}

                    {noMowAreas ? (
                        <Alert
                            type="error"
                            showIcon
                            message={t('mapImportOpenMower.noMowingAreasMessage')}
                            description={t('mapImportOpenMower.noMowingAreasDescription')}
                        />
                    ) : null}

                    <Table
                        size="small"
                        rowKey={(r) => `${r.name}|${r.type}`}
                        pagination={false}
                        dataSource={summary.areas}
                        columns={[
                            {title: t('mapImportOpenMower.colName'), dataIndex: "name", key: "name", render: (v: string) => v || <em>{t('mapImportOpenMower.unnamed')}</em>},
                            {
                                title: t('mapImportOpenMower.colType'),
                                dataIndex: "type",
                                key: "type",
                                render: (v: string) => <Tag color={v === "mow" ? "green" : "blue"}>{v}</Tag>,
                            },
                            {title: t('mapImportOpenMower.colVertices'), dataIndex: "vertices", key: "vertices"},
                            {title: t('mapImportOpenMower.colObstacles'), dataIndex: "obstacles", key: "obstacles"},
                            {
                                title: t('mapImportOpenMower.colApproxArea'),
                                dataIndex: "approx_area_sqm",
                                key: "approx_area_sqm",
                                render: (v: number) => v.toFixed(1),
                            },
                        ]}
                    />

                    {summary.warnings.length > 0 ? (
                        <Alert
                            type="warning"
                            showIcon
                            message={t('mapImportOpenMower.warningCount', {count: summary.warnings.length})}
                            description={
                                <ul style={{margin: 0, paddingLeft: 20}}>
                                    {summary.warnings.map((w, i) => (
                                        <li key={i}>{w}</li>
                                    ))}
                                </ul>
                            }
                        />
                    ) : null}

                    {applyError ? (
                        <Alert
                            type="error"
                            showIcon
                            message={t('mapImportOpenMower.applyFailed')}
                            description={applyError}
                        />
                    ) : null}
                </div>
            )}
        </Modal>
    );
}
