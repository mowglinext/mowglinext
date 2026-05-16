import {useState} from "react";
import {Alert, Modal, Statistic, Table, Tag, Typography} from "antd";
import type {ImportOpenMowerSummary} from "../hooks/useMapFiles.ts";

interface ImportOpenMowerModalProps {
    preview: ImportOpenMowerSummary | null;
    /**
     * Apply confirmation. The hook re-POSTs the stashed file body with
     * `apply: true`; this modal owns the loading + error UI but not the
     * fetch itself. Returning a rejected promise here surfaces the error
     * inline (we don't auto-close — the user can read the message and
     * cancel).
     */
    onApply: () => Promise<void>;
    onClose: () => void;
}

/**
 * Confirmation modal shown after an OpenMower map.json has been parsed
 * server-side. Renders a summary of what *would* be written if the
 * user confirms, then runs the live write path on Apply via the
 * `onApply` callback wired from MapPage / useMapFiles.
 */
export function ImportOpenMowerModal({preview, onApply, onClose}: ImportOpenMowerModalProps) {
    const open = preview !== null;
    const summary = preview;
    const [applying, setApplying] = useState(false);
    const [applyError, setApplyError] = useState<string | null>(null);

    const handleOk = async () => {
        setApplying(true);
        setApplyError(null);
        try {
            await onApply();
            // Reset local state before the parent clears `preview`, so
            // the next import session starts clean.
            setApplying(false);
            onClose();
        } catch (e: any) {
            setApplyError(e?.message ?? String(e));
            setApplying(false);
        }
    };

    const noMowAreas = (summary?.mowing_areas ?? 0) === 0;

    return (
        <Modal
            title="Import OpenMower map"
            open={open}
            onCancel={() => {
                if (applying) return;
                setApplyError(null);
                onClose();
            }}
            onOk={handleOk}
            okText={applying ? "Applying…" : "Apply (replaces current map)"}
            okButtonProps={{
                disabled: applying || noMowAreas,
                loading: applying,
                danger: true,
            }}
            cancelButtonProps={{disabled: applying}}
            cancelText="Cancel"
            width={720}
            maskClosable={!applying}
            closable={!applying}
        >
            {!summary ? null : (
                <div style={{display: "flex", flexDirection: "column", gap: 16}}>
                    <Alert
                        type="warning"
                        showIcon
                        message="Apply replaces the current MowgliNext map"
                        description="Clicking Apply will clear every existing area and dock pose, then write the areas shown below. Use Backup Map from the More menu first if you want a rollback file."
                    />

                    <div style={{display: "flex", gap: 32, flexWrap: "wrap"}}>
                        <Statistic title="Mowing areas" value={summary.mowing_areas} />
                        <Statistic title="Navigation areas" value={summary.navigation_areas} />
                        <Statistic title="Obstacles (re-parented)" value={summary.obstacles} />
                        {summary.orphan_obstacles > 0 ? (
                            <Statistic title="Orphan obstacles (dropped)" value={summary.orphan_obstacles} valueStyle={{color: "#cf1322"}} />
                        ) : null}
                    </div>

                    {summary.dock_pose ? (
                        <div>
                            <Typography.Text strong>Dock pose</Typography.Text>
                            <div style={{display: "flex", gap: 24, marginTop: 4}}>
                                <Statistic title="X (m)" value={summary.dock_pose.x} precision={3} />
                                <Statistic title="Y (m)" value={summary.dock_pose.y} precision={3} />
                                <Statistic title="Yaw (rad)" value={summary.dock_pose.yaw_rad} precision={4} />
                            </div>
                        </div>
                    ) : (
                        <Alert type="warning" message="No active docking station in source map — dock pose will not be touched." />
                    )}

                    {(summary.datum_shift_east_m !== 0 || summary.datum_shift_north_m !== 0) ? (
                        <Alert
                            type="info"
                            message={`Datum shift: (east=${summary.datum_shift_east_m.toFixed(2)} m, north=${summary.datum_shift_north_m.toFixed(2)} m)`}
                            description="OpenMower coordinates were translated to land in the MowgliNext map frame. If this looks wrong, double-check both datums."
                        />
                    ) : null}

                    {noMowAreas ? (
                        <Alert
                            type="error"
                            showIcon
                            message="No mowing areas in this file"
                            description="MowgliNext requires at least one mow area. Apply is disabled — close this modal and pick a different map.json."
                        />
                    ) : null}

                    <Table
                        size="small"
                        rowKey={(r) => `${r.name}|${r.type}`}
                        pagination={false}
                        dataSource={summary.areas}
                        columns={[
                            {title: "Name", dataIndex: "name", key: "name", render: (v: string) => v || <em>(unnamed)</em>},
                            {
                                title: "Type",
                                dataIndex: "type",
                                key: "type",
                                render: (v: string) => <Tag color={v === "mow" ? "green" : "blue"}>{v}</Tag>,
                            },
                            {title: "Vertices", dataIndex: "vertices", key: "vertices"},
                            {title: "Obstacles", dataIndex: "obstacles", key: "obstacles"},
                            {
                                title: "Approx area (m²)",
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
                            message={`${summary.warnings.length} warning${summary.warnings.length === 1 ? "" : "s"}`}
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
                            message="Apply failed"
                            description={applyError}
                        />
                    ) : null}
                </div>
            )}
        </Modal>
    );
}
