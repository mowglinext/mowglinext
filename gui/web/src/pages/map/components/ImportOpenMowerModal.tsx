import {Alert, Modal, Statistic, Table, Tag, Typography} from "antd";
import type {ImportOpenMowerSummary} from "../hooks/useMapFiles.ts";

interface ImportOpenMowerModalProps {
    preview: ImportOpenMowerSummary | null;
    onClose: () => void;
}

/**
 * Confirmation modal shown after an OpenMower map.json has been parsed
 * server-side. Renders a summary of what *would* be written if the
 * user confirmed — but the confirm button is intentionally disabled in
 * this iteration. See docs/IMPORT_OPENMOWER_MAP.md §5 for the open
 * questions gating the write path.
 *
 * When the apply path goes live this component should:
 *   1. enable the OK button
 *   2. on OK, POST again with `apply: true` and the same body the user
 *      already uploaded
 *   3. on success, refetch the map (parent owns this) and close
 */
export function ImportOpenMowerModal({preview, onClose}: ImportOpenMowerModalProps) {
    const open = preview !== null;
    const summary = preview;

    return (
        <Modal
            title="Import OpenMower map — preview"
            open={open}
            onCancel={onClose}
            // Apply is intentionally disabled until the design-doc open
            // questions are resolved (datum mismatch, replace-vs-merge,
            // backup-before-import). Showing the OK button greyed out
            // makes the "this is a preview, no writes" state obvious.
            okText="Apply (disabled — preview only)"
            okButtonProps={{disabled: true}}
            cancelText="Close"
            width={720}
        >
            {!summary ? null : (
                <div style={{display: "flex", flexDirection: "column", gap: 16}}>
                    <Alert
                        type="info"
                        showIcon
                        message="Preview only — nothing has been written yet"
                        description="The MowgliNext write path is not yet enabled for this importer. Review the parsed contents below; close the modal to discard. See docs/IMPORT_OPENMOWER_MAP.md for the open questions blocking the apply step."
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
                </div>
            )}
        </Modal>
    );
}
