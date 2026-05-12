import {StopOutlined, CheckCircleOutlined} from "@ant-design/icons";
import {App, Button} from "antd";
import {useThemeMode} from "../../../theme/ThemeContext.tsx";
import type {TrackedObstacle} from "../../../types/ros.ts";
import {useApi} from "../../../hooks/useApi.ts";

interface TrackedObstaclesPanelProps {
    obstacles: TrackedObstacle[];
    /// Pre-resolved area_index for each obstacle id, computed by the parent
    /// (centroid-in-polygon test against the user's mowing areas). null for
    /// obstacles that don't fall inside any area — promote is disabled
    /// because we'd have nowhere to attach it.
    obstacleAreaIndex: Record<number, number | null>;
    /// Human-readable area name per area_index, for the confirm dialog.
    areaNames: Record<number, string>;
}

/// Lists tracked obstacles published by /obstacle_tracker/obstacles. Each row
/// has a "Promote" action that calls /map_server_node/promote_obstacle so the
/// transient observation becomes a permanent keepout for the containing area.
/// After the obstacle-tracker decouple (#6), this is the only path that
/// adds entries to obstacle_polygons_ at runtime — auto-promotion is gone.
export const TrackedObstaclesPanel = ({obstacles, obstacleAreaIndex, areaNames}: TrackedObstaclesPanelProps) => {
    const {colors} = useThemeMode();
    const {modal, notification} = App.useApp();
    const api = useApi();

    if (obstacles.length === 0) return null;

    const handlePromote = (obs: TrackedObstacle) => {
        const id = obs.id ?? 0;
        const areaIdx = obstacleAreaIndex[id];
        if (areaIdx == null) {
            notification.warning({
                message: "Obstacle is outside every mowing area",
                description: "Draw a mowing area that covers this obstacle before promoting it.",
            });
            return;
        }
        const areaLabel = areaNames[areaIdx] ?? `area ${areaIdx}`;

        modal.confirm({
            title: "Promote obstacle to permanent keepout?",
            content: (
                <div>
                    <p>
                        This adds the polygon as a permanent keepout in <strong>{areaLabel}</strong>.
                        The robot will plan around it from now on (until you remove it).
                    </p>
                    <p style={{fontSize: 12, color: colors.muted}}>
                        Tracker id: {id}
                        {obs.polygon?.points ? ` · ${obs.polygon.points.length} points` : ''}
                    </p>
                </div>
            ),
            okText: "Promote",
            cancelText: "Cancel",
            onOk: async () => {
                try {
                    const res = await api.mowglinext.callCreate("promote_obstacle", {
                        area_index: areaIdx,
                        obstacle_id: id,
                        polygon: obs.polygon ?? {points: []},
                    });
                    if ((res as any).error) throw new Error((res as any).error.error);
                    notification.success({message: `Obstacle promoted to ${areaLabel}`});
                } catch (e: any) {
                    notification.error({
                        message: "Failed to promote obstacle",
                        description: e.message,
                    });
                }
            },
        });
    };

    return (
        <div style={{display: 'flex', flexDirection: 'column', minWidth: 0}}>
            <div style={{
                padding: '8px 12px',
                fontSize: 12,
                fontWeight: 600,
                color: colors.muted,
                textTransform: 'uppercase',
                letterSpacing: '0.05em',
                borderBottom: `1px solid ${colors.borderSubtle}`,
            }}>
                Tracked obstacles ({obstacles.length})
            </div>
            <div style={{overflowY: 'auto', flex: 1}}>
                {obstacles.map((obs) => {
                    const id = obs.id ?? 0;
                    const areaIdx = obstacleAreaIndex[id];
                    const areaLabel = areaIdx == null ? 'no area' : (areaNames[areaIdx] ?? `area ${areaIdx}`);
                    return (
                        <div key={id} style={{
                            display: 'flex',
                            alignItems: 'center',
                            gap: 10,
                            padding: '10px 12px',
                            borderLeft: `3px solid #bf0000`,
                        }}>
                            <span style={{color: '#bf0000', fontSize: 16, flexShrink: 0, width: 20}}>
                                <StopOutlined />
                            </span>
                            <div style={{flex: 1, minWidth: 0}}>
                                <div style={{fontWeight: 500, fontSize: 13, color: colors.text}}>
                                    Obstacle #{id}
                                </div>
                                <div style={{fontSize: 11, color: colors.muted, marginTop: 1}}>
                                    {areaLabel}
                                    {obs.polygon?.points ? ` · ${obs.polygon.points.length}pt` : ''}
                                </div>
                            </div>
                            <Button
                                size="small"
                                type="text"
                                icon={<CheckCircleOutlined />}
                                onClick={() => handlePromote(obs)}
                                disabled={areaIdx == null}
                                title={areaIdx == null ? "No containing area" : "Promote to permanent keepout"}
                            >
                                Promote
                            </Button>
                        </div>
                    );
                })}
            </div>
        </div>
    );
};
