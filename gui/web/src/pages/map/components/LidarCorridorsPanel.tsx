import {CheckOutlined, CloseOutlined, DeleteOutlined, EditOutlined} from "@ant-design/icons";
import {Alert, Button, InputNumber} from "antd";
import {useTranslation} from "react-i18next";
import {useThemeMode} from "../../../theme/ThemeContext.tsx";
import type {LidarIgnoreCorridor} from "../../../types/ros.ts";

/// Line colour on the map and the accent of each row (kept in sync with the
/// map layer in MapPage).
export const CORRIDOR_COLOR = '#eb2f96';

interface LidarCorridorsPanelProps {
    corridors: LidarIgnoreCorridor[];
    busy: boolean;
    /// False outside the map edit mode: the list is then read-only.
    editable: boolean;
    /// True while the operator is clicking points onto the map.
    drawing: boolean;
    drawPointCount: number;
    onStartDraw: () => void;
    onFinishDraw: () => void;
    onCancelDraw: () => void;
    /// Width in METRES (the panel edits centimetres).
    onChangeWidth: (index: number, widthM: number) => void;
    onDelete: (index: number) => void;
    /// Index of the line currently selected on the map, or null.
    selectedIndex: number | null;
    /// Round the selected line through its points / thin it out again.
    onSmooth: () => void;
    onSimplify: () => void;
}

/// Operator-drawn LiDAR-ignore lines. Inside a line's width the LiDAR returns
/// are dropped for BOTH the costmap (FTC/Nav2 avoidance) and collision_monitor,
/// so the robot follows the recorded boundary next to e.g. a hedge instead of
/// being pushed off it. Everywhere else the LiDAR keeps working normally.
export const LidarCorridorsPanel = ({
    corridors, busy, editable, drawing, drawPointCount, onStartDraw, onFinishDraw, onCancelDraw, onChangeWidth, onDelete,
    selectedIndex, onSmooth, onSimplify,
}: LidarCorridorsPanelProps) => {
    const {colors} = useThemeMode();
    const {t} = useTranslation();

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
                {t('mapLidarCorridors.header', {count: corridors.length})}
            </div>
            <div style={{padding: '6px 12px', fontSize: 11, color: colors.muted}}>
                {t('mapLidarCorridors.hint')}
            </div>
            {!editable && (
                <div style={{padding: '0 12px 6px', fontSize: 11, color: colors.muted}}>
                    {t('mapLidarCorridors.lockedHint')}
                </div>
            )}
            <div style={{padding: '0 12px 6px'}}>
                <Alert type="warning" showIcon style={{fontSize: 11, padding: '4px 8px'}}
                    message={t('mapLidarCorridors.safetyWarning')}/>
            </div>
            <div style={{overflowY: 'auto', flex: 1}}>
                {corridors.map((corridor, index) => (
                    <div key={corridor.id ?? index} style={{
                        display: 'flex',
                        alignItems: 'center',
                        gap: 6,
                        padding: '6px 12px',
                        borderLeft: `3px solid ${CORRIDOR_COLOR}`,
                        background: selectedIndex === index ? colors.bgElevated : 'transparent',
                    }}>
                        <div style={{flex: 1, fontSize: 12, color: colors.text, minWidth: 0}}>
                            {corridor.name || t('mapLidarCorridors.unnamed', {id: corridor.id ?? index + 1})}
                        </div>
                        <InputNumber
                            size="small"
                            style={{width: 84}}
                            min={5}
                            max={100}
                            step={5}
                            precision={0}
                            addonAfter="cm"
                            disabled={busy || !editable}
                            value={Math.round((corridor.width_m ?? 0.2) * 100)}
                            aria-label={t('mapLidarCorridors.widthLabel')}
                            title={t('mapLidarCorridors.widthTooltip')}
                            onChange={(value) => {
                                if (typeof value === 'number') onChangeWidth(index, value / 100);
                            }}
                        />
                        <Button size="small" type="text" danger disabled={busy || !editable}
                            icon={<DeleteOutlined aria-hidden="true"/>}
                            onClick={() => onDelete(index)}
                            title={t('mapLidarCorridors.delete')}/>
                    </div>
                ))}
            </div>
            <div style={{padding: '6px 12px 10px', display: 'flex', gap: 6, flexWrap: 'wrap'}}>
                {drawing ? (
                    <>
                        <div style={{width: '100%', fontSize: 11, color: colors.muted}}>
                            {t('mapLidarCorridors.drawing', {count: drawPointCount})}
                        </div>
                        <Button size="small" type="primary" icon={<CheckOutlined aria-hidden="true"/>}
                            disabled={drawPointCount < 2 || busy} onClick={onFinishDraw}>
                            {t('mapLidarCorridors.finish')}
                        </Button>
                        <Button size="small" icon={<CloseOutlined aria-hidden="true"/>} onClick={onCancelDraw}>
                            {t('mapLidarCorridors.cancel')}
                        </Button>
                    </>
                ) : (
                    <>
                        <Button size="small" icon={<EditOutlined aria-hidden="true"/>} disabled={busy || !editable} onClick={onStartDraw}>
                            {t('mapLidarCorridors.draw')}
                        </Button>
                        {selectedIndex !== null && editable && (
                            <>
                                <Button size="small" disabled={busy} onClick={onSmooth}
                                    title={t('mapLidarCorridors.smoothTooltip')}>
                                    {t('mapLidarCorridors.smooth')}
                                </Button>
                                <Button size="small" disabled={busy} onClick={onSimplify}
                                    title={t('mapLidarCorridors.simplifyTooltip')}>
                                    {t('mapLidarCorridors.simplify')}
                                </Button>
                            </>
                        )}
                        {editable && (
                            <div style={{width: '100%', fontSize: 11, color: colors.muted}}>
                                {t('mapLidarCorridors.editHint')}
                            </div>
                        )}
                    </>
                )}
            </div>
        </div>
    );
};
