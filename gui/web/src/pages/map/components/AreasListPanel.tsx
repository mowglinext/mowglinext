import {EnvironmentOutlined, CompassOutlined, StopOutlined, UpOutlined, DownOutlined} from "@ant-design/icons";
import {useTranslation} from "react-i18next";
import type {AreaListItem} from "../utils/types.ts";
import {useThemeMode} from "../../../theme/ThemeContext.tsx";

const TYPE_CONFIG: Record<string, { color: string; icon: React.ReactNode; labelKey: string }> = {
    workarea: {color: '#4caf50', icon: <EnvironmentOutlined/>, labelKey: 'mapAreasList.typeWorkArea'},
    navigation: {color: '#888888', icon: <CompassOutlined/>, labelKey: 'mapAreasList.typeNavigation'},
    obstacle: {color: '#bf0000', icon: <StopOutlined/>, labelKey: 'mapAreasList.typeObstacle'},
};

interface AreasListPanelProps {
    areas: AreaListItem[];
    onAreaClick?: (id: string) => void;
    onReorder?: (id: string, direction: 'up' | 'down') => void;
    selectedId?: string;
}

export const AreasListPanel = ({areas, onAreaClick, onReorder, selectedId}: AreasListPanelProps) => {
    const {colors} = useThemeMode();
    const {t} = useTranslation();
    const workAreas = areas.filter(a => a.ftype === 'workarea');
    if (areas.length === 0) return null;

    return (
        <div style={{display: 'flex', flexDirection: 'column', minWidth: 0}}>
            {/* Header */}
            <div style={{
                padding: '8px 12px',
                fontSize: 12,
                fontWeight: 600,
                color: colors.muted,
                textTransform: 'uppercase',
                letterSpacing: '0.05em',
                borderBottom: `1px solid ${colors.borderSubtle}`,
            }}>
                {t('mapAreasList.areasHeader', {count: areas.length})}
            </div>

            {/* Area items */}
            <div style={{overflowY: 'auto', flex: 1}}>
                {areas.map((item) => {
                    const cfg = TYPE_CONFIG[item.ftype] ?? TYPE_CONFIG.obstacle;
                    const isSelected = selectedId === item.id;
                    return (
                        <button
                            key={item.id}
                            onClick={() => onAreaClick?.(item.id)}
                            style={{
                                display: 'flex',
                                alignItems: 'center',
                                gap: 10,
                                width: '100%',
                                padding: '10px 12px',
                                background: isSelected ? colors.bgElevated : 'transparent',
                                border: 'none',
                                borderLeft: `3px solid ${cfg.color}`,
                                cursor: onAreaClick ? 'pointer' : 'default',
                                transition: 'background 0.15s',
                                textAlign: 'left',
                            }}
                            onMouseOver={(e) => {
                                if (!isSelected) e.currentTarget.style.background = colors.bgSubtle;
                            }}
                            onMouseOut={(e) => {
                                if (!isSelected) e.currentTarget.style.background = 'transparent';
                            }}
                        >
                            {/* Type icon */}
                            <span style={{
                                color: cfg.color,
                                fontSize: 16,
                                flexShrink: 0,
                                width: 20,
                                display: 'flex',
                                alignItems: 'center',
                                justifyContent: 'center',
                            }}>
                                {cfg.icon}
                            </span>

                            {/* Name + details */}
                            <div style={{flex: 1, minWidth: 0}}>
                                <div style={{
                                    fontWeight: 500,
                                    fontSize: 13,
                                    color: colors.text,
                                    overflow: 'hidden',
                                    textOverflow: 'ellipsis',
                                    whiteSpace: 'nowrap',
                                }}>
                                    {item.name}
                                </div>
                                <div style={{fontSize: 11, color: colors.muted, marginTop: 1}}>
                                    {item.areaLabel}
                                </div>
                            </div>

                            {/* Mowing order badge + reorder arrows */}
                            {item.mowingOrder != null && (
                                <span style={{display: 'flex', alignItems: 'center', gap: 4, flexShrink: 0}}>
                                    {onReorder && (
                                        <span style={{display: 'flex', flexDirection: 'column', gap: 0}}>
                                            <button
                                                onClick={(e) => { e.stopPropagation(); onReorder(item.id, 'up'); }}
                                                disabled={item.mowingOrder <= 1}
                                                style={{
                                                    background: 'transparent', border: 'none', padding: 0,
                                                    color: item.mowingOrder <= 1 ? colors.borderSubtle : colors.muted,
                                                    cursor: item.mowingOrder <= 1 ? 'not-allowed' : 'pointer',
                                                    fontSize: 10, lineHeight: 1,
                                                }}
                                                aria-label={t('mapAreasList.moveUp')}
                                            >
                                                <UpOutlined />
                                            </button>
                                            <button
                                                onClick={(e) => { e.stopPropagation(); onReorder(item.id, 'down'); }}
                                                disabled={item.mowingOrder >= workAreas.length}
                                                style={{
                                                    background: 'transparent', border: 'none', padding: 0,
                                                    color: item.mowingOrder >= workAreas.length ? colors.borderSubtle : colors.muted,
                                                    cursor: item.mowingOrder >= workAreas.length ? 'not-allowed' : 'pointer',
                                                    fontSize: 10, lineHeight: 1,
                                                }}
                                                aria-label={t('mapAreasList.moveDown')}
                                            >
                                                <DownOutlined />
                                            </button>
                                        </span>
                                    )}
                                    <span style={{
                                        backgroundColor: cfg.color,
                                        color: '#fff',
                                        borderRadius: '50%',
                                        width: 22,
                                        height: 22,
                                        display: 'flex',
                                        alignItems: 'center',
                                        justifyContent: 'center',
                                        fontSize: 11,
                                        fontWeight: 600,
                                    }}>
                                        {item.mowingOrder}
                                    </span>
                                </span>
                            )}
                        </button>
                    );
                })}
            </div>
        </div>
    );
};
