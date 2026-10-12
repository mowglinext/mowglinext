import {useState} from 'react';
import {Dropdown, InputNumber} from 'antd';
import {useTranslation} from 'react-i18next';
import {BorderOutlined, RadiusSettingOutlined, PlusOutlined, RadarChartOutlined} from '@ant-design/icons';
import type {ShapeType} from '../hooks/useMapEditing';
import {useThemeMode} from '../../../theme/ThemeContext.tsx';

const POPULAR_EMOJI = ['⭐', '❤️', '🌙', '🔔', '💎', '🍀', '🦋', '🐾', '🌸', '⚡', '🔥', '🎯'];

interface ShapePickerDropdownProps {
    onDrawShape?: (shape: ShapeType, sizeMeters: number) => void;
    onDrawEmoji?: (emoji: string, sizeMeters: number) => void;
    /// Starts a LiDAR-ignore line (click-to-place polyline, no fixed size) —
    /// listed alongside the geometry shapes below so it's reachable from the
    /// same "+ Add" entry point on both desktop and mobile, instead of its
    /// own standalone button.
    onDrawLidarCorridor?: () => void;
    children?: React.ReactNode;
    placement?: 'topLeft' | 'topRight' | 'bottomLeft' | 'bottomRight' | 'top' | 'bottom';
}

const shapes: {key: ShapeType; labelKey: string; icon: React.ReactNode}[] = [
    {key: 'square', labelKey: 'mapShapePicker.square', icon: <BorderOutlined />},
    {key: 'circle', labelKey: 'mapShapePicker.circle', icon: <RadiusSettingOutlined />},
    {key: 'hexagon', labelKey: 'mapShapePicker.hexagon', icon: <span style={{fontSize: 14, lineHeight: 1}}>⬡</span>},
];

export const ShapePickerDropdown = ({
    onDrawShape,
    onDrawEmoji,
    onDrawLidarCorridor,
    children,
    placement = 'topLeft',
}: ShapePickerDropdownProps) => {
    const [size, setSize] = useState(5);
    // Controlled open state so every selection explicitly closes the popup.
    // dropdownRender content (unlike antd's `menu` prop) never auto-closes on
    // its own — field-reported 2026-09-28: picking "Draw ignore line" on
    // mobile appeared to do nothing. Most likely cause: the popup stayed
    // open over the map, and the operator's next tap (meant to place the
    // first line point) landed on the still-open popup instead of the map
    // canvas beneath it. Closing immediately on every choice removes that
    // trap for shapes/emoji too, not just the ignore line.
    const [open, setOpen] = useState(false);
    const {colors} = useThemeMode();
    const {t} = useTranslation();

    const menuItemStyle: React.CSSProperties = {
        display: 'flex',
        alignItems: 'center',
        gap: 8,
        width: '100%',
        padding: '6px 8px',
        background: 'transparent',
        border: 'none',
        borderRadius: 6,
        color: colors.text,
        fontSize: 13,
        cursor: 'pointer',
        textAlign: 'left',
        transition: 'background 0.15s',
    };

    const hoverOn = (e: React.MouseEvent<HTMLButtonElement>) => {
        e.currentTarget.style.background = colors.bgElevated;
    };
    const hoverOff = (e: React.MouseEvent<HTMLButtonElement>) => {
        e.currentTarget.style.background = 'transparent';
    };

    const dropdownContent = (
        <div
            style={{
                background: colors.bgCard,
                borderRadius: 10,
                border: `1px solid ${colors.border}`,
                padding: 8,
                minWidth: 200,
                // Bounded + scrollable: on a short phone viewport with
                // placement="top" this content (size control + 3 shapes +
                // ignore line + emoji grid) can be taller than the space
                // above the toolbar, which would otherwise push the bottom
                // rows (including "Draw ignore line") off-screen and
                // unreachable instead of just scrolling into view.
                maxHeight: '70vh',
                overflowY: 'auto',
                boxShadow: colors.glassShadow,
            }}
        >
            {/* Size control */}
            <div style={{padding: '4px 8px 8px', display: 'flex', alignItems: 'center', gap: 8}}>
                <span style={{color: colors.muted, fontSize: 12}}>{t('mapShapePicker.size')}</span>
                <InputNumber
                    size="small"
                    min={0.1}
                    max={500}
                    step={0.1}
                    value={size}
                    onChange={(v) => { if (v != null) setSize(v); }}
                    suffix="m"
                    style={{width: 90}}
                />
            </div>

            <div style={{height: 1, background: colors.borderSubtle, margin: '0 4px 4px'}} />

            {/* Geometry shapes */}
            {shapes.map((s) => (
                <button
                    key={s.key}
                    onClick={() => { setOpen(false); onDrawShape?.(s.key, size); }}
                    style={menuItemStyle}
                    onMouseOver={hoverOn}
                    onMouseOut={hoverOff}
                >
                    {s.icon}
                    {t(s.labelKey)}
                </button>
            ))}

            {/* LiDAR-ignore line — a click-to-place polyline, not a fixed-size
                stamp, so it ignores the size control above; still listed here
                (not a separate toolbar button) so it's one tap away on mobile too.
                Closing the popup BEFORE starting the draw matters more here than
                for a shape: the very next tap has to land on the map. */}
            {onDrawLidarCorridor && (
                <button
                    onClick={() => { setOpen(false); onDrawLidarCorridor(); }}
                    style={menuItemStyle}
                    onMouseOver={hoverOn}
                    onMouseOut={hoverOff}
                >
                    <RadarChartOutlined />
                    {t('mapLidarCorridors.draw')}
                </button>
            )}

            <div style={{height: 1, background: colors.borderSubtle, margin: '4px'}} />

            {/* Emoji section */}
            <div style={{padding: '4px 8px 2px', color: colors.muted, fontSize: 11, fontWeight: 600}}>
                {t('mapShapePicker.emoji')}
            </div>
            <div style={{
                display: 'grid',
                gridTemplateColumns: 'repeat(6, 1fr)',
                gap: 2,
                padding: '2px 4px 4px',
            }}>
                {POPULAR_EMOJI.map((emoji) => (
                    <button
                        key={emoji}
                        onClick={() => { setOpen(false); onDrawEmoji?.(emoji, size); }}
                        style={{
                            background: 'transparent',
                            border: 'none',
                            borderRadius: 6,
                            fontSize: 20,
                            padding: 4,
                            cursor: 'pointer',
                            lineHeight: 1,
                            transition: 'background 0.15s',
                        }}
                        onMouseOver={hoverOn}
                        onMouseOut={hoverOff}
                        title={emoji}
                    >
                        {emoji}
                    </button>
                ))}
            </div>
        </div>
    );

    return (
        <Dropdown
            trigger={['click']}
            placement={placement}
            dropdownRender={() => dropdownContent}
            open={open}
            onOpenChange={setOpen}
        >
            {children ? (
                <span style={{display: 'inline-flex'}}>{children}</span>
            ) : (
                <button
                    style={{
                        display: 'inline-flex',
                        alignItems: 'center',
                        gap: 4,
                        padding: '4px 10px',
                        background: 'transparent',
                        border: `1px solid ${colors.border}`,
                        borderRadius: 6,
                        color: colors.text,
                        fontSize: 14,
                        cursor: 'pointer',
                    }}
                >
                    <PlusOutlined />
                    {t('mapShapePicker.add')}
                </button>
            )}
        </Dropdown>
    );
};

export default ShapePickerDropdown;
