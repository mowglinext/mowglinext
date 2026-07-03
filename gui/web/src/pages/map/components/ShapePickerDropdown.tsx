import {useState} from 'react';
import {Dropdown, InputNumber} from 'antd';
import {useTranslation} from 'react-i18next';
import {BorderOutlined, RadiusSettingOutlined, PlusOutlined} from '@ant-design/icons';
import type {ShapeType} from '../hooks/useMapEditing';
import {useThemeMode} from '../../../theme/ThemeContext.tsx';

const POPULAR_EMOJI = ['⭐', '❤️', '🌙', '🔔', '💎', '🍀', '🦋', '🐾', '🌸', '⚡', '🔥', '🎯'];

interface ShapePickerDropdownProps {
    onDrawShape?: (shape: ShapeType, sizeMeters: number) => void;
    onDrawEmoji?: (emoji: string, sizeMeters: number) => void;
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
    children,
    placement = 'topLeft',
}: ShapePickerDropdownProps) => {
    const [size, setSize] = useState(5);
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
                    onClick={() => onDrawShape?.(s.key, size)}
                    style={menuItemStyle}
                    onMouseOver={hoverOn}
                    onMouseOut={hoverOff}
                >
                    {s.icon}
                    {t(s.labelKey)}
                </button>
            ))}

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
                        onClick={() => onDrawEmoji?.(emoji, size)}
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
