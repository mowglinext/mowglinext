import {Button, InputNumber, Slider} from "antd";
import {useTranslation} from "react-i18next";
import {CompassOutlined} from "@ant-design/icons";
import {useThemeMode} from "../../../theme/ThemeContext.tsx";

interface MapOffsetPanelProps {
    offsetX: number;
    offsetY: number;
    bearing: number;
    onChangeX: (v: number) => void;
    onChangeY: (v: number) => void;
    onChangeBearing: (v: number) => void;
}

export const MapOffsetPanel = ({offsetX, offsetY, bearing, onChangeX, onChangeY, onChangeBearing}: MapOffsetPanelProps) => {
    const {colors} = useThemeMode();
    const {t} = useTranslation();
    return (
    <div>
        <div style={{
            fontSize: 12,
            fontWeight: 600,
            color: colors.muted,
            textTransform: 'uppercase' as const,
            letterSpacing: '0.05em',
            marginBottom: 6,
        }}>
            {t('mapOffsetPanel.mapOffset')}
        </div>
        <div style={{display: 'flex', gap: 8}}>
            <div style={{flex: 1}}>
                <label style={{fontSize: 11, color: colors.textSecondary, display: 'block', marginBottom: 2}}>X</label>
                <InputNumber size="small" value={offsetX} onChange={(v) => onChangeX(v ?? 0)} min={-30} max={30} step={0.01} style={{width: '100%'}}/>
            </div>
            <div style={{flex: 1}}>
                <label style={{fontSize: 11, color: colors.textSecondary, display: 'block', marginBottom: 2}}>Y</label>
                <InputNumber size="small" value={offsetY} onChange={(v) => onChangeY(v ?? 0)} min={-30} max={30} step={0.01} style={{width: '100%'}}/>
            </div>
        </div>

        <div style={{
            fontSize: 12,
            fontWeight: 600,
            color: colors.muted,
            textTransform: 'uppercase' as const,
            letterSpacing: '0.05em',
            marginTop: 10,
            marginBottom: 6,
        }}>
            {t('mapOffsetPanel.mapRotation')}
        </div>
        <div style={{display: 'flex', gap: 8, alignItems: 'center'}}>
            <div style={{flex: 1}}>
                <Slider
                    min={-180}
                    max={180}
                    step={1}
                    value={bearing}
                    onChange={(v) => onChangeBearing(v as number)}
                    tooltip={{formatter: (v) => `${v}°`}}
                />
            </div>
            <InputNumber
                size="small"
                value={Math.round(bearing)}
                onChange={(v) => onChangeBearing(v ?? 0)}
                min={-180}
                max={180}
                step={1}
                style={{width: 70}}
            />
            <Button
                size="small"
                icon={<CompassOutlined/>}
                onClick={() => onChangeBearing(0)}
                title={t('mapOffsetPanel.resetToNorthUp')}
            />
        </div>
    </div>
    );
};
