import {Button, InputNumber, Slider} from "antd";
import {useTranslation} from "react-i18next";
import {CompassOutlined} from "@ant-design/icons";
import {useThemeMode} from "../../../theme/ThemeContext.tsx";

interface MapOffsetPanelProps {
    offsetX: number;
    offsetY: number;
    onChangeX: (v: number) => void;
    onChangeY: (v: number) => void;
}

/// X / Y shift of the map. The title ("Map offset") is the sidebar section header.
export const MapOffsetPanel = ({offsetX, offsetY, onChangeX, onChangeY}: MapOffsetPanelProps) => {
    const {colors} = useThemeMode();
    return (
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
    );
};

interface MapRotationPanelProps {
    bearing: number;
    onChangeBearing: (v: number) => void;
}

/// Map rotation (bearing). The title ("Map rotation") is the sidebar section header.
export const MapRotationPanel = ({bearing, onChangeBearing}: MapRotationPanelProps) => {
    const {t} = useTranslation();
    return (
        <div style={{display: 'flex', gap: 8, alignItems: 'center'}}>
            <div style={{flex: 1}}>
                <Slider
                    min={-180}
                    max={180}
                    step={1}
                    value={bearing}
                    onChange={(v) => onChangeBearing(v)}
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
    );
};
