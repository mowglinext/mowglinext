import {Tooltip} from "antd";
import {
    SaveOutlined,
    CloseOutlined,
    UndoOutlined,
    RedoOutlined,
    BorderOutlined,
    DeleteOutlined,
    MergeCellsOutlined,
    SplitCellsOutlined,
    MinusSquareOutlined,
    FormOutlined,
    PlusOutlined,
    AimOutlined,
} from "@ant-design/icons";
import AsyncButton from "../../../components/AsyncButton.tsx";
import {ShapePickerDropdown} from "./ShapePickerDropdown.tsx";
import type {ShapeType} from "../hooks/useMapEditing.ts";
import {useThemeMode} from "../../../theme/ThemeContext.tsx";

interface MapEditorToolbarProps {
    hasUnsavedChanges: boolean;
    historyIndex: number;
    editHistoryLength: number;
    selectedFeatureCount: number;
    onSaveMap: () => Promise<void>;
    onCancel: () => void;
    onUndo: () => void;
    onRedo: () => void;
    onDrawPolygon?: () => void;
    onDrawShape?: (shape: ShapeType, sizeMeters: number) => void;
    onDrawEmoji?: (emoji: string, sizeMeters: number) => void;
    onTrash?: () => void;
    onCombine?: () => void;
    onSubtract?: () => void;
    onSplit?: () => void;
    onEditSelectedFeature?: () => void;
    onPlaceDock?: () => void;
    dockPlacementMode?: boolean;
}

interface ToolButtonProps {
    icon: React.ReactNode;
    tooltip: string;
    onClick?: () => void;
    disabled?: boolean;
    danger?: boolean;
    primary?: boolean;
    glow?: boolean;
}

const ToolButton = ({icon, tooltip, onClick, disabled, danger, primary, glow}: ToolButtonProps) => {
    const {colors} = useThemeMode();
    return (
    <Tooltip title={tooltip} placement="right">
        <button
            onClick={onClick}
            disabled={disabled}
            style={{
                width: 40,
                height: 40,
                display: 'flex',
                alignItems: 'center',
                justifyContent: 'center',
                background: 'transparent',
                border: 'none',
                borderRadius: 8,
                fontSize: 18,
                cursor: disabled ? 'not-allowed' : 'pointer',
                transition: 'background 0.15s, color 0.15s',
                color: disabled ? colors.muted : danger ? colors.danger : primary ? colors.success : colors.text,
                boxShadow: glow ? `0 0 0 2px ${colors.danger}66` : undefined,
            }}
            onMouseOver={(e) => {
                if (!disabled) e.currentTarget.style.background = colors.bgElevated;
            }}
            onMouseOut={(e) => {
                e.currentTarget.style.background = 'transparent';
            }}
        >
            {icon}
        </button>
    </Tooltip>
    );
};

export const MapEditorToolbar = ({
    hasUnsavedChanges, historyIndex, editHistoryLength,
    selectedFeatureCount, onSaveMap, onCancel, onUndo, onRedo,
    onDrawPolygon, onDrawShape, onDrawEmoji, onTrash, onCombine, onSubtract, onSplit, onEditSelectedFeature,
    onPlaceDock, dockPlacementMode,
}: MapEditorToolbarProps) => {
    const {colors} = useThemeMode();

    const btnStyle: React.CSSProperties = {
        width: 40,
        height: 40,
        display: 'flex',
        alignItems: 'center',
        justifyContent: 'center',
        background: 'transparent',
        border: 'none',
        borderRadius: 8,
        color: colors.text,
        fontSize: 18,
        cursor: 'pointer',
        transition: 'background 0.15s, color 0.15s',
    };

    return (
    <div style={{
        position: 'absolute',
        left: 12,
        top: 12,
        zIndex: 10,
        display: 'flex',
        flexDirection: 'column',
        gap: 2,
        background: colors.glassBackground,
        backdropFilter: 'blur(16px) saturate(180%)',
        WebkitBackdropFilter: 'blur(16px) saturate(180%)',
        borderRadius: 12,
        border: colors.glassBorder,
        boxShadow: colors.glassShadow,
        padding: 6,
    }}>
        {/* Save / Cancel */}
        <Tooltip title={hasUnsavedChanges ? "Save Map *" : "Save Map"} placement="right">
            <div>
                <AsyncButton
                    type="text"
                    icon={<SaveOutlined/>}
                    onAsyncClick={onSaveMap}
                    style={{
                        ...btnStyle,
                        color: hasUnsavedChanges ? colors.danger : colors.success,
                        boxShadow: hasUnsavedChanges ? `0 0 0 2px ${colors.danger}66` : undefined,
                    }}
                />
            </div>
        </Tooltip>
        <ToolButton icon={<CloseOutlined/>} tooltip="Cancel editing" onClick={onCancel}/>

        <div style={{height: 1, background: colors.borderSubtle, margin: '2px 4px'}}/>

        {/* Undo / Redo */}
        <ToolButton icon={<UndoOutlined/>} tooltip="Undo" onClick={onUndo} disabled={historyIndex <= 0}/>
        <ToolButton icon={<RedoOutlined/>} tooltip="Redo" onClick={onRedo} disabled={historyIndex >= editHistoryLength - 1}/>

        <div style={{height: 1, background: colors.borderSubtle, margin: '2px 4px'}}/>

        {/* Drawing tools */}
        <ToolButton icon={<BorderOutlined/>} tooltip="Draw polygon" onClick={onDrawPolygon}/>
        <ShapePickerDropdown onDrawShape={onDrawShape} onDrawEmoji={onDrawEmoji} placement="bottomLeft">
            <Tooltip title="Add shape" placement="right">
                <button
                    style={btnStyle}
                    onMouseOver={(e) => {
                        e.currentTarget.style.background = colors.bgElevated;
                    }}
                    onMouseOut={(e) => {
                        e.currentTarget.style.background = 'transparent';
                    }}
                >
                    <PlusOutlined />
                </button>
            </Tooltip>
        </ShapePickerDropdown>
        <ToolButton icon={<DeleteOutlined/>} tooltip="Delete selected" onClick={onTrash} disabled={selectedFeatureCount === 0} danger/>

        <div style={{height: 1, background: colors.borderSubtle, margin: '2px 4px'}}/>

        {/* Boolean operations */}
        <ToolButton icon={<MergeCellsOutlined/>} tooltip="Combine (select 2+)" onClick={onCombine} disabled={selectedFeatureCount < 2}/>
        <ToolButton icon={<MinusSquareOutlined/>} tooltip="Subtract (select 2)" onClick={onSubtract} disabled={selectedFeatureCount !== 2}/>
        <ToolButton icon={<SplitCellsOutlined/>} tooltip="Split (select 1, draw line)" onClick={onSplit} disabled={selectedFeatureCount !== 1}/>

        <div style={{height: 1, background: colors.borderSubtle, margin: '2px 4px'}}/>

        {/* Edit properties */}
        <ToolButton icon={<FormOutlined/>} tooltip="Edit properties" onClick={onEditSelectedFeature} disabled={selectedFeatureCount !== 1}/>

        <div style={{height: 1, background: colors.borderSubtle, margin: '2px 4px'}}/>

        {/* Dock placement */}
        <ToolButton
            icon={<AimOutlined/>}
            tooltip={dockPlacementMode ? "Click on map to place dock" : "Place dock point"}
            onClick={onPlaceDock}
            primary={dockPlacementMode}
            glow={dockPlacementMode}
        />
    </div>
    );
};
