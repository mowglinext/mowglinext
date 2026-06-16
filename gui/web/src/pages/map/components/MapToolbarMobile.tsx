import {useState} from "react";
import {useThemeMode} from "../../../theme/ThemeContext.tsx";
import {App, Button, Dropdown, Space} from "antd";
import type {MenuProps} from "antd";
import {
    UndoOutlined,
    RedoOutlined,
    GlobalOutlined,
    EllipsisOutlined,
    SaveOutlined,
    EditOutlined,
    FormOutlined,
    CloseOutlined,
    DeleteOutlined,
    MergeCellsOutlined,
    DatabaseOutlined,
    DownloadOutlined,
    UploadOutlined,
    ScissorOutlined,
    ControlOutlined,
    StopOutlined,
    SplitCellsOutlined,
    MinusSquareOutlined,
    PlayCircleOutlined,
    HomeOutlined,
    WarningOutlined,
    PlusOutlined,
    BorderOutlined,
    AimOutlined,
    ForwardOutlined,
    PauseOutlined,
    CaretRightOutlined,
    ThunderboltOutlined,
    ImportOutlined,
} from "@ant-design/icons";
import type {MenuInfo} from "rc-menu/lib/interface";
import AsyncButton from "../../../components/AsyncButton.tsx";
import type {Feature} from "geojson";
import type {MenuItemType} from "antd/es/menu/interface";
import {ShapePickerDropdown} from "./ShapePickerDropdown.tsx";
import type {ShapeType} from "../hooks/useMapEditing.ts";

interface MowingAreaItem extends MenuItemType {
    feat: Feature;
}

interface MapToolbarMobileProps {
    editMap: boolean;
    hasUnsavedChanges: boolean;
    manualMode: boolean;
    useSatellite: boolean;
    historyIndex: number;
    editHistoryLength: number;
    mowingAreas: MowingAreaItem[];
    onEditMap: () => void;
    onSaveMap: () => Promise<void>;
    onUndo: () => void;
    onRedo: () => void;
    onToggleSatellite: () => void;
    onManualMode: () => Promise<void>;
    onStopManualMode: () => Promise<void>;
    onBackupMap: () => void;
    onRestoreMap: () => void;
    onDownloadGeoJSON: () => void;
    onUploadGeoJSON: () => void;
    onImportOpenMower: () => void;
    onMowArea: (key: string) => Promise<void>;
    selectedFeatureCount?: number;
    onEditSelectedFeature?: () => void;
    onDrawPolygon?: () => void;
    onDrawShape?: (shape: ShapeType, sizeMeters: number) => void;
    onDrawEmoji?: (emoji: string, sizeMeters: number) => void;
    onTrash?: () => void;
    onCombine?: () => void;
    onSubtract?: () => void;
    onSplit?: () => void;
    onPlaceDock?: () => void;
    dockPlacementMode?: boolean;
    stateName?: string;
    emergency?: boolean;
    onStart?: () => Promise<void>;
    onHome?: () => Promise<void>;
    onEmergencyOn?: () => Promise<void>;
    onEmergencyOff?: () => Promise<void>;
    onAreaRecording?: () => Promise<void>;
    onMowNextArea?: () => Promise<void>;
    onContinueOrPause?: () => Promise<void>;
    onBladeForward?: () => Promise<void>;
    onBladeBackward?: () => Promise<void>;
    onBladeOff?: () => Promise<void>;
    onRecordFinish?: () => Promise<void>;
    onRecordCancel?: () => Promise<void>;
}

export const MapToolbarMobile = ({
    editMap, hasUnsavedChanges, manualMode, useSatellite,
    historyIndex, editHistoryLength, mowingAreas,
    onEditMap, onSaveMap, onUndo, onRedo, onToggleSatellite,
    onManualMode, onStopManualMode,
    onBackupMap, onRestoreMap, onDownloadGeoJSON, onUploadGeoJSON, onImportOpenMower,
    onMowArea, selectedFeatureCount = 0, onEditSelectedFeature,
    onDrawPolygon, onDrawShape, onDrawEmoji, onTrash, onCombine, onSubtract, onSplit,
    onPlaceDock, dockPlacementMode,
    stateName, emergency,
    onStart, onHome, onEmergencyOn, onEmergencyOff,
    onAreaRecording, onMowNextArea, onContinueOrPause,
    onBladeForward, onBladeBackward, onBladeOff,
}: MapToolbarMobileProps) => {
    const {colors} = useThemeMode();
    const {notification} = App.useApp();
    const [mowLoading, setMowLoading] = useState(false);

    // 44px minimum touch target on every control in the cluster (Apple/WCAG
    // thumb-reach guideline). Applied via a shared style so size="large" AntD
    // buttons never fall below the floor.
    const touchTarget: React.CSSProperties = {minWidth: 44, minHeight: 44};

    const toolbarStyle: React.CSSProperties = {
        // Anchor to the VIEWPORT (fixed), not the map container — that container
        // bleeds past the viewport (height: 100% + 122px, negative bottom margin),
        // and iOS Safari positions an absolute child relative to that off-screen
        // bottom, hiding the toolbar entirely. Fixed keeps it just above the nav.
        position: "fixed",
        // Sit just above the floating bottom-nav (≈85px tall + safe-area). Derive
        // the offset from the safe-area inset so it tracks the nav height on
        // notched phones, with a comfortable gap above the nav pill.
        bottom: "calc(env(safe-area-inset-bottom, 0px) + 100px)",
        // Leave room on the right for the pinned STOP button so the scrolling
        // cluster never slides under it.
        left: 12,
        right: 80,
        // Above the bottom-nav (zIndex 50) so the nav never paints over it.
        zIndex: 55,
        display: "flex",
        gap: 8,
        // Horizontally scrollable cluster — never force a multi-row wrap that
        // would push controls under the bottom-nav on short phones.
        flexWrap: "nowrap",
        overflowX: "auto",
        overflowY: "hidden",
        WebkitOverflowScrolling: "touch",
        alignItems: "center",
        background: colors.glassBackground,
        backdropFilter: "blur(22px) saturate(140%)",
        WebkitBackdropFilter: "blur(22px) saturate(140%)",
        borderRadius: 18,
        border: colors.glassBorder,
        boxShadow: colors.glassShadow,
        padding: "8px 12px",
    };

    // The emergency / STOP button is deliberately separated from the tool
    // cluster and pinned to its own fixed corner so a panicking beginner finds
    // it instantly. Larger than every other control, filled rose.
    const stopButtonStyle: React.CSSProperties = {
        position: "fixed",
        bottom: "calc(env(safe-area-inset-bottom, 0px) + 100px)",
        right: 12,
        zIndex: 56,
        width: 60,
        height: 60,
        borderRadius: 18,
        fontWeight: 700,
        background: emergency ? colors.bgElevated : colors.danger,
        borderColor: colors.danger,
        color: emergency ? colors.danger : colors.text,
        boxShadow: colors.glassShadow,
    };

    const isIdle = stateName === "IDLE" || stateName === "IDLE_DOCKED";
    const isRecording = stateName === "RECORDING";

    const safeCall = (fn?: () => Promise<void>) => {
        fn?.().catch((e: Error) => {
            console.error(e);
            notification.error({
                message: "Action failed",
                description: e.message,
            });
        });
    };

    const dataMenuItems: MenuProps["items"] = [
        {key: "satellite", icon: <GlobalOutlined />, label: useSatellite ? "Dark map" : "Satellite"},
        {type: "divider"},
        {key: "areaRecording", icon: <AimOutlined />, label: "Area Recording"},
        {key: "mowNext", icon: <ForwardOutlined />, label: "Mow Next Area"},
        {key: "continueOrPause", icon: isIdle ? <CaretRightOutlined /> : <PauseOutlined />, label: isIdle ? "Continue" : "Pause"},
        {type: "divider"},
        {key: "bladeForward", icon: <ThunderboltOutlined />, label: "Blade Forward"},
        {key: "bladeBackward", icon: <ThunderboltOutlined />, label: "Blade Backward"},
        {key: "bladeOff", icon: <ThunderboltOutlined />, label: "Blade Off", danger: true},
        {type: "divider"},
        {key: "backup", icon: <DatabaseOutlined />, label: "Backup Map"},
        {key: "restore", icon: <DatabaseOutlined />, label: "Restore Map"},
        {key: "importOpenMower", icon: <ImportOutlined />, label: "Import from OpenMower"},
        {type: "divider"},
        {key: "download", icon: <DownloadOutlined />, label: "Download GeoJSON"},
        ...(editMap
            ? [{key: "upload", icon: <UploadOutlined />, label: "Upload GeoJSON"} satisfies NonNullable<MenuProps["items"]>[number]]
            : []),
    ];

    const handleMoreClick: MenuProps["onClick"] = ({key}: MenuInfo) => {
        switch (key) {
            case "satellite": onToggleSatellite(); break;
            case "areaRecording": safeCall(onAreaRecording); break;
            case "mowNext": safeCall(onMowNextArea); break;
            case "continueOrPause": safeCall(onContinueOrPause); break;
            case "bladeForward": safeCall(onBladeForward); break;
            case "bladeBackward": safeCall(onBladeBackward); break;
            case "bladeOff": safeCall(onBladeOff); break;
            case "backup": onBackupMap(); break;
            case "restore": onRestoreMap(); break;
            case "importOpenMower": onImportOpenMower(); break;
            case "download": onDownloadGeoJSON(); break;
            case "upload": onUploadGeoJSON(); break;
        }
    };

    const handleMowClick: MenuProps["onClick"] = ({key}: MenuInfo) => {
        setMowLoading(true);
        onMowArea(key).finally(() => setMowLoading(false));
    };

    const editMenuItems: MenuProps["items"] = [
        {key: "editProps", icon: <FormOutlined />, label: "Edit Properties", disabled: selectedFeatureCount !== 1},
        {key: "combine", icon: <MergeCellsOutlined />, label: "Combine", disabled: selectedFeatureCount < 2},
        {key: "subtract", icon: <MinusSquareOutlined />, label: "Subtract", disabled: selectedFeatureCount !== 2},
        {key: "split", icon: <SplitCellsOutlined />, label: "Split (draw line)", disabled: selectedFeatureCount !== 1},
        {type: "divider"},
        ...dataMenuItems,
    ];

    const handleEditMenuClick: MenuProps["onClick"] = ({key}: MenuInfo) => {
        switch (key) {
            case "editProps": onEditSelectedFeature?.(); break;
            case "combine": onCombine?.(); break;
            case "subtract": onSubtract?.(); break;
            case "split": onSplit?.(); break;
            default: handleMoreClick({key} as MenuInfo); break;
        }
    };

    // Always-present, dominant, separated emergency control. Rendered outside
    // the scrolling cluster in its own fixed corner. Keeps the exact emergency
    // on/off commands — only the styling/label/placement changed.
    const stopButton = (
        <AsyncButton
            danger
            type={emergency ? "default" : "primary"}
            icon={<WarningOutlined />}
            onAsyncClick={(emergency ? onEmergencyOff : onEmergencyOn)!}
            aria-label={emergency ? "Emergency Off" : "Emergency On"}
            style={stopButtonStyle}
        >
            STOP
        </AsyncButton>
    );

    if (editMap) {
        return (
            <>
                <div style={toolbarStyle}>
                    {/* Save / Cancel */}
                    <Space.Compact size="large">
                        <AsyncButton
                            type="primary"
                            size="large"
                            danger={hasUnsavedChanges}
                            icon={<SaveOutlined />}
                            onAsyncClick={onSaveMap}
                            aria-label="Save"
                            style={touchTarget}
                        />
                        <Button
                            size="large"
                            icon={<CloseOutlined />}
                            onClick={onEditMap}
                            aria-label="Cancel"
                            style={touchTarget}
                        />
                    </Space.Compact>

                    {/* Undo / Redo */}
                    <Space.Compact size="large">
                        <Button
                            size="large"
                            icon={<UndoOutlined />}
                            onClick={onUndo}
                            disabled={historyIndex <= 0}
                            aria-label="Undo"
                            style={touchTarget}
                        />
                        <Button
                            size="large"
                            icon={<RedoOutlined />}
                            onClick={onRedo}
                            disabled={historyIndex >= editHistoryLength - 1}
                            aria-label="Redo"
                            style={touchTarget}
                        />
                    </Space.Compact>

                    {/* Draw / Add shape / Delete */}
                    <Space.Compact size="large">
                        <Button
                            size="large"
                            icon={<BorderOutlined />}
                            onClick={onDrawPolygon}
                            aria-label="Draw polygon"
                            style={touchTarget}
                        />
                        <ShapePickerDropdown
                            onDrawShape={onDrawShape}
                            onDrawEmoji={onDrawEmoji}
                            placement="top"
                        >
                            <Button size="large" icon={<PlusOutlined />} aria-label="Add shape" style={touchTarget} />
                        </ShapePickerDropdown>
                        <Button
                            size="large"
                            icon={<DeleteOutlined />}
                            disabled={selectedFeatureCount === 0}
                            onClick={onTrash}
                            aria-label="Delete"
                            style={touchTarget}
                        />
                    </Space.Compact>

                    <Button
                        size="large"
                        icon={<AimOutlined />}
                        type={dockPlacementMode ? "primary" : "default"}
                        onClick={onPlaceDock}
                        aria-label="Place dock"
                        style={touchTarget}
                    />

                    {/* Combine/Subtract/Split now live inside this More menu
                        (editMenuItems) to keep the top row uncluttered. */}
                    <Dropdown
                        menu={{items: editMenuItems, onClick: handleEditMenuClick}}
                        trigger={["click"]}
                        placement="topRight"
                    >
                        <Button size="large" icon={<EllipsisOutlined />} aria-label="More" style={touchTarget} />
                    </Dropdown>
                </div>
                {stopButton}
            </>
        );
    }

    // View mode — top row stays ≤5 items:
    //   [primary action] · Edit Map · Mow · Manual · More
    // The emergency control is pulled out into the pinned STOP corner.
    // While RECORDING the toolbar suppresses its own primary action AND the
    // Finish/Cancel record buttons — the JoystickOverlay owns the single
    // Finish/Cancel/Home set so they aren't duplicated.
    return (
        <>
            <div style={toolbarStyle}>
                {!isRecording && (
                    isIdle ? (
                        <AsyncButton
                            type="primary"
                            size="large"
                            icon={<PlayCircleOutlined />}
                            onAsyncClick={onStart!}
                            aria-label="Start"
                            style={touchTarget}
                        />
                    ) : (
                        <AsyncButton
                            type="primary"
                            size="large"
                            icon={<HomeOutlined />}
                            onAsyncClick={onHome!}
                            aria-label="Home"
                            style={touchTarget}
                        />
                    )
                )}

                <Button
                    size="large"
                    icon={<EditOutlined />}
                    onClick={onEditMap}
                    aria-label="Edit Map"
                    style={touchTarget}
                />

                <Dropdown
                    menu={{items: mowingAreas, onClick: handleMowClick}}
                    trigger={["click"]}
                    placement="topLeft"
                >
                    <Button
                        size="large"
                        icon={<ScissorOutlined />}
                        loading={mowLoading}
                        aria-label="Mow area"
                        style={touchTarget}
                    >
                        Mow
                    </Button>
                </Dropdown>

                <AsyncButton
                    size="large"
                    danger={manualMode}
                    icon={manualMode ? <StopOutlined /> : <ControlOutlined />}
                    onAsyncClick={manualMode ? onStopManualMode : onManualMode}
                    aria-label={manualMode ? "Stop Manual Mowing" : "Manual Mowing"}
                    style={touchTarget}
                />

                <Dropdown
                    menu={{items: dataMenuItems, onClick: handleMoreClick}}
                    trigger={["click"]}
                    placement="topRight"
                >
                    <Button size="large" icon={<EllipsisOutlined />} aria-label="More" style={touchTarget} />
                </Dropdown>
            </div>
            {stopButton}
        </>
    );
};
