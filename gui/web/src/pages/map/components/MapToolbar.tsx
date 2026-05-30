import {App, Button, Dropdown, Space} from "antd";
import type {MenuProps} from "antd";
import type {MenuItemType} from "antd/es/menu/interface";
import {
    GlobalOutlined,
    EllipsisOutlined,
    EditOutlined,
    DatabaseOutlined,
    DownloadOutlined,
    ControlOutlined,
    StopOutlined,
    PlayCircleOutlined,
    HomeOutlined,
    WarningOutlined,
    ScissorOutlined,
    AimOutlined,
    ForwardOutlined,
    PauseOutlined,
    CaretRightOutlined,
    ThunderboltOutlined,
    CheckOutlined,
    CloseOutlined,
    ImportOutlined,
} from "@ant-design/icons";
import type {MenuInfo} from "rc-menu/lib/interface";
import AsyncButton from "../../../components/AsyncButton.tsx";
import AsyncDropDownButton from "../../../components/AsyncDropDownButton.tsx";
import type {Feature} from "geojson";

interface MowingAreaItem extends MenuItemType {
    feat: Feature;
}

interface MapToolbarProps {
    manualMode: boolean;
    useSatellite: boolean;
    mowingAreas: MowingAreaItem[];
    stateName?: string;
    emergency?: boolean;
    onEditMap: () => void;
    onToggleSatellite: () => void;
    onManualMode: () => Promise<void>;
    onStopManualMode: () => Promise<void>;
    onBackupMap: () => void;
    onRestoreMap: () => void;
    onDownloadGeoJSON: () => void;
    onImportOpenMower: () => void;
    onMowArea: (key: string) => Promise<void>;
    pitched?: boolean;
    onTogglePitch?: () => void;
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

export const MapToolbar = ({
    manualMode, useSatellite, mowingAreas, stateName, emergency,
    onEditMap, onToggleSatellite,
    onManualMode, onStopManualMode,
    onBackupMap, onRestoreMap, onDownloadGeoJSON, onImportOpenMower,
    onMowArea, pitched, onTogglePitch,
    onStart, onHome, onEmergencyOn, onEmergencyOff,
    onAreaRecording, onMowNextArea, onContinueOrPause,
    onBladeForward, onBladeBackward, onBladeOff,
    onRecordFinish, onRecordCancel,
}: MapToolbarProps) => {
    const {notification} = App.useApp();
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

    const moreMenuItems: MenuProps["items"] = [
        {key: "satellite", icon: <GlobalOutlined />, label: useSatellite ? "Dark map" : "Satellite"},
        ...(onTogglePitch
            ? [{key: "pitch", icon: <GlobalOutlined />, label: pitched ? "Flatten map" : "Tilt 3D view"} satisfies NonNullable<MenuProps["items"]>[number]]
            : []),
        {type: "divider"},
        {key: "areaRecording", icon: <AimOutlined />, label: "Area Recording"},
        {key: "mowNext", icon: <ForwardOutlined />, label: "Mow Next Area"},
        {key: "continueOrPause", icon: isIdle ? <CaretRightOutlined /> : <PauseOutlined />, label: isIdle ? "Continue" : "Pause"},
        {type: "divider"},
        ...(manualMode
            ? [{key: "stopManual", icon: <StopOutlined />, label: "Stop Manual Mowing", danger: true} satisfies NonNullable<MenuProps["items"]>[number]]
            : [{key: "manual", icon: <ControlOutlined />, label: "Manual Mowing"} satisfies NonNullable<MenuProps["items"]>[number]]
        ),
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
    ];

    const handleMoreClick: MenuProps["onClick"] = ({key}: MenuInfo) => {
        switch (key) {
            case "satellite": onToggleSatellite(); break;
            case "pitch": onTogglePitch?.(); break;
            case "manual": safeCall(() => onManualMode()); break;
            case "stopManual": safeCall(() => onStopManualMode()); break;
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
        }
    };

    return (
        <Space size="small" wrap>
            <Button
                type="primary"
                icon={<EditOutlined />}
                onClick={onEditMap}
            >
                Edit Map
            </Button>

            {isRecording ? (
                <>
                    <AsyncButton
                        type="primary"
                        icon={<CheckOutlined />}
                        onAsyncClick={onRecordFinish!}
                    >
                        Finish Recording
                    </AsyncButton>
                    <AsyncButton
                        danger
                        icon={<CloseOutlined />}
                        onAsyncClick={onRecordCancel!}
                    >
                        Cancel Recording
                    </AsyncButton>
                </>
            ) : isIdle ? (
                <AsyncButton
                    type="primary"
                    icon={<PlayCircleOutlined />}
                    onAsyncClick={onStart!}
                >
                    Start
                </AsyncButton>
            ) : (
                <AsyncButton
                    type="primary"
                    icon={<HomeOutlined />}
                    onAsyncClick={onHome!}
                >
                    Home
                </AsyncButton>
            )}

            {!emergency ? (
                <AsyncButton
                    danger
                    icon={<WarningOutlined />}
                    onAsyncClick={onEmergencyOn!}
                >
                    Emergency On
                </AsyncButton>
            ) : (
                <AsyncButton
                    danger
                    icon={<WarningOutlined />}
                    onAsyncClick={onEmergencyOff!}
                >
                    Emergency Off
                </AsyncButton>
            )}

            <AsyncDropDownButton
                icon={<ScissorOutlined />}
                menu={{
                    items: mowingAreas,
                    onAsyncClick: (e: MenuInfo) => onMowArea(e.key),
                }}
            >
                Mow area
            </AsyncDropDownButton>

            <Dropdown
                menu={{items: moreMenuItems, onClick: handleMoreClick}}
                trigger={["click"]}
            >
                <Button icon={<EllipsisOutlined />}>More</Button>
            </Dropdown>
        </Space>
    );
};
