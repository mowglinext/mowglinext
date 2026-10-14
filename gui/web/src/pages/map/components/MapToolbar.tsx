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
    PlayCircleOutlined,
    HomeOutlined,
    WarningOutlined,
    ScissorOutlined,
    AimOutlined,
    CaretRightOutlined,
    PauseOutlined,
    ThunderboltOutlined,
    CheckOutlined,
    CloseOutlined,
    ImportOutlined,
    DeleteOutlined,
    BarsOutlined,
    ExpandOutlined,
} from "@ant-design/icons";
import type {MenuInfo} from "rc-menu/lib/interface";
import {useTranslation} from "react-i18next";
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
    highLevelState?: number;
    emergency?: boolean;
    onEditMap: () => void;
    onToggleSatellite: () => void;
    showObstacleClearance?: boolean;
    onToggleObstacleClearance?: () => void;
    showCoveragePreview?: boolean;
    onToggleCoveragePreview?: () => void;
    onManualMode: () => Promise<void>;
    onStopManualMode: () => Promise<void>;
    onBackupMap: () => void;
    onRestoreMap: () => void;
    onDownloadGeoJSON: () => void;
    onImportOpenMower: () => void;
    onResetMowingProgress: () => void;
    onMowArea: (key: string) => Promise<void>;
    pitched?: boolean;
    onTogglePitch?: () => void;
    onStart?: () => Promise<void>;
    onHome?: () => Promise<void>;
    onEmergencyOn?: () => Promise<void>;
    onEmergencyOff?: () => Promise<void>;
    onAreaRecording?: () => Promise<void>;
    onContinueOrPause?: () => Promise<void>;
    onBladeForward?: () => Promise<void>;
    onBladeBackward?: () => Promise<void>;
    onBladeOff?: () => Promise<void>;
    onRecordFinish?: () => Promise<void>;
    onRecordCancel?: () => Promise<void>;
}

export const MapToolbar = ({
    manualMode, useSatellite, mowingAreas, stateName, highLevelState, emergency,
    onEditMap, onToggleSatellite,
    showObstacleClearance = false, onToggleObstacleClearance,
    showCoveragePreview = false, onToggleCoveragePreview,
    onManualMode, onStopManualMode,
    onBackupMap, onRestoreMap, onDownloadGeoJSON, onImportOpenMower, onResetMowingProgress,
    onMowArea, pitched, onTogglePitch,
    onStart, onHome, onEmergencyOn, onEmergencyOff,
    onAreaRecording, onContinueOrPause,
    onBladeForward, onBladeBackward, onBladeOff,
    onRecordFinish, onRecordCancel,
}: MapToolbarProps) => {
    const {notification} = App.useApp();
    const {t} = useTranslation();
    // DIG_OBSTRUCTION is a held robot (numeric state IDLE, wheels hard-stopped
    // by firmware): the exits are Play after lifting it clear, or Home — so
    // offer Continue, not Pause.
    const isIdle = stateName === "IDLE" || stateName === "IDLE_DOCKED" || stateName === "DIG_OBSTRUCTION";
    const isRecording = stateName === "RECORDING";
    // Numeric state is the authoritative signal. States 2 and above are
    // autonomous, recording, manual mowing, or a future active mode; clearing
    // persisted progress during any of them could race an active mission.
    // Fail closed until the first status frame arrives.
    const resetDisabled = highLevelState === undefined || highLevelState >= 2;

    const safeCall = (fn?: () => Promise<void>) => {
        fn?.().catch((e: Error) => {
            console.error(e);
            notification.error({
                message: t("mapToolbar.actionFailed"),
                description: e.message,
            });
        });
    };

    const moreMenuItems: MenuProps["items"] = [
        {type: "group", label: t("mapToolbar.displayGroup"), children: [
            {key: "satellite", icon: <GlobalOutlined />, label: useSatellite ? t("mapToolbar.darkMap") : t("mapToolbar.satellite")},
            ...(onToggleObstacleClearance
                ? [{
                    key: "obstacleClearance",
                    icon: <ExpandOutlined />,
                    label: showObstacleClearance ? t("mapToolbar.hideObstacleClearance") : t("mapToolbar.showObstacleClearance"),
                } satisfies NonNullable<MenuProps["items"]>[number]]
                : []),
            ...(onToggleCoveragePreview
                ? [{
                    key: "coveragePreview",
                    icon: <BarsOutlined />,
                    label: showCoveragePreview ? t("mapToolbar.hideCoveragePreview") : t("mapToolbar.showCoveragePreview"),
                } satisfies NonNullable<MenuProps["items"]>[number]]
                : []),
            ...(onTogglePitch
                ? [{key: "pitch", icon: <GlobalOutlined />, label: pitched ? t("mapToolbar.flattenMap") : t("mapToolbar.tilt3dView")} satisfies NonNullable<MenuProps["items"]>[number]]
                : []),
            {key: "hardwareAppearance", label: <a href="#/settings?section=hardware">{t("mowerVisual.hardwareLink")}</a>},
        ]},
        {type: "group", label: t("mapToolbar.motionGroup"), children: [
            {key: "areaRecording", icon: <AimOutlined />, label: t("mapToolbar.areaRecording")},
            {key: "continueOrPause", icon: isIdle ? <CaretRightOutlined /> : <PauseOutlined />, label: isIdle ? t("mapToolbar.continue") : t("mapToolbar.pause")},
            {type: "divider"},
            ...(manualMode
                ? [{key: "stopManual", icon: <HomeOutlined />, label: t("mapToolbar.stopManualMowing"), danger: true} satisfies NonNullable<MenuProps["items"]>[number]]
                : [{key: "manual", icon: <ControlOutlined />, label: t("mapToolbar.manualMowing")} satisfies NonNullable<MenuProps["items"]>[number]]
            ),
        ]},
        {type: "group", label: t("mapToolbar.bladeGroup"), children: [
            {key: "bladeForward", icon: <ThunderboltOutlined />, label: t("mapToolbar.bladeForward")},
            {key: "bladeBackward", icon: <ThunderboltOutlined />, label: t("mapToolbar.bladeBackward")},
            {key: "bladeOff", icon: <ThunderboltOutlined />, label: t("mapToolbar.bladeOff"), danger: true},
        ]},
        {type: "group", label: t("mapToolbar.filesGroup"), children: [
            {key: "backup", icon: <DatabaseOutlined />, label: t("mapToolbar.backupMap")},
            {key: "restore", icon: <DatabaseOutlined />, label: t("mapToolbar.restoreMap")},
            {key: "importOpenMower", icon: <ImportOutlined />, label: t("mapToolbar.importFromOpenMower")},
            {
                key: "resetMowingProgress",
                icon: <DeleteOutlined />,
                label: t("resetMowingProgress.action"),
                danger: true,
                disabled: resetDisabled,
            },
            {type: "divider"},
            {key: "download", icon: <DownloadOutlined />, label: t("mapToolbar.downloadGeojson")},
        ]},
    ];

    const handleMoreClick: MenuProps["onClick"] = ({key}: MenuInfo) => {
        switch (key) {
            case "satellite": onToggleSatellite(); break;
            case "obstacleClearance": onToggleObstacleClearance?.(); break;
            case "coveragePreview": onToggleCoveragePreview?.(); break;
            case "pitch": onTogglePitch?.(); break;
            case "manual": safeCall(() => onManualMode()); break;
            case "stopManual": safeCall(() => onStopManualMode()); break;
            case "areaRecording": safeCall(onAreaRecording); break;
            case "continueOrPause": safeCall(onContinueOrPause); break;
            case "bladeForward": safeCall(onBladeForward); break;
            case "bladeBackward": safeCall(onBladeBackward); break;
            case "bladeOff": safeCall(onBladeOff); break;
            case "backup": onBackupMap(); break;
            case "restore": onRestoreMap(); break;
            case "importOpenMower": onImportOpenMower(); break;
            case "resetMowingProgress": onResetMowingProgress(); break;
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
                {t("mapToolbar.editMap")}
            </Button>

            {isRecording ? (
                <>
                    <AsyncButton
                        type="primary"
                        icon={<CheckOutlined />}
                        onAsyncClick={onRecordFinish!}
                    >
                        {t("mapToolbar.finishRecording")}
                    </AsyncButton>
                    <AsyncButton
                        danger
                        icon={<CloseOutlined />}
                        onAsyncClick={onRecordCancel!}
                    >
                        {t("mapToolbar.cancelRecording")}
                    </AsyncButton>
                </>
            ) : (
                <>
                    {isIdle && (
                        <AsyncButton
                            type="primary"
                            icon={<PlayCircleOutlined />}
                            onAsyncClick={onStart!}
                        >
                            {t("mapToolbar.start")}
                        </AsyncButton>
                    )}
                    {/* Home (return-to-dock) is always available outside recording
                        so the robot can be sent back even while idle off-dock. */}
                    <AsyncButton
                        type={isIdle ? "default" : "primary"}
                        icon={<HomeOutlined />}
                        onAsyncClick={onHome!}
                    >
                        {t("mapToolbar.home")}
                    </AsyncButton>
                </>
            )}

            <AsyncButton
                    danger
                    type="primary"
                    className="emergency-stop"
                    aria-label={t("mapToolbar.emergencyOn")}
                    icon={<WarningOutlined aria-hidden />}
                    onAsyncClick={onEmergencyOn!}
                >
                    {t("mapToolbar.emergencyOn")}
            </AsyncButton>
            {emergency && (
                <AsyncButton
                    icon={<WarningOutlined />}
                    onAsyncClick={onEmergencyOff!}
                >
                    {t("mapToolbar.emergencyOff")}
                </AsyncButton>
            )}

            <AsyncDropDownButton
                icon={<ScissorOutlined />}
                menu={{
                    items: mowingAreas,
                    onAsyncClick: (e: MenuInfo) => onMowArea(e.key),
                }}
            >
                {t("mapToolbar.mowArea")}
            </AsyncDropDownButton>

            <AsyncButton
                danger={manualMode}
                icon={manualMode ? <HomeOutlined /> : <ControlOutlined />}
                onAsyncClick={manualMode ? onStopManualMode : onManualMode}
            >
                {manualMode ? t("mapToolbar.stopManual") : t("mapToolbar.manualMow")}
            </AsyncButton>

            <Dropdown
                menu={{items: moreMenuItems, onClick: handleMoreClick, style: {maxHeight: "70dvh", overflowY: "auto"}}}
                trigger={["click"]}
            >
                <Button icon={<EllipsisOutlined />}>{t("mapToolbar.more")}</Button>
            </Dropdown>
        </Space>
    );
};
