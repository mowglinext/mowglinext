import { useCallback, useState } from "react";
import { Alert, Badge, Button, Input, Spin, Typography } from "antd";
import {
    ReloadOutlined,
    SaveOutlined,
    SearchOutlined,
    UndoOutlined,
} from "@ant-design/icons";
import { useApi } from "../hooks/useApi.ts";
import { useIsMobile } from "../hooks/useIsMobile.ts";
import { useThemeMode } from "../theme/ThemeContext.tsx";
import { SettingsSection, useSettingsManager } from "../hooks/useSettingsManager.ts";
import { restartRos2 } from "../utils/containers.ts";
import { useContainerRestart } from "../hooks/useContainerRestart.ts";
import { SettingsNav } from "../components/settings/SettingsNav.tsx";
import { HardwareSection } from "../components/settings/HardwareSection.tsx";
import { PositioningSection } from "../components/settings/PositioningSection.tsx";
import { SensorsSection } from "../components/settings/SensorsSection.tsx";
import { LocalizationSection } from "../components/settings/LocalizationSection.tsx";
import { MowingSection } from "../components/settings/MowingSection.tsx";
import { DockingSection } from "../components/settings/DockingSection.tsx";
import { BatterySection } from "../components/settings/BatterySection.tsx";
import { SafetySection } from "../components/settings/SafetySection.tsx";
import { NavigationSection } from "../components/settings/NavigationSection.tsx";
import { RainSection } from "../components/settings/RainSection.tsx";
import { AdvancedSection } from "../components/settings/AdvancedSection.tsx";

const { Text } = Typography;

export const SettingsPage = () => {
    const guiApi = useApi();
    const isMobile = useIsMobile();
    const { colors } = useThemeMode();
    const [activeSection, setActiveSection] = useState<SettingsSection>("hardware");

    const {
        sections,
        values,
        loading,
        saving,
        isDirty,
        dirtyKeys,
        restartRequired,
        searchQuery,
        advancedKeys,
        setSearchQuery,
        handleChange,
        handleBulkChange,
        isSectionDirty,
        save,
        revert,
    } = useSettingsManager();

    // Long-running: container restart + rosbridge reconnect. Disable button
    // until ROS2 is reachable again to avoid duplicate-click restart storms.
    const ros2Restart = useContainerRestart({
        pendingLabel: "Redémarrage ROS2…",
        successMessage: "ROS2 redémarré",
        errorMessage: "Échec du redémarrage ROS2",
    });
    const handleRestartRos2 = useCallback(
        () => ros2Restart.run(() => restartRos2(guiApi)),
        [ros2Restart, guiApi],
    );

    const renderSection = () => {
        switch (activeSection) {
            case "hardware":
                return <HardwareSection values={values} onChange={handleChange} onBulkChange={handleBulkChange} />;
            case "positioning":
                return <PositioningSection values={values} onChange={handleChange} />;
            case "sensors":
                return <SensorsSection values={values} onChange={handleChange} />;
            case "localization":
                return <LocalizationSection values={values} onChange={handleChange} />;
            case "mowing":
                return <MowingSection values={values} onChange={handleChange} />;
            case "docking":
                return <DockingSection values={values} onChange={handleChange} />;
            case "battery":
                return <BatterySection values={values} onChange={handleChange} />;
            case "safety":
                return <SafetySection values={values} onChange={handleChange} />;
            case "navigation":
                return <NavigationSection values={values} onChange={handleChange} />;
            case "rain":
                return <RainSection values={values} onChange={handleChange} />;
            case "advanced":
                return <AdvancedSection values={values} advancedKeys={advancedKeys} onChange={handleChange} />;
            default:
                return null;
        }
    };

    if (loading) {
        return <Spin size="large" style={{ display: "block", margin: "100px auto" }} />;
    }

    const currentSectionMeta = sections.find((s) => s.id === activeSection);

    return (
        <div style={{ height: isMobile ? "auto" : "calc(100vh - 64px)", display: "flex", flexDirection: "column" }}>
            {/* Header bar */}
            <div style={{
                padding: isMobile ? "12px 12px 0" : "16px 24px 0",
                flexShrink: 0,
            }}>
                {/* Search + save status */}
                <div style={{ display: "flex", alignItems: "center", gap: 12, marginBottom: 12 }}>
                    <Input
                        prefix={<SearchOutlined style={{ color: colors.muted }} />}
                        placeholder="Search settings..."
                        value={searchQuery}
                        onChange={(e) => setSearchQuery(e.target.value)}
                        allowClear
                        size="small"
                        style={{ maxWidth: 280 }}
                    />
                    <div style={{ flex: 1 }} />
                    {isDirty && (
                        <Badge count={dirtyKeys.size} size="small" offset={[-4, 0]}>
                            <Text type="secondary" style={{ fontSize: 11 }}>unsaved changes</Text>
                        </Badge>
                    )}
                </div>

                {/* Restart banner */}
                {restartRequired && (
                    <Alert
                        type="warning"
                        showIcon
                        message="Restart required to apply saved changes"
                        action={
                            <Button
                                size="small"
                                type="primary"
                                icon={<ReloadOutlined />}
                                onClick={handleRestartRos2}
                                loading={ros2Restart.pending}
                                disabled={ros2Restart.pending}
                            >
                                {ros2Restart.pending ? ros2Restart.pendingLabel : "Restart ROS2"}
                            </Button>
                        }
                        style={{ marginBottom: 12 }}
                    />
                )}
            </div>

            {/* Main content */}
            <div style={{
                flex: 1,
                display: "flex",
                flexDirection: isMobile ? "column" : "row",
                overflow: "hidden",
                minHeight: 0,
            }}>
                {/* Navigation */}
                <div style={{
                    width: isMobile ? "100%" : 200,
                    flexShrink: 0,
                    paddingLeft: isMobile ? 0 : 8,
                    overflowX: isMobile ? "auto" : undefined,
                    overflowY: isMobile ? undefined : "auto",
                }}>
                    <SettingsNav
                        sections={sections}
                        activeSection={activeSection}
                        onSectionChange={setActiveSection}
                        isSectionDirty={isSectionDirty}
                    />
                </div>

                {/* Section content */}
                <div style={{
                    flex: 1,
                    overflowY: "auto",
                    padding: isMobile ? "0 12px 120px" : "0 24px 120px 16px",
                    minHeight: 0,
                }}>
                    {/* Section header */}
                    {currentSectionMeta && (
                        <div style={{ marginBottom: 16 }}>
                            <Text strong style={{ fontSize: 16 }}>{currentSectionMeta.label}</Text>
                            <br />
                            <Text type="secondary" style={{ fontSize: 12 }}>{currentSectionMeta.description}</Text>
                        </div>
                    )}

                    {renderSection()}
                </div>
            </div>

            {/* Fixed save bar */}
            <div style={{
                position: "fixed",
                bottom: isMobile ? "calc(56px + env(safe-area-inset-bottom, 0px))" : 0,
                left: isMobile ? 0 : undefined,
                right: 0,
                padding: "10px 16px",
                background: colors.bgCard,
                borderTop: `1px solid ${colors.border}`,
                zIndex: 50,
                display: "flex",
                alignItems: "center",
                gap: 8,
            }}>
                <Button
                    type="primary"
                    icon={<SaveOutlined />}
                    onClick={save}
                    loading={saving}
                    disabled={!isDirty}
                >
                    {isDirty ? `Save (${dirtyKeys.size} changes)` : "Saved"}
                </Button>
                {isDirty && (
                    <Button
                        icon={<UndoOutlined />}
                        onClick={revert}
                    >
                        Revert
                    </Button>
                )}
                <div style={{ flex: 1 }} />
                <Button
                    icon={<ReloadOutlined />}
                    onClick={handleRestartRos2}
                    size="small"
                    loading={ros2Restart.pending}
                    disabled={ros2Restart.pending}
                >
                    {ros2Restart.pending ? ros2Restart.pendingLabel : "Restart ROS2"}
                </Button>
            </div>
        </div>
    );
};

export default SettingsPage;
