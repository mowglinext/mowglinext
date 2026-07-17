import { useCallback, useEffect, useMemo, useState } from "react";
import { useTranslation } from "react-i18next";
import { Alert, App, Badge, Button, Empty, Input, Spin, Typography } from "antd";
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
import { DriveMotorSection } from "../components/settings/DriveMotorSection.tsx";
import { NtripSection } from "../components/settings/NtripSection.tsx";
import { PositioningSection } from "../components/settings/PositioningSection.tsx";
import { SensorsSection } from "../components/settings/SensorsSection.tsx";
import { LocalizationSection } from "../components/settings/LocalizationSection.tsx";
import { MowingSection } from "../components/settings/MowingSection.tsx";
import { DockingSection } from "../components/settings/DockingSection.tsx";
import { BatterySection } from "../components/settings/BatterySection.tsx";
import { SafetySection } from "../components/settings/SafetySection.tsx";
import { ObstaclesSection } from "../components/settings/ObstaclesSection.tsx";
import { NavigationSection } from "../components/settings/NavigationSection.tsx";
import { RainSection } from "../components/settings/RainSection.tsx";
import { AdvancedSection } from "../components/settings/AdvancedSection.tsx";
import { SettingsPreview } from "../components/settings/SettingsPreview.tsx";

const { Text } = Typography;

export const SettingsPage = () => {
    const { t } = useTranslation();
    const { modal } = App.useApp();
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
        matchesSearch,
        handleChange,
        handleBulkChange,
        isSectionDirty,
        hasDefault,
        isOverridden,
        resetToDefault,
        save,
        savePartialValues,
        saveAndRestartGps,
        acceptPersistedValues,
        revert,
        gpsRestarting,
    } = useSettingsManager();

    // Long-running: container restart + rosbridge reconnect. Disable button
    // until ROS2 is reachable again to avoid duplicate-click restart storms.
    const ros2Restart = useContainerRestart({
        pendingLabel: t('settingsPage.ros2Restarting'),
        successMessage: t('settingsPage.ros2Restarted'),
        errorMessage: t('settingsPage.ros2RestartFailed'),
    });
    const handleRestartRos2 = useCallback(
        () => ros2Restart.run(() => restartRos2(guiApi)),
        [ros2Restart, guiApi],
    );
    const confirmRestartRos2 = useCallback(() => {
        modal.confirm({
            title: t("settingsPage.restartConfirmTitle"),
            content: t("settingsPage.restartConfirmBody"),
            okText: t("settingsPage.restartConfirmOk"),
            cancelText: t("settingsPage.restartConfirmCancel"),
            onOk: handleRestartRos2,
        });
    }, [modal, t, handleRestartRos2]);

    // Search filter: a section is visible when the query is empty, or when it
    // matches ANY of the section's keys, or the section's translated
    // label/description. Empty query shows everything.
    const visibleSections = useMemo(() => {
        if (!searchQuery) return sections;
        return sections.filter(
            (section) =>
                section.keys.some((key) => matchesSearch(key)) ||
                matchesSearch("", t(section.label)) ||
                matchesSearch("", t(section.description)),
        );
    }, [sections, searchQuery, matchesSearch, t]);

    // When the active section gets filtered out by the search, jump to the
    // first still-visible section so the content pane never goes blank.
    useEffect(() => {
        if (visibleSections.length === 0) return;
        if (!visibleSections.some((s) => s.id === activeSection)) {
            setActiveSection(visibleSections[0].id);
        }
    }, [visibleSections, activeSection]);

    const renderSection = () => {
        switch (activeSection) {
            case "hardware":
                return (
                    <HardwareSection
                        values={values}
                        onChange={handleChange}
                        onBulkChange={handleBulkChange}
                        isOverridden={isOverridden}
                        hasDefault={hasDefault}
                        onReset={resetToDefault}
                    />
                );
            case "drive_motor":
                return (
                    <DriveMotorSection
                        values={values}
                        onChange={handleChange}
                        acceptPersistedValues={acceptPersistedValues}
                    />
                );
            case "ntrip":
                return <NtripSection values={values} onChange={handleChange} />;
            case "positioning":
                return (
                    <PositioningSection
                        values={values}
                        onChange={handleChange}
                        isDirty={isDirty}
                        saving={saving}
                        gpsRestarting={gpsRestarting}
                        onSave={save}
                        onPersistGnssSettings={(settings) => savePartialValues(settings, {
                            silentSuccess: true,
                            errorMessage: t("settingsPage.gnssSaveError"),
                        })}
                        onSaveAndRestartGps={saveAndRestartGps}
                    />
                );
            case "sensors":
                return <SensorsSection values={values} onChange={handleChange} />;
            case "localization":
                return <LocalizationSection values={values} onChange={handleChange} />;
            case "mowing":
                return (
                    <MowingSection
                        values={values}
                        onChange={handleChange}
                        isOverridden={isOverridden}
                        hasDefault={hasDefault}
                        onReset={resetToDefault}
                    />
                );
            case "docking":
                return (
                    <DockingSection
                        values={values}
                        onChange={handleChange}
                        isOverridden={isOverridden}
                        hasDefault={hasDefault}
                        onReset={resetToDefault}
                    />
                );
            case "battery":
                return (
                    <BatterySection
                        values={values}
                        onChange={handleChange}
                        isOverridden={isOverridden}
                        hasDefault={hasDefault}
                        onReset={resetToDefault}
                    />
                );
            case "safety":
                return <SafetySection values={values} onChange={handleChange} />;
            case "obstacles":
                return (
                    <ObstaclesSection
                        values={values}
                        onChange={handleChange}
                        isOverridden={isOverridden}
                        hasDefault={hasDefault}
                        onReset={resetToDefault}
                    />
                );
            case "navigation":
                return (
                    <NavigationSection
                        values={values}
                        onChange={handleChange}
                        isOverridden={isOverridden}
                        hasDefault={hasDefault}
                        onReset={resetToDefault}
                    />
                );
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
        <div style={{ minHeight: isMobile ? "auto" : "calc(100vh - 64px)", display: "flex", flexDirection: "column" }}>
            {/* Header bar */}
            <div style={{
                padding: isMobile ? "12px 12px 0" : "16px 24px 0",
                flexShrink: 0,
            }}>
                {/* Search + save status */}
                <div style={{ display: "flex", alignItems: "center", gap: 12, marginBottom: 12 }}>
                    <Input
                        prefix={<SearchOutlined style={{ color: colors.muted }} />}
                        placeholder={t('settingsPage.searchSettingPlaceholder')}
                        value={searchQuery}
                        onChange={(e) => setSearchQuery(e.target.value)}
                        allowClear
                        style={{ maxWidth: 280 }}
                    />
                    <div style={{ flex: 1 }} />
                    {isDirty && (
                        <Badge count={dirtyKeys.size} size="small" offset={[-4, 0]}>
                            <Text type="secondary" style={{ fontSize: 11 }}>{t("settingsPage.unsavedChanges")}</Text>
                        </Badge>
                    )}
                </div>

                {/* Restart banner */}
                {restartRequired && (
                    <Alert
                        type="warning"
                        showIcon
                        message={t("settingsPage.restartRequired")}
                        action={
                            <Button
                                size="small"
                                type="primary"
                                icon={<ReloadOutlined />}
                                onClick={confirmRestartRos2}
                                loading={ros2Restart.pending}
                                disabled={ros2Restart.pending}
                            >
                                {ros2Restart.pending ? ros2Restart.pendingLabel : t("settingsPage.restartRos2")}
                            </Button>
                        }
                        style={{ marginBottom: 12 }}
                    />
                )}
            </div>

            {/* Main content. We no longer pin this to a fixed viewport height —
                the AppShell <main> scrolls the whole page instead, which avoids
                the calc(100vh - 64px) fragility (mobile URL bars, nested scroll
                traps). The nav and preview rails are made sticky on desktop so
                they stay in view while the section content scrolls naturally. */}
            <div style={{
                flex: 1,
                display: "flex",
                flexDirection: isMobile ? "column" : "row",
                minHeight: 0,
            }}>
                {/* Navigation */}
                <div style={{
                    width: isMobile ? "100%" : 200,
                    flexShrink: 0,
                    paddingLeft: isMobile ? 0 : 8,
                    overflowX: isMobile ? "auto" : undefined,
                    position: isMobile ? undefined : "sticky",
                    top: isMobile ? undefined : 8,
                    alignSelf: isMobile ? undefined : "flex-start",
                }}>
                    {visibleSections.length > 0 ? (
                        <SettingsNav
                            sections={visibleSections}
                            activeSection={activeSection}
                            onSectionChange={setActiveSection}
                            isSectionDirty={isSectionDirty}
                        />
                    ) : (
                        <Empty
                            image={Empty.PRESENTED_IMAGE_SIMPLE}
                            description={t("settingsPage.noSearchResults")}
                            style={{ marginTop: 24 }}
                        />
                    )}
                </div>

                {/* Section content */}
                <div style={{
                    flex: 1,
                    // Extra bottom space on mobile so content scrolls clear of the
                    // fixed save bar (~92px) + bottom nav stacked below it.
                    padding: isMobile ? "0 12px 180px" : "0 24px 120px 16px",
                    minWidth: 0,
                }}>
                    {/* Section header */}
                    {visibleSections.length > 0 && currentSectionMeta && (
                        <div style={{ marginBottom: 20 }}>
                            <div className="mn-display" style={{
                                fontSize: 28, color: colors.text, lineHeight: 1.1, letterSpacing: '-0.01em',
                            }}>
                                {t(currentSectionMeta.label)}
                            </div>
                            <div style={{
                                fontSize: 12, color: colors.textDim, marginTop: 4,
                            }}>
                                {t(currentSectionMeta.description)}
                            </div>
                        </div>
                    )}

                    {visibleSections.length > 0 && renderSection()}
                </div>

                {/* Live preview rail (desktop only) */}
                {!isMobile && (
                    <div style={{
                        width: 260, flexShrink: 0,
                        padding: "0 16px 120px 0",
                        position: "sticky",
                        top: 8,
                        alignSelf: "flex-start",
                    }}>
                        <SettingsPreview values={values} section={activeSection}/>
                    </div>
                )}
            </div>

            {/* Fixed save bar */}
            <div style={{
                position: "fixed",
                // Sit above the floating bottom-nav (~85px tall + safe-area) so the
                // Save bar doesn't collide with / hide behind the nav on mobile.
                bottom: isMobile ? "calc(env(safe-area-inset-bottom, 0px) + 92px)" : 0,
                left: isMobile ? 0 : undefined,
                right: 0,
                padding: "10px 16px",
                background: colors.bgCard,
                borderTop: `1px solid ${colors.border}`,
                zIndex: 51,
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
                    {isDirty ? t("settingsPage.saveWithCount", {count: dirtyKeys.size}) : t("settingsPage.saved")}
                </Button>
                {isDirty && (
                    <Button
                        icon={<UndoOutlined />}
                        onClick={revert}
                    >
                        {t("settingsPage.revert")}
                    </Button>
                )}
                <div style={{ flex: 1 }} />
                <Button
                    icon={<ReloadOutlined />}
                    onClick={confirmRestartRos2}
                    size="small"
                    loading={ros2Restart.pending}
                    disabled={ros2Restart.pending}
                >
                    {ros2Restart.pending ? ros2Restart.pendingLabel : t("settingsPage.restartRos2")}
                </Button>
            </div>
        </div>
    );
};

export default SettingsPage;
