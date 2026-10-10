import {useSectionFocus} from "../hooks/useSectionFocus.ts";
import {settingSearchText} from "../utils/settingsSearch.ts";
import { useCallback, useMemo, useRef, useEffect } from "react";
import { useTranslation } from "react-i18next";
import { useSearchParams } from "react-router-dom";
import { UpdatesSection } from "../components/settings/UpdatesSection.tsx";
import { Alert, App, Badge, Button, Empty, Input, Space, Spin, Typography } from "antd";
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
import { HardwareBackendSection } from "../components/settings/HardwareBackendSection.tsx";
import { HardwareBackendCard } from "../components/settings/HardwareBackendCard.tsx";
import { HardwareViewSwitcher } from "../components/settings/HardwareViewSwitcher.tsx";
import { FcuFeatureTool } from "../components/settings/FcuFeatureTool.tsx";
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
import { LedsSection } from "../components/settings/LedsSection.tsx";
import { MqttSection } from "../components/settings/MqttSection.tsx";
import { IrriSenseSection } from "../components/settings/IrriSenseSection.tsx";
import { RemoteAccessSection } from "../components/settings/RemoteAccessSection.tsx";
import { NotificationsSection } from "../components/settings/NotificationsSection.tsx";
import { AdvancedSection } from "../components/settings/AdvancedSection.tsx";
import { SettingsPreview } from "../components/settings/SettingsPreview.tsx";
import { DisplayModeSection } from "../components/settings/DisplayModeSection.tsx";
import { LogTimeZoneSection } from "../components/settings/LogTimeZoneSection.tsx";
import { SettingsFieldCard } from "../components/settings/SettingsFieldCard.tsx";
import { FirmwareParamsCard } from "../components/settings/FirmwareParamsCard.tsx";
import {
    AREA_RECORDING_GROUP,
    BEHAVIOR_TREE_GROUP,
    CHARGE_LIMITS_GROUP,
    DOCK_CALIBRATION_GROUP,
    DOCK_DETECTION_GROUP,
    FIRMWARE_SAFETY_GROUP,
    LOCALIZATION_GUARD_GROUP,
    REVERSE_ESCAPE_GROUP,
    START_ESCAPE_GROUP,
    TURN_SPEED_GROUP,
    YAW_LOOP_GROUP,
    OPENMOWER_WHEEL_LOOP_GROUP,
    OPENMOWER_WIRING_GROUP,
    groupForBackend,
    type SettingsFieldGroup,
} from "../components/settings/settingsFieldGroups.ts";

const { Text } = Typography;

export const SettingsPage = () => {
    const { t } = useTranslation();
    const { modal } = App.useApp();
    const guiApi = useApi();
    const isMobile = useIsMobile();
    const { colors } = useThemeMode();
    const [searchParams, setSearchParams] = useSearchParams();
    const selectSection = (section: SettingsSection, field?: string) => {
        setSearchParams((current) => { const next = new URLSearchParams(current); next.set('section', section); if (field) next.set('field', field); else next.delete('field'); return next; }, {replace: true});
    };

    const {
        sections,
        values,
        defaults,
        loading,
        saving,
        isDirty,
        dirtyCount,
        registerExternalSaver,
        unregisterExternalSaver,
        dirtyKeys,
        restartRequired,
        acknowledgeRestart,
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
        hardwareBackend,
        backendDefaultOverrides,
    } = useSettingsManager();

    // Long-running: container restart + rosbridge reconnect. Disable button
    // until ROS2 is reachable again to avoid duplicate-click restart storms.
    const ros2Restart = useContainerRestart({
        onSuccess: acknowledgeRestart,
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

    // Sections merged into "weather" keep their old links working.
    const LEGACY_SECTIONS: Record<string, string> = {
        rain: "weather", irrisense: "weather", hardware_backend: "hardware",
    };
    const rawSection = searchParams.get('section') ?? 'hardware';
    const requestedSection = LEGACY_SECTIONS[rawSection] ?? rawSection;
    const activeSection = visibleSections.find(section => section.id === requestedSection)?.id
        ?? visibleSections[0]?.id ?? 'hardware';

    const sectionHeadingRef = useRef<HTMLHeadingElement>(null);
    useSectionFocus(sectionHeadingRef, requestedSection, !loading);
    const targetField = searchParams.get('field');
    useEffect(() => {
        if (!targetField || loading) return;
        const target = document.querySelector<HTMLElement>(`[data-setting-key="${CSS.escape(targetField)}"]`);
        if (!target) return;
        target.dataset.searchTarget = 'true';
        target.scrollIntoView({block: 'center'});
        const control = target.matches('input, button, [tabindex]') ? target : target.querySelector<HTMLElement>('input, button, [tabindex]');
        control?.focus({preventScroll: true});
        return () => { delete target.dataset.searchTarget; };
    }, [targetField, activeSection, loading]);
    const fieldResults = searchQuery ? sections.flatMap(section => section.keys
        .filter(key => matchesSearch(key))
        .map(key => ({section, key, label: settingSearchText(key, t)[0] || key}))) : [];

    // Each card shows only what exists on this robot's hardware backend.
    const renderFieldCards = (...groups: SettingsFieldGroup[]) =>
        groups
            .map((group) => groupForBackend(group, hardwareBackend.backend))
            .filter((group): group is SettingsFieldGroup => group !== null)
            .map((group) => (
            <SettingsFieldCard
                key={group.id}
                group={group}
                values={values}
                onChange={handleChange}
                isOverridden={isOverridden}
                hasDefault={hasDefault}
                onReset={resetToDefault}
            />
        ));

    const renderSection = () => {
        switch (activeSection) {
            case "updates":
                return <UpdatesSection configuredModel={values.mower_model ? String(values.mower_model) : undefined}/>;
            case "appearance":
                return (
                    <Space direction="vertical" size={16} style={{width: "100%"}}>
                        <DisplayModeSection />
                        <LogTimeZoneSection />
                    </Space>
                );
            case "hardware":
                return (
                    <HardwareViewSwitcher
                        backend={hardwareBackend.backend}
                        fcu={<FcuFeatureTool />}
                        chassis={
                            <>
                                <HardwareBackendCard info={hardwareBackend} />

                                <HardwareBackendSection backend={hardwareBackend.backend}>
                                    {renderFieldCards(OPENMOWER_WIRING_GROUP)}
                                </HardwareBackendSection>

                                <HardwareSection
                                    values={values}
                                    onChange={handleChange}
                                    onBulkChange={handleBulkChange}
                                    isOverridden={isOverridden}
                                    hasDefault={hasDefault}
                                    onReset={resetToDefault}
                                    revealAdvanced={!!targetField || !!searchQuery}
                                    backendDefaultOverrides={backendDefaultOverrides}
                                />
                            </>
                        }
                    />
                );
            case "drive_motor":
                // Each backend exposes its own drive controls; never show
                // Mowgli STM32 or OpenMower xESC controls on MAVROS.
                if (hardwareBackend.backend === "openmower") {
                    return <>{renderFieldCards(OPENMOWER_WHEEL_LOOP_GROUP)}</>;
                }

                return (
                    <>
                        <DriveMotorSection
                            values={values}
                            onChange={handleChange}
                            acceptPersistedValues={acceptPersistedValues}
                            hardwareBackend={hardwareBackend.backend}
                        />

                        {hardwareBackend.backend === "mowgli" &&
                            renderFieldCards(YAW_LOOP_GROUP)}
                    </>
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
                return (
                    <>
                        <LocalizationSection values={values} onChange={handleChange} />
                        {renderFieldCards(LOCALIZATION_GUARD_GROUP)}
                    </>
                );
            case "mowing":
                return (
                    <>
                        <MowingSection
                            values={values}
                            onChange={handleChange}
                            isOverridden={isOverridden}
                            hasDefault={hasDefault}
                            onReset={resetToDefault}
                            defaults={defaults}
                        />
                        {renderFieldCards(TURN_SPEED_GROUP)}
                    </>
                );
            case "docking":
                return (
                    <>
                        <DockingSection
                            values={values}
                            onChange={handleChange}
                            isOverridden={isOverridden}
                            hasDefault={hasDefault}
                            onReset={resetToDefault}
                        />
                        {renderFieldCards(DOCK_DETECTION_GROUP, DOCK_CALIBRATION_GROUP)}
                    </>
                );
            case "battery":
                return (
                    <>
                        <BatterySection
                            values={values}
                            onChange={handleChange}
                            isOverridden={isOverridden}
                            hasDefault={hasDefault}
                            onReset={resetToDefault}
                        />
                        {renderFieldCards(CHARGE_LIMITS_GROUP)}
                    </>
                );
            case "safety":
                if (hardwareBackend.backend === "mavros") {
                    return <SafetySection values={values} onChange={handleChange} />;
                }

                return (
                    <>
                        <SafetySection values={values} onChange={handleChange} />
                        {renderFieldCards(FIRMWARE_SAFETY_GROUP)}
                        {hardwareBackend.backend === "mowgli" && <FirmwareParamsCard />}
                    </>
                );
            case "obstacles":
                return (
                    <>
                        <ObstaclesSection
                            values={values}
                            onChange={handleChange}
                            isOverridden={isOverridden}
                            hasDefault={hasDefault}
                            defaults={defaults}
                            onReset={resetToDefault}
                        />
                        {renderFieldCards(REVERSE_ESCAPE_GROUP)}
                    </>
                );
            case "navigation":
                return (
                    <>
                        <NavigationSection
                            values={values}
                            onChange={handleChange}
                            isOverridden={isOverridden}
                            hasDefault={hasDefault}
                            onReset={resetToDefault}
                        />
                        {renderFieldCards(AREA_RECORDING_GROUP, BEHAVIOR_TREE_GROUP, START_ESCAPE_GROUP)}
                    </>
                );
            case "weather":
                // IrriSense keeps its own save path (GUI DB, token included);
                // the rain keys go through the yaml like any other section.
                return (
                    <>
                        <RainSection values={values} onChange={handleChange} />
                        <IrriSenseSection
                            registerSaver={registerExternalSaver}
                            unregisterSaver={unregisterExternalSaver}
                        />
                    </>
                );
            case "leds":
                return (
                    <LedsSection
                        values={values}
                        onChange={handleChange}
                        isOverridden={isOverridden}
                        hasDefault={hasDefault}
                        onReset={resetToDefault}
                    />
                );
            case "mqtt":
                return (
                    <MqttSection
                        values={values}
                        onChange={handleChange}
                        isOverridden={isOverridden}
                        hasDefault={hasDefault}
                        onReset={resetToDefault}
                    />
                );
            case "remote_access":
                return (
                    <RemoteAccessSection
                        registerSaver={registerExternalSaver}
                        unregisterSaver={unregisterExternalSaver}
                    />
                );
            case "notifications":
                return (
                    <NotificationsSection
                        registerSaver={registerExternalSaver}
                        unregisterSaver={unregisterExternalSaver}
                    />
                );
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
        <div className="settings-page" style={{ minHeight: isMobile ? "auto" : "calc(100vh - 64px)", display: "flex", flexDirection: "column" }}>
            {/* Header bar */}
            <div style={{
                padding: isMobile ? "12px 12px 0" : "16px 24px 0",
                flexShrink: 0,
            }}>
                {/* Search + save status */}
                <div style={{ display: "flex", alignItems: "center", gap: 12, marginBottom: 12 }}>
                    <Input
                        prefix={<SearchOutlined style={{ color: colors.muted }} />}
                        aria-label={t('settingsPage.searchSettingPlaceholder')}
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

                {fieldResults.length > 0 && (
                    <div role="region" aria-label={t('settingsPage.searchResults')} style={{marginBottom: 12, display: 'flex', flexWrap: 'wrap', gap: 8}}>
                        {fieldResults.map(({section, key, label}) => (
                            <Button key={key} size="small" onClick={() => selectSection(section.id, key)}>
                                {t(section.label)} · {label}
                            </Button>
                        ))}
                    </div>
                )}
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
                    maxHeight: isMobile ? undefined : 'calc(100dvh - 160px)',
                    overflowY: isMobile ? undefined : 'auto',
                    alignSelf: isMobile ? undefined : "flex-start",
                }}>
                    {visibleSections.length > 0 ? (
                        <SettingsNav
                            sections={visibleSections}
                            activeSection={activeSection}
                            onSectionChange={selectSection}
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
                            <h1 ref={sectionHeadingRef} tabIndex={-1} className="mn-display" style={{
                                margin: 0, fontWeight: 400, fontSize: 28, color: colors.text, lineHeight: 1.1, letterSpacing: '-0.01em',
                            }}>
                                {t(currentSectionMeta.label)}
                            </h1>
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
                {!isMobile && ['hardware', 'mowing', 'navigation', 'battery'].includes(activeSection) && (
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
            {(!['updates', 'appearance'].includes(activeSection) || isDirty) && <div style={{
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
                    disabled={!isDirty || ros2Restart.pending}
                >
                    {isDirty ? t("settingsPage.saveWithCount", {count: dirtyCount}) : t("settingsPage.saved")}
                </Button>
                {isDirty && (
                    <Button
                        icon={<UndoOutlined />}
                        onClick={revert}
                    >
                        {t("settingsPage.revert")}
                    </Button>
                )}
            </div>}
        </div>
    );
};

export default SettingsPage;