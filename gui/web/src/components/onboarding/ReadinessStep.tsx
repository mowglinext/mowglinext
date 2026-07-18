import React, {useMemo, useState} from "react";
import {useNavigate} from "react-router-dom";
import {useTranslation} from "react-i18next";
import {App, Alert, Button, Card, List, Result, Space, Tag, Typography} from "antd";
import {
    CheckCircleTwoTone,
    CheckCircleOutlined,
    EnvironmentOutlined,
    InfoCircleTwoTone,
    LoadingOutlined,
    RocketOutlined,
    WarningTwoTone,
} from "@ant-design/icons";
import {useThemeMode} from "../../theme/ThemeContext.tsx";
import {useApi} from "../../hooks/useApi.ts";
import {useGnssStatus} from "../../hooks/useGnssStatus.ts";
import {useCalibrationStatus} from "../../hooks/useCalibrationStatus.ts";
import {useFusionGraphDiagnostics} from "../../hooks/useFusionGraphDiagnostics.ts";
import {useFirmwareStatus} from "../../hooks/useFirmwareStatus.ts";
import {useMowingMap} from "../../hooks/useMowingMap.ts";
import {restartRos2, restartGui} from "../../utils/containers.ts";
import {httpBase} from "../../utils/apiHost.ts";
import {
    computeReadinessChecks,
    requiredFailingChecks,
    type ReadinessCheck,
    type ReadinessCtaTarget,
    type ReadinessState,
    type WorkingAreaLike,
} from "./readinessChecks.ts";
import {STEP_CALIBRATION, STEP_DATUM, STEP_FIRMWARE, STEP_GPS, STEP_NTRIP} from "./steps.ts";

const {Text} = Typography;

interface ReadinessStepProps {
    values: Record<string, unknown>;
    /** Deep-links back to an in-wizard step (wraps the wizard's setCurrentStep). */
    onJumpToStep: (idx: number) => void;
}

/** Maps a check's CTA target to an in-wizard step index (or null for routes). */
const CTA_STEP_INDEX: Record<ReadinessCtaTarget, number | null> = {
    gps: STEP_GPS,
    ntrip: STEP_NTRIP,
    datum: STEP_DATUM,
    firmware: STEP_FIRMWARE,
    calibration: STEP_CALIBRATION,
    diagnostics: null,
    map: null,
};

function stateIcon(check: ReadinessCheck): React.ReactNode {
    if (check.state === "pass") {
        return <CheckCircleTwoTone twoToneColor="#52c41a" />;
    }
    if (check.state === "pending") {
        return <LoadingOutlined spin style={{color: "#1677ff"}} />;
    }
    // fail — required reads as a warning, recommended stays informational.
    return check.required
        ? <WarningTwoTone twoToneColor="#faad14" />
        : <InfoCircleTwoTone twoToneColor="#1677ff" />;
}

export const ReadinessStep: React.FC<ReadinessStepProps> = ({values, onJumpToStep}) => {
    const {t} = useTranslation();
    const {colors} = useThemeMode();
    const guiApi = useApi();
    const navigate = useNavigate();
    const {modal} = App.useApp();

    const gnssStatus = useGnssStatus();
    const {stats: fusion} = useFusionGraphDiagnostics();
    const {status: calibration} = useCalibrationStatus();
    const {firmwareCompatible} = useFirmwareStatus();
    const map = useMowingMap();

    const [committing, setCommitting] = useState(false);
    const [committed, setCommitted] = useState(false);
    const [error, setError] = useState<string | null>(null);

    const checks = useMemo(
        () => computeReadinessChecks({
            gnss: gnssStatus,
            fusion,
            calibration,
            firmwareCompatible,
            values,
            workingArea: (map.working_area ?? undefined) as WorkingAreaLike[] | undefined,
            nowMs: Date.now(),
        }),
        [gnssStatus, fusion, calibration, firmwareCompatible, values, map.working_area],
    );

    const requiredFailing = requiredFailingChecks(checks);
    const gated = requiredFailing.length > 0;

    const runCta = (target: ReadinessCtaTarget) => {
        const stepIndex = CTA_STEP_INDEX[target];
        if (stepIndex !== null) {
            onJumpToStep(stepIndex);
            return;
        }
        navigate(target === "map" ? "/map" : "/diagnostics");
    };

    const commit = async () => {
        setCommitting(true);
        setError(null);
        try {
            // Mark onboarding done in the DB so the wizard doesn't redirect again,
            // then restart ROS2 (picks up the new mowgli_robot.yaml) and the GUI.
            await fetch(`${httpBase()}/api/settings/status`, {method: "POST"});
            await restartRos2(guiApi);
            await restartGui(guiApi);
            setCommitted(true);
        } catch (e) {
            setError(e instanceof Error ? e.message : String(e));
            setCommitted(true);
        } finally {
            setCommitting(false);
        }
    };

    const confirmFinishAnyway = () => {
        const items = requiredFailing.map((c) => t(c.labelKey)).join(", ");
        modal.confirm({
            title: t("onboardingPage.readinessFinishAnyway"),
            content: t("onboardingPage.readinessFinishAnywayConfirm", {items}),
            okText: t("onboardingPage.readinessFinishApply"),
            okButtonProps: {danger: true},
            onOk: commit,
        });
    };

    if (committing) {
        return (
            <Result
                icon={<RocketOutlined style={{color: colors.primary}} spin />}
                title={t("onboardingPage.applyingConfigTitle")}
                subTitle={t("onboardingPage.applyingConfigSubtitle")}
            />
        );
    }

    if (committed) {
        return (
            <Result
                icon={<CheckCircleOutlined style={{color: colors.primary}} />}
                title={t("onboardingPage.allSetTitle")}
                subTitle={t("onboardingPage.allSetSubtitle")}
                extra={[
                    <Button
                        key="map"
                        type="primary"
                        size="large"
                        icon={<EnvironmentOutlined />}
                        onClick={() => navigate("/map")}
                    >
                        {t("onboardingPage.drawMowingArea")}
                    </Button>,
                    <Button key="dashboard" size="large" onClick={() => navigate("/mowglinext")}>
                        {t("onboardingPage.goToDashboard")}
                    </Button>,
                ]}
            >
                {error && (
                    <Alert
                        type="warning"
                        showIcon
                        message={t("onboardingPage.restartFailedTitle")}
                        description={`${error}. ${t("onboardingPage.restartFailedDesc")}`}
                        style={{maxWidth: 500, margin: "0 auto", textAlign: "left"}}
                    />
                )}
            </Result>
        );
    }

    const stateWord = (state: ReadinessState): string =>
        state === "pass"
            ? t("onboardingPage.readinessStatePass")
            : state === "pending"
                ? t("onboardingPage.readinessStatePending")
                : t("onboardingPage.readinessStateFail");

    return (
        <div style={{maxWidth: 760, margin: "0 auto"}}>
            <Typography.Title level={4}>
                <CheckCircleOutlined /> {t("onboardingPage.readinessTitle")}
            </Typography.Title>
            <Typography.Paragraph type="secondary">
                {t("onboardingPage.readinessIntro")}
            </Typography.Paragraph>

            <Card size="small" style={{marginBottom: 16}}>
                <List
                    itemLayout="horizontal"
                    dataSource={checks}
                    renderItem={(check) => {
                        const showCta = check.state !== "pass" && check.ctaKey && check.ctaTarget;
                        return (
                            <List.Item
                                actions={showCta ? [
                                    <Button
                                        key="cta"
                                        type="link"
                                        size="small"
                                        onClick={() => runCta(check.ctaTarget!)}
                                    >
                                        {t(check.ctaKey!)}
                                    </Button>,
                                ] : undefined}
                            >
                                <List.Item.Meta
                                    avatar={<span style={{fontSize: 18}}>{stateIcon(check)}</span>}
                                    title={
                                        <Space size={8} wrap>
                                            <Text>{t(check.labelKey)}</Text>
                                            {!check.required && (
                                                <Tag>{t("onboardingPage.readinessOptional")}</Tag>
                                            )}
                                        </Space>
                                    }
                                    description={
                                        <Text type="secondary" style={{fontSize: 12}}>
                                            {stateWord(check.state)}
                                            {check.valueText ? ` · ${check.valueText}` : ""}
                                        </Text>
                                    }
                                />
                            </List.Item>
                        );
                    }}
                />
            </Card>

            {gated && (
                <Alert
                    type="warning"
                    showIcon
                    message={t("onboardingPage.readinessGatedTitle")}
                    description={t("onboardingPage.readinessGatedDesc", {count: requiredFailing.length})}
                    style={{marginBottom: 16, textAlign: "left"}}
                />
            )}

            <div style={{textAlign: "center"}}>
                <Space direction="vertical" size={8}>
                    <Button
                        type="primary"
                        size="large"
                        icon={<CheckCircleOutlined />}
                        disabled={gated}
                        onClick={commit}
                    >
                        {t("onboardingPage.readinessFinishApply")}
                    </Button>
                    {gated && (
                        <Button type="link" onClick={confirmFinishAnyway}>
                            {t("onboardingPage.readinessFinishAnyway")}
                        </Button>
                    )}
                </Space>
            </div>
        </div>
    );
};
