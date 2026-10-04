import {Button, Col, Collapse, Progress, Row, Steps, Typography} from "antd";
import {useEffect, useRef, useState} from "react";
import {FormButtonGroup} from "@formily/antd-v5";
import Terminal, {ColorMode, TerminalOutput} from "react-terminal-ui";
import {useTranslation} from "react-i18next";
import {StyledTerminal} from "./StyledTerminal.tsx";
import {useIsMobile} from "../hooks/useIsMobile";
import {useThemeMode} from "../theme/ThemeContext.tsx";
import {flashPercent, type FlashStage} from "./flashStage.ts";

export interface FlashProgressViewProps {
    stage: FlashStage | null;
    log: string[];
    isFlashing: boolean;
    flashDone: boolean;
    flashError: string | null;
    onBackToConfig: () => void;
    onNext: () => void;
}

/**
 * Progress screen of a firmware flash: a step list of the plan the backend
 * announced, a percentage bar, and the raw openocd/platformio log folded away
 * (opened automatically on failure, where the log is the diagnosis).
 */
export const FlashProgressView = ({
    stage,
    log,
    isFlashing,
    flashDone,
    flashError,
    onBackToConfig,
    onNext,
}: FlashProgressViewProps) => {
    const {t} = useTranslation();
    const isMobile = useIsMobile();
    const {colors} = useThemeMode();
    const terminalRef = useRef<HTMLDivElement>(null);
    // The log stays folded while things go well; a failure unfolds it, since
    // the openocd/platformio output is then the diagnosis. An explicit toggle
    // by the operator wins over that default.
    const [logToggle, setLogToggle] = useState<boolean | null>(null);
    const isLogOpen = logToggle ?? !!flashError;

    useEffect(() => {
        if (terminalRef.current) {
            terminalRef.current.scrollTop = terminalRef.current.scrollHeight;
        }
    }, [log, isLogOpen]);

    const stageTitle = (key: string) => t(`flashBoard.stages.${key}`, {defaultValue: key});
    const currentKey = stage?.stages[stage.current];
    const percent = flashPercent(stage, flashDone);
    const title = isFlashing
        ? t("flashBoard.flashingFirmware")
        : flashError ? t("flashBoard.flashFailed") : t("flashBoard.flashComplete");
    const barStatus = flashError ? "exception" : flashDone ? "success" : "active";
    const stepsStatus = flashError ? "error" : flashDone ? "finish" : "process";
    const statusLine = flashError
        ? `${t("flashBoard.errorPrefix")}: ${flashError}`
        : flashDone
            ? t("flashBoard.flashedSuccessfully")
            : currentKey
                ? t(`flashBoard.stageHints.${currentKey}`, {defaultValue: stageTitle(currentKey)})
                : t("flashBoard.stageStarting");

    return (
        <Row gutter={[0, 16]}>
            <Col span={24}>
                <Typography.Title level={5} style={{margin: 0}}>{title}</Typography.Title>
            </Col>
            <Col span={24}>
                <Progress percent={percent} status={barStatus} />
                <Typography.Text
                    type={flashError ? "danger" : "secondary"}
                    data-testid="flash-status-line"
                >
                    {statusLine}
                </Typography.Text>
            </Col>
            {stage && (
                <Col span={24}>
                    <Steps
                        size="small"
                        direction={isMobile ? "vertical" : "horizontal"}
                        current={flashDone ? stage.stages.length : stage.current}
                        status={stepsStatus}
                        items={stage.stages.map((key) => ({title: stageTitle(key)}))}
                    />
                </Col>
            )}
            <Col span={24} style={{paddingBottom: isMobile ? 80 : 60}}>
                <Collapse
                    ghost
                    activeKey={isLogOpen ? ["log"] : []}
                    onChange={(keys) => setLogToggle(keys.length > 0)}
                    items={[{
                        key: "log",
                        label: t("flashBoard.showLog"),
                        children: (
                            <div ref={terminalRef} style={{height: isMobile ? "30vh" : "35vh", overflowY: "auto"}}>
                                <StyledTerminal>
                                    <Terminal colorMode={ColorMode.Dark}>
                                        {log.map((line, index) => (
                                            <TerminalOutput key={index}>{line}</TerminalOutput>
                                        ))}
                                    </Terminal>
                                </StyledTerminal>
                            </div>
                        ),
                    }]}
                />
            </Col>
            <Col span={24} style={{
                position: "fixed",
                bottom: isMobile ? "calc(56px + env(safe-area-inset-bottom, 0px))" : 20,
                left: isMobile ? 0 : undefined,
                right: isMobile ? 0 : undefined,
                padding: isMobile ? "8px 12px" : undefined,
                background: isMobile ? colors.bgCard : undefined,
                borderTop: isMobile ? `1px solid ${colors.border}` : undefined,
                zIndex: 50,
            }}>
                <FormButtonGroup>
                    {flashError && (
                        <Button onClick={onBackToConfig}>{t("flashBoard.backToConfig")}</Button>
                    )}
                    <Button type="primary" disabled={isFlashing} onClick={onNext}>
                        {isFlashing ? t("flashBoard.flashingShort") : t("flashBoard.next")}
                    </Button>
                </FormButtonGroup>
            </Col>
        </Row>
    );
};
