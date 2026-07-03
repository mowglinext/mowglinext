import {Alert, App, Button, Col, Collapse, Row, Typography} from "antd";
import {useEffect, useMemo, useRef, useState} from "react";
import {fetchEventSource} from "@microsoft/fetch-event-source";
import {createSchemaField} from "@formily/react";
import {
    Checkbox,
    Form,
    FormButtonGroup,
    FormItem,
    FormLayout,
    Input,
    NumberPicker,
    Select,
} from "@formily/antd-v5";
import {StyledTerminal} from "./StyledTerminal.tsx";
import Terminal, {ColorMode, TerminalOutput} from "react-terminal-ui";
import {useTranslation} from "react-i18next";
import {createForm, onFieldValueChange} from "@formily/core";
import {useApi} from "../hooks/useApi.ts";
import {useIsMobile} from "../hooks/useIsMobile";
import {useThemeMode} from "../theme/ThemeContext.tsx";

const SchemaField = createSchemaField({
    components: {
        Input,
        FormItem,
        Select,
        Checkbox,
        NumberPicker,
    },
})
type Config = {
    repository: string
    branch: string
    directory: string
    file: string
    version: string,
    boardType: string,
    panelType: string,
    debugType: string,
    disableEmergency: boolean,
    maxMps: number,
    maxChargeCurrent: number,
    limitVoltage150MA: number,
    maxChargeVoltage: number,
    batChargeCutoffVoltage: number,
    oneWheelLiftEmergencyMillis: number,
    bothWheelsLiftEmergencyMillis: number,
    tiltEmergencyMillis: number,
    stopButtonEmergencyMillis: number,
    playButtonClearEmergencyMillis: number,
    imuOnboardInclinationThreshold: number,
    externalImuAcceleration: boolean,
    externalImuAngular: boolean,
    masterJ18: boolean,
    tickPerM: number,
    wheelBase: number
    perimeterWire: boolean
}

export const FlashBoardComponent = (props: { onNext: () => void }) => {
    const isMobile = useIsMobile();
    const {colors} = useThemeMode();
    const {t} = useTranslation();
    const form = useMemo(() => createForm({
        validateFirst: true,
        effects: (form) => {
            onFieldValueChange('boardType', (field) => {
                form.setFieldState('*(panelType,tickPerM,wheelBase,directory,branch,repository,debugType,disableEmergency,maxMps,maxChargeCurrent,limitVoltage150MA,maxChargeVoltage,batChargeCutoffVoltage,oneWheelLiftEmergencyMillis,bothWheelsLiftEmergencyMillis,tiltEmergencyMillis,stopButtonEmergencyMillis,playButtonClearEmergencyMillis,imuOnboardInclinationThreshold,externalImuAcceleration,externalImuAngular,masterJ18,perimeterWire)', (state) => {
                    state.display = field.value !== "BOARD_VERMUT_YARDFORCE500" ? "visible" : "hidden";
                })
                form.setFieldState('*(version,file)', (state) => {
                    state.display = field.value === "BOARD_VERMUT_YARDFORCE500" ? "visible" : "hidden";
                })
            })
        },
    }), [])
    const guiApi = useApi();
    const {notification, modal} = App.useApp();
    const [data, setData] = useState<string[]>()
    const abortControllerRef = useRef<AbortController | null>(null);
    const [isFlashing, setIsFlashing] = useState(false);
    const [flashDone, setFlashDone] = useState(false);
    const [flashError, setFlashError] = useState<string | null>(null);
    const terminalRef = useRef<HTMLDivElement>(null);

    useEffect(() => {
        (async () => {
            try {
                const config = await guiApi.config.keysGetCreate({
                    "gui.firmware.config": ""
                })
                const jsonConfig = config.data["gui.firmware.config"]
                if (jsonConfig) {
                    form.setInitialValues(JSON.parse(jsonConfig))
                }
            } catch (e: any) {
                notification.error({
                    message: t('flashBoard.errorRetrievingConfig'),
                    description: e.toString(),
                });
            }
        })()
        return () => {
            if (abortControllerRef.current) {
                abortControllerRef.current.abort();
            }
        };
    }, []);

    // Auto-scroll terminal to bottom
    useEffect(() => {
        if (terminalRef.current) {
            terminalRef.current.scrollTop = terminalRef.current.scrollHeight;
        }
    }, [data]);

    const doFlashFirmware = async (values: Config) => {
        if (isFlashing) return;
        if (abortControllerRef.current) {
            abortControllerRef.current.abort();
        }
        const controller = new AbortController();
        abortControllerRef.current = controller;
        setIsFlashing(true);
        setFlashDone(false);
        setFlashError(null);
        setData([]);
        try {
            await fetchEventSource(`/api/setup/flashBoard`, {
                method: "POST",
                keepalive: false,
                // Keep the stream open when the tab is backgrounded / loses
                // focus. Without this, fetch-event-source closes the connection
                // on `visibilitychange` (hidden) and RE-OPENS it on return —
                // and "re-open" re-sends this POST, which makes the backend
                // start a whole new flash (git clone + platformio build) from
                // scratch instead of continuing the one in progress.
                openWhenHidden: true,
                body: JSON.stringify(values),
                headers: {
                    Accept: "text/event-stream",
                },
                signal: controller.signal,
                onopen(res) {
                    if (res.status >= 400 && res.status < 500 && res.status !== 429) {
                        notification.error({
                            message: t('flashBoard.errorConnectingFlashEndpoint'),
                            description: res.statusText,
                        });
                    }
                    return Promise.resolve()
                },
                onmessage(event) {
                    if (event.event == "end") {
                        setIsFlashing(false);
                        setFlashDone(true);
                        return;
                    } else if (event.event == "error") {
                        setIsFlashing(false);
                        setFlashError(event.data);
                        return;
                    } else {
                        setData((data) => [...(data ?? []), event.data]);
                    }
                },
                onclose() {},
                onerror(err) {
                    setIsFlashing(false);
                    setFlashError(err.toString());
                    // Re-throw to stop fetch-event-source's automatic retry.
                    // Otherwise the library re-POSTs on any stream error and
                    // relaunches the flash; we surface the error in the UI and
                    // let the user retry deliberately instead.
                    throw err;
                },
            });
        } catch (e: any) {
            if (e.name !== 'AbortError') {
                setIsFlashing(false);
                setFlashError(e.toString());
            }
        }
    };

    const flashFirmware = (values: Config) => {
        const confirmModal = modal.confirm({
            title: t('flashBoard.confirmFlashTitle'),
            content: (
                <div>
                    <p><strong>{t('flashBoard.confirmVerifyParams')}</strong></p>
                    <ul style={{listStyle: "none", padding: 0}}>
                        <li>{t('flashBoard.maxChargeCurrentLabel')}: <strong>{values.maxChargeCurrent} A</strong></li>
                        <li>{t('flashBoard.maxChargeVoltageLabel')}: <strong>{values.maxChargeVoltage} V</strong></li>
                        <li>{t('flashBoard.batChargeCutoffVoltageLabel')}: <strong>{values.batChargeCutoffVoltage} V</strong></li>
                        <li>{t('flashBoard.limitVoltage150MALabel')}: <strong>{values.limitVoltage150MA} V</strong></li>
                        <li>{t('flashBoard.imuInclinationThresholdLabel')}: <strong>0x{(values.imuOnboardInclinationThreshold ?? 0x38).toString(16).toUpperCase().padStart(2, "0")}</strong></li>
                    </ul>
                    <p style={{color: colors.danger}}><strong>{t('flashBoard.confirmWrongValuesWarning')}</strong></p>
                </div>
            ),
            okText: t('flashBoard.flash'),
            okType: "danger",
            cancelText: t('flashBoard.cancel'),
            onOk: () => {
                confirmModal.destroy();
                doFlashFirmware(values);
            },
        });
    };

    // Show flashing progress view
    if (data !== undefined && data.length > 0 || isFlashing || flashDone || flashError) {
        return (
            <Row gutter={[0, 16]}>
                <Col span={24}>
                    <Typography.Title level={5} style={{margin: 0}}>
                        {isFlashing ? t('flashBoard.flashingFirmware') : flashError ? t('flashBoard.flashFailed') : t('flashBoard.flashComplete')}
                    </Typography.Title>
                </Col>
                <Col span={24}>
                    <div ref={terminalRef} style={{height: isMobile ? "30vh" : "35vh", overflowY: "auto"}}>
                        <StyledTerminal>
                            <Terminal colorMode={ColorMode.Dark}>
                                {(data ?? []).map((line, index) => (
                                    <TerminalOutput key={index}>{line}</TerminalOutput>
                                ))}
                                {flashDone && (
                                    <TerminalOutput>
                                        {`\n✅ ${t('flashBoard.flashedSuccessfully')}`}
                                    </TerminalOutput>
                                )}
                                {flashError && (
                                    <TerminalOutput>
                                        {`\n❌ ${t('flashBoard.errorPrefix')}: ${flashError}`}
                                    </TerminalOutput>
                                )}
                            </Terminal>
                        </StyledTerminal>
                    </div>
                </Col>
                <Col span={24} style={{
                    position: "fixed",
                    bottom: isMobile ? 'calc(56px + env(safe-area-inset-bottom, 0px))' : 20,
                    left: isMobile ? 0 : undefined,
                    right: isMobile ? 0 : undefined,
                    padding: isMobile ? '8px 12px' : undefined,
                    background: isMobile ? colors.bgCard : undefined,
                    borderTop: isMobile ? `1px solid ${colors.border}` : undefined,
                    zIndex: 50,
                }}>
                    <FormButtonGroup>
                        {flashError && (
                            <Button onClick={() => {
                                setData(undefined);
                                setFlashError(null);
                            }}>{t('flashBoard.backToConfig')}</Button>
                        )}
                        <Button
                            type="primary"
                            disabled={isFlashing}
                            onClick={props.onNext}
                        >
                            {isFlashing ? t('flashBoard.flashingShort') : t('flashBoard.next')}
                        </Button>
                    </FormButtonGroup>
                </Col>
            </Row>
        );
    }

    // Show config form
    return <Form form={form}>
        <Row>
            <Col span={24} style={{height: isMobile ? "auto" : "55vh", overflowY: isMobile ? undefined : "auto", paddingBottom: isMobile ? 80 : undefined}}>
                <FormLayout layout="vertical">
                    <SchemaField><SchemaField.String
                        name={"boardType"}
                        title={t('flashBoard.boardSelectionTitle')}
                        default={"BOARD_VERMUT_YARDFORCE500"}
                        enum={[{
                            label: "Vermut - YardForce 500 Classic",
                            value: "BOARD_VERMUT_YARDFORCE500"
                        }, {
                            label: "Mowgli - YardForce 500 Classic",
                            value: "BOARD_YARDFORCE500"
                        }, {
                            label: "Mowgli - YardForce 500 B Variant",
                            value: "BOARD_YARDFORCE500B"
                        },
                            {
                                label: "Mowgli - LUV1000RI",
                                value: "BOARD_LUV1000RI"
                            }
                        ]} x-component="Select"
                        x-decorator="FormItem"/></SchemaField>
                    <SchemaField><SchemaField.String
                        name={"version"}
                        title={t('flashBoard.versionTitle')}
                        default={"0_13_X"}
                        enum={[{
                            label: "V0.13",
                            value: "0_13_X"
                        }, {
                            label: "V0.12 (LSM6DSO)",
                            value: "0_12_X_LSM6DSO"
                        },
                            {
                                label: "V0.12",
                                value: "0_12_X"
                            },
                            {
                                label: "V0.11 (MPU9250)",
                                value: "0_11_X_MPU9250"
                            },
                            {
                                label: "V0.11 (WT901)",
                                value: "0_11_X_WT901"
                            },
                            {
                                label: "V0.10 (MPU9250)",
                                value: "0_10_X_MPU9250"
                            },
                            {
                                label: "V0.10 (WT901)",
                                value: "0_10_X_WT901"
                            },
                            {
                                label: "V0.9 (MPU9250)",
                                value: "0_9_X_MPU9250"
                            },
                            {
                                label: "V0.9 (WT901 instead of sound)",
                                value: "0_9_X_WT901_INSTEAD_OF_SOUND"
                            }
                        ]} x-component="Select"
                        x-decorator="FormItem"/></SchemaField>
                    <SchemaField><SchemaField.String
                        name={"file"}
                        title={t('flashBoard.archiveTitle')}
                        default={"https://github.com/ClemensElflein/MowgliNext/releases/download/latest/firmware.zip"}
                        x-decorator-props={{tooltip: t('flashBoard.archiveTooltip')}}
                        x-component="Input"
                        x-decorator="FormItem"/></SchemaField>
                    <SchemaField><SchemaField.String
                        name={"repository"}
                        title={t('flashBoard.repositoryTitle')}
                        default={"https://github.com/mowglinext/mowglinext"}
                        x-decorator-props={{tooltip: t('flashBoard.repositoryTooltip')}}
                        x-component="Input"
                        x-decorator="FormItem"/></SchemaField>
                    <SchemaField><SchemaField.String
                        name={"branch"}
                        title={t('flashBoard.branchTitle')}
                        default={"main"}
                        x-decorator-props={{tooltip: t('flashBoard.branchTooltip')}}
                        x-component="Input"
                        x-decorator="FormItem"/></SchemaField>
                    <SchemaField><SchemaField.String
                        name={"directory"}
                        title={t('flashBoard.firmwareDirectoryTitle')}
                        default={"firmware"}
                        x-decorator-props={{tooltip: t('flashBoard.firmwareDirectoryTooltip')}}
                        x-component="Input"
                        x-decorator="FormItem"/></SchemaField>
                    <SchemaField><SchemaField.String
                        name={"panelType"}
                        title={t('flashBoard.panelSelectionTitle')}
                        default={"PANEL_TYPE_YARDFORCE_500_CLASSIC"}
                        enum={[
                            {label: "YardForce 500 Classic", value: "PANEL_TYPE_YARDFORCE_500_CLASSIC"},
                            {label: "YardForce LUV1000RI", value: "PANEL_TYPE_YARDFORCE_LUV1000RI"},
                            {label: "YardForce 500B Classic", value: "PANEL_TYPE_YARDFORCE_500B_CLASSIC"},
                            {label: "YardForce 900 ECO", value: "PANEL_TYPE_YARDFORCE_900_ECO"},
                        ]} x-component="Select"
                        x-decorator="FormItem"/></SchemaField>
                    <SchemaField><SchemaField.String
                        name={"debugType"}
                        title={t('flashBoard.debugTypeTitle')}
                        default={"DEBUG_TYPE_UART"}
                        enum={[
                            {label: t('flashBoard.debugTypeNone'), value: "DEBUG_TYPE_NONE"},
                            {label: "Uart", value: "DEBUG_TYPE_UART"},
                            {label: "Swo", value: "DEBUG_TYPE_SWO"},
                        ]} x-component="Select"
                        x-decorator="FormItem"/></SchemaField>
                    <SchemaField><SchemaField.Number name={"maxMps"} title={t('flashBoard.maxMpsTitle')} default={0.5}
                        x-decorator-props={{tooltip: t('flashBoard.maxMpsTooltip')}}
                        x-component-props={{step: 0.1, max: 1.0}}
                        x-component="NumberPicker" x-decorator="FormItem"/></SchemaField>
                    <SchemaField><SchemaField.Number name={"tickPerM"} title={t('flashBoard.tickPerMTitle')} default={300.0}
                        x-decorator-props={{tooltip: t('flashBoard.tickPerMTooltip')}}
                        x-component-props={{step: 0.1}}
                        x-component="NumberPicker" x-decorator="FormItem"/></SchemaField>
                    <SchemaField><SchemaField.Number name={"wheelBase"} title={t('flashBoard.wheelBaseTitle')} default={0.325}
                        x-decorator-props={{tooltip: t('flashBoard.wheelBaseTooltip')}}
                        x-component-props={{step: 0.001}}
                        x-component="NumberPicker" x-decorator="FormItem"/></SchemaField>

                    {/* Progressive disclosure: the charge/voltage/current,
                        emergency-timer and IMU-threshold fields are hidden behind
                        an explicit fold. Wrong values here can damage the battery
                        or disable physical safety, so they are NOT shown to a
                        first-time flasher by default. The Formily field-state
                        wiring (boardType effect, save/restore) is unaffected —
                        these are the same fields, just rendered inside a Collapse. */}
                    <Collapse
                        ghost
                        style={{marginBottom: 8}}
                        items={[{
                            key: "advanced",
                            // forceRender keeps the Formily fields mounted even when
                            // the panel is collapsed, so their `default` values are
                            // registered in the form model and ALWAYS submitted with
                            // the flash payload — never silently undefined.
                            forceRender: true,
                            label: (
                                <Typography.Text strong style={{color: colors.warning}}>
                                    {t('flashBoard.advancedParamsLabel')}
                                </Typography.Text>
                            ),
                            children: (
                                <FormLayout layout="vertical">
                                    <Alert
                                        type="warning"
                                        showIcon
                                        style={{marginBottom: 12}}
                                        message={t('flashBoard.advancedAlertMessage')}
                                        description={t('flashBoard.advancedAlertDescription')}
                                    />
                                    <div style={{
                                        border: `1px solid ${colors.danger}`,
                                        background: colors.dangerBg,
                                        borderRadius: 8,
                                        padding: "10px 12px",
                                        marginBottom: 12,
                                    }}>
                                        <SchemaField><SchemaField.Boolean name={"disableEmergency"} title={t('flashBoard.disableEmergencyTitle')} default={false}
                                            x-decorator-props={{tooltip: t('flashBoard.disableEmergencyTooltip'), style: {marginBottom: 0}}}
                                            x-component="Checkbox"
                                            x-component-props={{style: {color: colors.danger, fontWeight: 600}}}
                                            x-decorator="FormItem"/></SchemaField>
                                        <Typography.Text style={{color: colors.danger, fontSize: 12}}>
                                            {t('flashBoard.disableEmergencyWarning')}
                                        </Typography.Text>
                                    </div>
                                    <SchemaField><SchemaField.Number name={"maxChargeCurrent"} title={t('flashBoard.maxChargeCurrentTitle')} default={1.0}
                                        x-component-props={{step: 0.1, max: 5.0}}
                                        x-decorator-props={{tooltip: t('flashBoard.maxChargeCurrentTooltip')}}
                                        x-component="NumberPicker" x-decorator="FormItem"/></SchemaField>
                                    <SchemaField><SchemaField.Number name={"limitVoltage150MA"} title={t('flashBoard.limitVoltage150MATitle')} default={28.0}
                                        x-decorator-props={{tooltip: t('flashBoard.limitVoltage150MATooltip')}}
                                        x-component-props={{step: 0.1, max: 30.0}}
                                        x-component="NumberPicker" x-decorator="FormItem"/></SchemaField>
                                    <SchemaField><SchemaField.Number name={"maxChargeVoltage"} title={t('flashBoard.maxChargeVoltageTitle')} default={29.0}
                                        x-decorator-props={{tooltip: t('flashBoard.maxChargeVoltageTooltip')}}
                                        x-component-props={{step: 0.1, max: 30.0}}
                                        x-component="NumberPicker" x-decorator="FormItem"/></SchemaField>
                                    <SchemaField><SchemaField.Number name={"batChargeCutoffVoltage"} title={t('flashBoard.batChargeCutoffVoltageTitle')} default={28.0}
                                        x-decorator-props={{tooltip: t('flashBoard.batChargeCutoffVoltageTooltip')}}
                                        x-component-props={{step: 0.1, max: 30.0}}
                                        x-component="NumberPicker" x-decorator="FormItem"/></SchemaField>
                                    <SchemaField><SchemaField.Number name={"oneWheelLiftEmergencyMillis"} title={t('flashBoard.oneWheelLiftEmergencyMillisTitle')} default={10000}
                                        x-decorator-props={{tooltip: t('flashBoard.oneWheelLiftEmergencyMillisTooltip')}}
                                        x-component-props={{step: 1}}
                                        x-component="NumberPicker" x-decorator="FormItem"/></SchemaField>
                                    <SchemaField><SchemaField.Number name={"bothWheelsLiftEmergencyMillis"} title={t('flashBoard.bothWheelsLiftEmergencyMillisTitle')} default={1000}
                                        x-decorator-props={{tooltip: t('flashBoard.bothWheelsLiftEmergencyMillisTooltip')}}
                                        x-component-props={{step: 1}}
                                        x-component="NumberPicker" x-decorator="FormItem"/></SchemaField>
                                    <SchemaField><SchemaField.Number name={"tiltEmergencyMillis"} title={t('flashBoard.tiltEmergencyMillisTitle')} default={500}
                                        x-decorator-props={{tooltip: t('flashBoard.tiltEmergencyMillisTooltip')}}
                                        x-component-props={{step: 1}}
                                        x-component="NumberPicker" x-decorator="FormItem"/></SchemaField>
                                    <SchemaField><SchemaField.Number name={"stopButtonEmergencyMillis"} title={t('flashBoard.stopButtonEmergencyMillisTitle')} default={100}
                                        x-decorator-props={{tooltip: t('flashBoard.stopButtonEmergencyMillisTooltip')}}
                                        x-component-props={{step: 1}}
                                        x-component="NumberPicker" x-decorator="FormItem"/></SchemaField>
                                    <SchemaField><SchemaField.Number name={"playButtonClearEmergencyMillis"} title={t('flashBoard.playButtonClearEmergencyMillisTitle')} default={2000}
                                        x-decorator-props={{tooltip: t('flashBoard.playButtonClearEmergencyMillisTooltip')}}
                                        x-component-props={{step: 1}}
                                        x-component="NumberPicker" x-decorator="FormItem"/></SchemaField>
                                    <SchemaField><SchemaField.Number name={"imuOnboardInclinationThreshold"} title={t('flashBoard.imuOnboardInclinationThresholdTitle')} default={0x38}
                                        x-decorator-props={{tooltip: t('flashBoard.imuOnboardInclinationThresholdTooltip')}}
                                        x-component-props={{step: 1, min: 0, max: 127}}
                                        x-component="NumberPicker" x-decorator="FormItem"/></SchemaField>
                                    <SchemaField><SchemaField.Boolean name={"externalImuAcceleration"} title={t('flashBoard.externalImuAccelerationTitle')} default={true}
                                        x-decorator-props={{tooltip: t('flashBoard.externalImuAccelerationTooltip')}}
                                        x-component="Checkbox" x-decorator="FormItem"/></SchemaField>
                                    <SchemaField><SchemaField.Boolean name={"externalImuAngular"} title={t('flashBoard.externalImuAngularTitle')} default={true}
                                        x-decorator-props={{tooltip: t('flashBoard.externalImuAngularTooltip')}}
                                        x-component="Checkbox" x-decorator="FormItem"/></SchemaField>
                                    <SchemaField><SchemaField.Boolean name={"masterJ18"} title={t('flashBoard.masterJ18Title')} default={true}
                                        x-decorator-props={{tooltip: t('flashBoard.masterJ18Tooltip')}}
                                        x-component="Checkbox" x-decorator="FormItem"/></SchemaField>
                                    <SchemaField><SchemaField.Boolean name={"perimeterWire"} title={t('flashBoard.perimeterWireTitle')} default={true}
                                        x-decorator-props={{tooltip: t('flashBoard.perimeterWireTooltip')}}
                                        x-component="Checkbox" x-decorator="FormItem"/></SchemaField>
                                </FormLayout>
                            ),
                        }]}
                    />
                </FormLayout>
            </Col>
            <Col span={24} style={{
                position: "fixed",
                bottom: isMobile ? 'calc(56px + env(safe-area-inset-bottom, 0px))' : 20,
                left: isMobile ? 0 : undefined,
                right: isMobile ? 0 : undefined,
                padding: isMobile ? '8px 12px' : undefined,
                background: isMobile ? colors.bgCard : undefined,
                borderTop: isMobile ? `1px solid ${colors.border}` : undefined,
                zIndex: 50,
            }}>
                <FormButtonGroup>
                    <Button type="primary" onClick={() => {
                        form.submit(flashFirmware).catch((err: unknown) => {
                            if (err instanceof Error) {
                                notification.error({
                                    message: t('flashBoard.validationFailed'),
                                    description: err.message,
                                });
                            }
                        });
                    }}>{t('flashBoard.flashFirmware')}</Button>
                    <Button onClick={props.onNext}>{t('flashBoard.skip')}</Button>
                </FormButtonGroup>
            </Col>
        </Row>
    </Form>;
};
