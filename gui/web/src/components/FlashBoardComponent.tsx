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
                    message: "Error retrieving config",
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
                            message: "Error connecting to flash endpoint",
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
            title: "Confirm firmware flash",
            content: (
                <div>
                    <p><strong>Please verify the following parameters before flashing:</strong></p>
                    <ul style={{listStyle: "none", padding: 0}}>
                        <li>Max Charge Current: <strong>{values.maxChargeCurrent} A</strong></li>
                        <li>Max Charge Voltage: <strong>{values.maxChargeVoltage} V</strong></li>
                        <li>Bat Charge Cutoff Voltage: <strong>{values.batChargeCutoffVoltage} V</strong></li>
                        <li>Limit Voltage 150mA: <strong>{values.limitVoltage150MA} V</strong></li>
                        <li>IMU Inclination Threshold: <strong>0x{(values.imuOnboardInclinationThreshold ?? 0x38).toString(16).toUpperCase().padStart(2, "0")}</strong></li>
                    </ul>
                    <p style={{color: colors.danger}}><strong>Wrong voltage or current values can damage your battery or hardware!</strong></p>
                </div>
            ),
            okText: "Flash",
            okType: "danger",
            cancelText: "Cancel",
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
                        {isFlashing ? "Flashing firmware..." : flashError ? "Flash failed" : "Flash complete"}
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
                                        {"\n✅ Firmware flashed successfully!"}
                                    </TerminalOutput>
                                )}
                                {flashError && (
                                    <TerminalOutput>
                                        {`\n❌ Error: ${flashError}`}
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
                            }}>Back to config</Button>
                        )}
                        <Button
                            type="primary"
                            disabled={isFlashing}
                            onClick={props.onNext}
                        >
                            {isFlashing ? "Flashing..." : "Next"}
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
                        title={"Board Selection"}
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
                        title={"Version"}
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
                        title={"Archive"}
                        default={"https://github.com/ClemensElflein/MowgliNext/releases/download/latest/firmware.zip"}
                        x-decorator-props={{tooltip: "Archive to use for firmware"}}
                        x-component="Input"
                        x-decorator="FormItem"/></SchemaField>
                    <SchemaField><SchemaField.String
                        name={"repository"}
                        title={"Repository"}
                        default={"https://github.com/cedbossneo/mowglinext"}
                        x-decorator-props={{tooltip: "Repository to use for firmware"}}
                        x-component="Input"
                        x-decorator="FormItem"/></SchemaField>
                    <SchemaField><SchemaField.String
                        name={"branch"}
                        title={"Branch"}
                        default={"main"}
                        x-decorator-props={{tooltip: "Branch to use for firmware"}}
                        x-component="Input"
                        x-decorator="FormItem"/></SchemaField>
                    <SchemaField><SchemaField.String
                        name={"directory"}
                        title={"Firmware Directory"}
                        default={"firmware"}
                        x-decorator-props={{tooltip: "Path to the firmware directory inside the cloned repository (containing stm32/ros_usbnode)"}}
                        x-component="Input"
                        x-decorator="FormItem"/></SchemaField>
                    <SchemaField><SchemaField.String
                        name={"panelType"}
                        title={"Panel Selection"}
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
                        title={"Debug Type"}
                        default={"DEBUG_TYPE_UART"}
                        enum={[
                            {label: "None", value: "DEBUG_TYPE_NONE"},
                            {label: "Uart", value: "DEBUG_TYPE_UART"},
                            {label: "Swo", value: "DEBUG_TYPE_SWO"},
                        ]} x-component="Select"
                        x-decorator="FormItem"/></SchemaField>
                    <SchemaField><SchemaField.Number name={"maxMps"} title={"Max MPS"} default={0.5}
                        x-decorator-props={{tooltip: "Max speed in meters per second"}}
                        x-component-props={{step: 0.1, max: 1.0}}
                        x-component="NumberPicker" x-decorator="FormItem"/></SchemaField>
                    <SchemaField><SchemaField.Number name={"tickPerM"} title={"Tick per meter"} default={300.0}
                        x-decorator-props={{tooltip: "Number of wheel ticks per meter"}}
                        x-component-props={{step: 0.1}}
                        x-component="NumberPicker" x-decorator="FormItem"/></SchemaField>
                    <SchemaField><SchemaField.Number name={"wheelBase"} title={"Wheel base"} default={0.325}
                        x-decorator-props={{tooltip: "Wheel base in meters"}}
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
                                    Paramètres firmware avancés (peuvent endommager le matériel)
                                </Typography.Text>
                            ),
                            children: (
                                <FormLayout layout="vertical">
                                    <Alert
                                        type="warning"
                                        showIcon
                                        style={{marginBottom: 12}}
                                        message="Ces réglages contrôlent la charge de la batterie et les sécurités physiques"
                                        description="Des valeurs de tension ou de courant incorrectes peuvent endommager la batterie ou la carte. Ne modifiez ces champs que si vous savez précisément ce que vous faites."
                                    />
                                    <div style={{
                                        border: `1px solid ${colors.danger}`,
                                        background: colors.dangerBg,
                                        borderRadius: 8,
                                        padding: "10px 12px",
                                        marginBottom: 12,
                                    }}>
                                        <SchemaField><SchemaField.Boolean name={"disableEmergency"} title={"Disable Emergency"} default={false}
                                            x-decorator-props={{tooltip: "Disable emergency stop", style: {marginBottom: 0}}}
                                            x-component="Checkbox"
                                            x-component-props={{style: {color: colors.danger, fontWeight: 600}}}
                                            x-decorator="FormItem"/></SchemaField>
                                        <Typography.Text style={{color: colors.danger, fontSize: 12}}>
                                            ⚠ Désactive l'arrêt d'urgence du firmware (roues levées, inclinaison,
                                            bouton stop). À n'utiliser qu'en banc de test, jamais sur un robot avec lame.
                                        </Typography.Text>
                                    </div>
                                    <SchemaField><SchemaField.Number name={"maxChargeCurrent"} title={"Max Charge Current"} default={1.0}
                                        x-component-props={{step: 0.1, max: 5.0}}
                                        x-decorator-props={{tooltip: "Max charge current in Amps"}}
                                        x-component="NumberPicker" x-decorator="FormItem"/></SchemaField>
                                    <SchemaField><SchemaField.Number name={"limitVoltage150MA"} title={"Limit Voltage 150mA"} default={28.0}
                                        x-decorator-props={{tooltip: "Voltage limit during slow charge in Volts"}}
                                        x-component-props={{step: 0.1, max: 30.0}}
                                        x-component="NumberPicker" x-decorator="FormItem"/></SchemaField>
                                    <SchemaField><SchemaField.Number name={"maxChargeVoltage"} title={"Max Charge Voltage"} default={29.0}
                                        x-decorator-props={{tooltip: "Max charge voltage in Volts"}}
                                        x-component-props={{step: 0.1, max: 30.0}}
                                        x-component="NumberPicker" x-decorator="FormItem"/></SchemaField>
                                    <SchemaField><SchemaField.Number name={"batChargeCutoffVoltage"} title={"Bat Charge Cutoff Voltage"} default={28.0}
                                        x-decorator-props={{tooltip: "Max battery voltage allowed in Volts"}}
                                        x-component-props={{step: 0.1, max: 30.0}}
                                        x-component="NumberPicker" x-decorator="FormItem"/></SchemaField>
                                    <SchemaField><SchemaField.Number name={"oneWheelLiftEmergencyMillis"} title={"One Wheel Lift Emergency Millis"} default={10000}
                                        x-decorator-props={{tooltip: "Time in ms before emergency when one wheel is lifted"}}
                                        x-component-props={{step: 1}}
                                        x-component="NumberPicker" x-decorator="FormItem"/></SchemaField>
                                    <SchemaField><SchemaField.Number name={"bothWheelsLiftEmergencyMillis"} title={"Both Wheel Lift Emergency Millis"} default={1000}
                                        x-decorator-props={{tooltip: "Time in ms before emergency when both wheels are lifted"}}
                                        x-component-props={{step: 1}}
                                        x-component="NumberPicker" x-decorator="FormItem"/></SchemaField>
                                    <SchemaField><SchemaField.Number name={"tiltEmergencyMillis"} title={"Tilt Emergency Millis"} default={500}
                                        x-decorator-props={{tooltip: "Time in ms before emergency when mower is tilted"}}
                                        x-component-props={{step: 1}}
                                        x-component="NumberPicker" x-decorator="FormItem"/></SchemaField>
                                    <SchemaField><SchemaField.Number name={"stopButtonEmergencyMillis"} title={"Stop Button Emergency Millis"} default={100}
                                        x-decorator-props={{tooltip: "Time in ms before emergency when stop button is pressed"}}
                                        x-component-props={{step: 1}}
                                        x-component="NumberPicker" x-decorator="FormItem"/></SchemaField>
                                    <SchemaField><SchemaField.Number name={"playButtonClearEmergencyMillis"} title={"Play Button Clear Emergency Millis"} default={2000}
                                        x-decorator-props={{tooltip: "Time in ms to hold play button to clear emergency"}}
                                        x-component-props={{step: 1}}
                                        x-component="NumberPicker" x-decorator="FormItem"/></SchemaField>
                                    <SchemaField><SchemaField.Number name={"imuOnboardInclinationThreshold"} title={"IMU Onboard Inclination Threshold"} default={0x38}
                                        x-decorator-props={{tooltip: "IMU inclination threshold (0x2C=more allowed, 0x38=stock)"}}
                                        x-component-props={{step: 1, min: 0, max: 127}}
                                        x-component="NumberPicker" x-decorator="FormItem"/></SchemaField>
                                    <SchemaField><SchemaField.Boolean name={"externalImuAcceleration"} title={"External IMU Acceleration"} default={true}
                                        x-decorator-props={{tooltip: "Use external IMU for acceleration"}}
                                        x-component="Checkbox" x-decorator="FormItem"/></SchemaField>
                                    <SchemaField><SchemaField.Boolean name={"externalImuAngular"} title={"External IMU Angular"} default={true}
                                        x-decorator-props={{tooltip: "Use external IMU for angular"}}
                                        x-component="Checkbox" x-decorator="FormItem"/></SchemaField>
                                    <SchemaField><SchemaField.Boolean name={"masterJ18"} title={"Master J18"} default={true}
                                        x-decorator-props={{tooltip: "Use J18 as master"}}
                                        x-component="Checkbox" x-decorator="FormItem"/></SchemaField>
                                    <SchemaField><SchemaField.Boolean name={"perimeterWire"} title={"Use Perimeter wire"} default={true}
                                        x-decorator-props={{tooltip: "Use perimeter wire"}}
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
                                    message: "Validation failed",
                                    description: err.message,
                                });
                            }
                        });
                    }}>Flash Firmware</Button>
                    <Button onClick={props.onNext}>Skip</Button>
                </FormButtonGroup>
            </Col>
        </Row>
    </Form>;
};
