import {useApi} from "../hooks/useApi.ts";
import {App, Card, Col, Divider, Row} from "antd";
import {PlayCircleOutlined, HomeOutlined, WarningOutlined} from '@ant-design/icons';
import AsyncButton from "./AsyncButton.tsx";
import React from "react";
import styled from "styled-components";
import AsyncDropDownButton from "./AsyncDropDownButton.tsx";
import {useHighLevelStatus} from "../hooks/useHighLevelStatus.ts";
import {HighLevelStatusConstants} from "../types/ros.ts";
import {useThemeMode} from "../theme/ThemeContext.tsx";

const ActionsCard = styled(Card)`
  .ant-card-body > button {
    margin-right: 10px;
    margin-bottom: 10px;
  }
`;

export const useMowerAction = () => {
    const guiApi = useApi()
    return (command: string, args: Record<string, any> = {}) => async () => {
        try {
            const res = await guiApi.mowglinext.callCreate(command, args)
            if (res.error) {
                throw new Error(res.error.error)
            }
        } catch (e: any) {
            throw new Error(e.message)
        }
    };
};

export const MowerActions: React.FC<React.PropsWithChildren<{bare?: boolean}>> = (props) => {
    const {highLevelStatus} = useHighLevelStatus();
    const mowerAction = useMowerAction()
    const {modal} = App.useApp();
    const {colors} = useThemeMode();

    // Home from IDLE means the robot is somewhere on the lawn (it's been
    // undocked) and the operator wants it to drive itself back to the dock.
    // The BT already accepts COMMAND_HOME from any non-charging state via
    // the HomeSequence guard in main_tree.xml, but we ask the operator to
    // confirm because the implied autonomous transit can be surprising
    // (collision_monitor stays active but the robot will plan a path
    // across whatever is in front of it).
    const sendHome = mowerAction("high_level_control", {Command: 2});
    const onHomeClick = async () => {
        if (highLevelStatus.state_name === "IDLE") {
            return new Promise<void>((resolve, reject) => {
                modal.confirm({
                    title: "Renvoyer le robot à la base ?",
                    content: (
                        <div>
                            <p>
                                Le robot va calculer un itinéraire vers la base et s'y rendre seul.
                            </p>
                            <p style={{marginBottom: 0, color: colors.textSecondary}}>
                                Assurez-vous que la zone entre le robot et la base est dégagée et que la
                                position de la base est correctement définie.
                            </p>
                        </div>
                    ),
                    okText: "Retour dock",
                    okType: "primary",
                    cancelText: "Annuler",
                    onOk: async () => {
                        try {
                            await sendHome();
                            resolve();
                        } catch (e) {
                            reject(e);
                        }
                    },
                    onCancel: () => resolve(),
                });
            });
        }
        return sendHome();
    };
    const actionMenuItems: {
        key: string,
        label: string,
        actions: { command: string, args: any }[],
        danger?: boolean
    }[] = [
        {
            key: "mower_s1",
            label: "Enregistrer une zone",
            actions: [{
                command: "high_level_control",
                args: {
                    Command: 3,
                }
            }]
        },
        {
            key: "mower_s2",
            label: "Tondre la zone suivante",
            actions: [{
                command: "high_level_control",
                args: {
                    Command: 4,
                }
            }]
        },
        {
            // Match MapToolbar: resting state is IDLE_DOCKED (BT never emits
            // plain IDLE except as the manual-mow fallthrough).
            key: (highLevelStatus.state_name == "IDLE_DOCKED" || highLevelStatus.state_name == "IDLE") ? "continue" : "pause",
            label: (highLevelStatus.state_name == "IDLE_DOCKED" || highLevelStatus.state_name == "IDLE") ? "Continuer" : "Pause",
            // No pause flag exists in the stack (the old mower_logic/
            // manual_pause_mowing command returned HTTP 500). Continue = START
            // (resumes via persisted mow_progress); Pause = HOME (dock).
            actions: (highLevelStatus.state_name == "IDLE_DOCKED" || highLevelStatus.state_name == "IDLE") ? [{
                command: "high_level_control",
                args: {
                    Command: 1,
                }
            }] : [{
                command: "high_level_control",
                args: {
                    Command: 2,
                }
            }]
        },
        {
            key: "emergency_off",
            "label": "Désactiver l'arrêt d'urgence",
            "danger": true,
            actions: [{
                command: "emergency",
                args: {
                    Emergency: 0,
                }
            }]
        },
        {
            key: "mow_forward",
            "label": "Lame avant",
            actions: [{
                command: "mow_enabled",
                args: {MowEnabled: 1, MowDirection: 0}
            }]
        },
        {
            key: "mow_backward",
            "label": "Lame arrière",
            actions: [{
                command: "mow_enabled",
                args: {MowEnabled: 1, MowDirection: 1}
            }]
        },
        {
            key: "mow_off",
            "label": "Lame arrêtée",
            "danger": true,
            actions: [{
                command: "mow_enabled",
                args: {MowEnabled: 0, MowDirection: 0}
            }]
        },
    ];
    let children = props.children;
    if (children && Array.isArray(children)) {
        children = children.map(c => {
            return c ? <Col>{c}</Col> : null
        })
    } else if (children) {
        children = <Col>{children}</Col>
    }
    const content = (
        <Row gutter={[8, 8]} justify={"start"}>
            {children}
            {children ? <Col><Divider type={"vertical"}/></Col> : null}
            <Col>
                {/* Gate Start on the numeric HL state, not the string state_name:
                    while the BT is AUTONOMOUS (state=2) another COMMAND_START is
                    a no-op at best and re-kicks the mission at worst. Operator
                    should use HOME or STOP instead. */}
                {highLevelStatus.state !== HighLevelStatusConstants.HIGH_LEVEL_STATE_AUTONOMOUS &&
                 (highLevelStatus.state_name === "IDLE" || highLevelStatus.state_name === "IDLE_DOCKED") ? (
                    <AsyncButton icon={<PlayCircleOutlined/>} type="primary" key="btnHLC1"
                                 onAsyncClick={mowerAction("high_level_control", {Command: 1})}
                    >Démarrer</AsyncButton>
                ) : null}
                {/* Home button is hidden only when the robot is already
                    docked (IDLE_DOCKED). From IDLE we show it so the operator
                    can recall the robot from anywhere on the lawn — see #175.
                    The click handler injects a confirmation modal in IDLE
                    because the autonomous transit is non-trivial. */}
                {highLevelStatus.state_name !== "IDLE_DOCKED" ? <AsyncButton icon={<HomeOutlined/>} type="primary" key="btnHLC2"
                                                                           onAsyncClick={onHomeClick}
                >Retour dock</AsyncButton> : null}
            </Col>
            <Col>
                {!highLevelStatus.emergency ?
                    <AsyncButton danger icon={<WarningOutlined/>} key="btnEmergencyOn" onAsyncClick={mowerAction("emergency", {Emergency: 1})}
                    >Arrêt d'urgence</AsyncButton> : null}
                {highLevelStatus.emergency ?
                    <AsyncButton danger icon={<WarningOutlined/>} key="btnEmergencyOff" onAsyncClick={mowerAction("emergency", {Emergency: 0})}
                    >Réarmer</AsyncButton> : null}
            </Col>
            <Col>
                <AsyncDropDownButton style={{display: "inline"}}  key="drpActions"  menu={{
                    items: actionMenuItems,
                    onAsyncClick: async (e) => {
                        const item = actionMenuItems.find(item => item.key == e.key)
                        for (const action of (item?.actions ?? [])) {
                            await mowerAction(action.command, action.args)();
                        }
                    }
                }}>
                    Plus
                </AsyncDropDownButton>
            </Col>
        </Row>
    );

    if (props.bare) {
        return content;
    }

    return <ActionsCard title={"Actions"} size={"small"}>
        {content}
    </ActionsCard>;
};