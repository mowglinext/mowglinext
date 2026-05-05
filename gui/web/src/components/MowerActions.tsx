import {useApi} from "../hooks/useApi.ts";
import {App, Card, Col, Divider, Row} from "antd";
import {PlayCircleOutlined, HomeOutlined, WarningOutlined} from '@ant-design/icons';
import AsyncButton from "./AsyncButton.tsx";
import React from "react";
import styled from "styled-components";
import AsyncDropDownButton from "./AsyncDropDownButton.tsx";
import {useHighLevelStatus} from "../hooks/useHighLevelStatus.ts";
import {HighLevelStatusConstants} from "../types/ros.ts";

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
                    title: "Send robot home?",
                    content: (
                        <div>
                            <p>
                                The robot will plan a path back to the dock and drive itself there.
                            </p>
                            <p style={{marginBottom: 0, color: "rgba(0,0,0,0.55)"}}>
                                Make sure the area between the robot and the dock is clear and the dock pose
                                is set correctly.
                            </p>
                        </div>
                    ),
                    okText: "Send home",
                    okType: "primary",
                    cancelText: "Cancel",
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
            label: "Area Recording",
            actions: [{
                command: "high_level_control",
                args: {
                    Command: 3,
                }
            }]
        },
        {
            key: "mower_s2",
            label: "Mow Next Area",
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
            label: (highLevelStatus.state_name == "IDLE_DOCKED" || highLevelStatus.state_name == "IDLE") ? "Continue" : "Pause",
            actions: (highLevelStatus.state_name == "IDLE_DOCKED" || highLevelStatus.state_name == "IDLE") ? [{
                command: "mower_logic", args: {
                    Config: {
                        Bools: [{
                            Name: "manual_pause_mowing",
                            Value: false
                        }]
                    }
                }
            }, {
                command: "high_level_control",
                args: {
                    Command: 1,
                }
            }] : [{
                command: "mower_logic", args: {
                    Config: {
                        Bools: [{
                            Name: "manual_pause_mowing",
                            Value: true
                        }]
                    }
                }
            }]
        },
        {
            key: "emergency_off",
            "label": "Emergency Off",
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
            "label": "Blade Forward",
            actions: [{
                command: "mow_enabled",
                args: {MowEnabled: 1, MowDirection: 0}
            }]
        },
        {
            key: "mow_backward",
            "label": "Blade Backward",
            actions: [{
                command: "mow_enabled",
                args: {MowEnabled: 1, MowDirection: 1}
            }]
        },
        {
            key: "mow_off",
            "label": "Blade Off",
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
                    >Start</AsyncButton>
                ) : null}
                {/* Home button is hidden only when the robot is already
                    docked (IDLE_DOCKED). From IDLE we show it so the operator
                    can recall the robot from anywhere on the lawn — see #175.
                    The click handler injects a confirmation modal in IDLE
                    because the autonomous transit is non-trivial. */}
                {highLevelStatus.state_name !== "IDLE_DOCKED" ? <AsyncButton icon={<HomeOutlined/>} type="primary" key="btnHLC2"
                                                                           onAsyncClick={onHomeClick}
                >Home</AsyncButton> : null}
            </Col>
            <Col>
                {!highLevelStatus.emergency ?
                    <AsyncButton danger icon={<WarningOutlined/>} key="btnEmergencyOn" onAsyncClick={mowerAction("emergency", {Emergency: 1})}
                    >Emergency On</AsyncButton> : null}
                {highLevelStatus.emergency ?
                    <AsyncButton danger icon={<WarningOutlined/>} key="btnEmergencyOff" onAsyncClick={mowerAction("emergency", {Emergency: 0})}
                    >Emergency Off</AsyncButton> : null}
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
                    More
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