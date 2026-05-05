import React from "react";
import {App, Button, Card, Space, Typography} from "antd";
import {PoweroffOutlined, ReloadOutlined, WarningOutlined} from "@ant-design/icons";
import {useApi} from "../../hooks/useApi.ts";
import {SystemInfoComponent} from "../SystemInfoComponent.tsx";

const {Text, Paragraph} = Typography;

/**
 * Host-level actions: reboot and shutdown the Raspberry Pi (or whichever
 * box runs mowgli-gui). The backend handlers (gui/pkg/api/system.go) shell
 * out to `sudo reboot` / `sudo shutdown -h now` and return immediately —
 * the GUI will lose its connection in seconds, which is the expected
 * behaviour, so we don't try to reflect a "rebooting…" state.
 *
 * Both actions are wrapped in a danger-styled confirmation modal because
 * they kill the running ROS2 stack mid-action; an accidental click during
 * an autonomous mowing session would be very unfortunate.
 */
export const SystemSection: React.FC = () => {
    const guiApi = useApi();
    const {modal, notification} = App.useApp();

    const confirmAndRun = (
        action: "reboot" | "shutdown",
        title: string,
        body: React.ReactNode,
        run: () => Promise<unknown>,
    ) => () => {
        modal.confirm({
            title,
            icon: <WarningOutlined style={{color: "#ff4d4f"}}/>,
            content: body,
            okText: action === "reboot" ? "Reboot now" : "Shutdown now",
            okType: "danger",
            cancelText: "Cancel",
            onOk: async () => {
                try {
                    await run();
                    notification.info({
                        message: action === "reboot" ? "Reboot requested" : "Shutdown requested",
                        description:
                            action === "reboot"
                                ? "The host will go down and come back up in ~1 min."
                                : "The host will power down. You will need to power it back on manually.",
                    });
                } catch (e: any) {
                    notification.error({
                        message: action === "reboot" ? "Reboot failed" : "Shutdown failed",
                        description: e?.message ?? String(e),
                    });
                }
            },
        });
    };

    return (
        <div>
            <SystemInfoComponent/>

            <Card
                size="small"
                style={{marginTop: 16}}
                title={
                    <Space>
                        <PoweroffOutlined/>
                        <span>Power</span>
                    </Space>
                }
            >
                <Paragraph type="secondary" style={{marginTop: 0}}>
                    These actions reboot or power off the host (typically the Raspberry Pi)
                    and stop every container, including the autonomous stack. Use them only
                    when the robot is idle.
                </Paragraph>

                <Space wrap size={12}>
                    <Button
                        type="primary"
                        danger
                        icon={<ReloadOutlined/>}
                        onClick={confirmAndRun(
                            "reboot",
                            "Reboot the host?",
                            <div>
                                <Text>
                                    The system will reboot. The robot stack will be unavailable
                                    for ~1 minute.
                                </Text>
                            </div>,
                            () => guiApi.system.rebootCreate(),
                        )}
                    >
                        Reboot host
                    </Button>

                    <Button
                        danger
                        icon={<PoweroffOutlined/>}
                        onClick={confirmAndRun(
                            "shutdown",
                            "Shut down the host?",
                            <div>
                                <Text>
                                    The system will power off. You will need to physically power
                                    it back on.
                                </Text>
                            </div>,
                            () => guiApi.system.shutdownCreate(),
                        )}
                    >
                        Shut down host
                    </Button>
                </Space>
            </Card>
        </div>
    );
};
