import {useState, type ReactElement} from "react";
import {App, Dropdown, Modal, Space, Spin, Typography, type MenuProps} from "antd";
import {DesktopOutlined, PoweroffOutlined, ReloadOutlined, SettingOutlined} from "@ant-design/icons";
import {useTranslation} from "react-i18next";
import {useApi} from "../hooks/useApi.ts";
import {useMowerAction} from "./MowerActions.tsx";
import {restartMowgliStack} from "../utils/containers.ts";

// Shared existing power flow: the battery menu and Diagnostics differ only in
// their trigger and menu entries, not in handlers, confirmation or reconnect UI.
export function PowerMenu({children, hostOnly = false}: {children: ReactElement; hostOnly?: boolean}) {
    const {t} = useTranslation();
    const guiApi = useApi();
    const {notification, modal} = App.useApp();
    const mowerAction = useMowerAction();
    const rebootBoardAction = mowerAction("reboot_board", {});
    // A power action that takes the backend/host down — whole-stack "Restart
    // Mowgli" OR "Restart the Host" — shows this blocking overlay;
    // watchForGuiAndReload then polls and hard-reloads once the GUI answers
    // again. null = hidden. It also drives the restart-mowgli menu lock.
    // (Shutdown deliberately does NOT use it — the Host stays off.)
    const [reconnect, setReconnect] = useState<{title: string; body: string} | null>(null);
    // Controlled so we can close the dropdown before a confirm dialog opens —
    // otherwise the menu lingers behind the modal (antd renders it beneath).
    const [powerMenuOpen, setPowerMenuOpen] = useState(false);

    // After a whole-stack restart or a Host reboot the GUI goes down too, so poll
    // a lightweight backend endpoint every 15 s and hard-reload once it answers
    // again (usually back within an interval or two; a full Host reboot is longer).
    const watchForGuiAndReload = () => {
        const poll = window.setInterval(async () => {
            try {
                const r = await fetch("/api/system/info", {cache: "no-store"});
                if (r.ok) {
                    window.clearInterval(poll);
                    window.location.reload();
                }
            } catch {
                /* GUI still down — keep polling */
            }
        }, 15_000);
    };

    const restartMowgli = async () => {
        setReconnect({title: t('mowerStatus.restartingStackTitle'), body: t('mowerStatus.restartingStackBody')});
        try {
            // Restarts every mowgli-* container; the GUI last (fire-and-forget).
            await restartMowgliStack(guiApi);
        } catch (e: any) {
            // Non-GUI restarts failed before we took ourselves down — recover.
            setReconnect(null);
            notification.error({message: t('mowerStatus.mowgliRestartFailed'), description: e.message});
            return;
        }
        // GUI is now bouncing; wait for it to return, then reload the page.
        watchForGuiAndReload();
    };

    const rebootSystem = async () => {
        // A Host reboot takes the whole host (and this GUI) down, so reuse the
        // reconnect overlay + auto-reload, same as the whole-stack restart.
        setReconnect({title: t('mowerStatus.rebootingPiTitle'), body: t('mowerStatus.rebootingPiBody')});
        try {
            await guiApi.request({path: "/system/reboot", method: "POST"});
        } catch (e: any) {
            setReconnect(null);
            notification.error({message: t('mowerStatus.restartFailed'), description: e.message});
            return;
        }
        watchForGuiAndReload();
    };

    const shutdownSystem = async () => {
        try {
            await guiApi.request({path: "/system/shutdown", method: "POST"});
            notification.success({message: t('mowerStatus.shuttingDown')});
        } catch (e: any) {
            notification.error({message: t('mowerStatus.shutdownFailed'), description: e.message});
        }
    };

    const confirmAction = (title: string, content: string, onOk: () => Promise<void>) => {
        // Close the power menu first so it doesn't linger behind the modal.
        setPowerMenuOpen(false);
        // Use the App-context modal (App.useApp().modal), NOT the static
        // Modal.confirm: antd v5's static API renders outside the App/
        // ConfigProvider tree, so on React 19 the confirm dialog never
        // appears — the whole power menu (restart/reboot/shutdown) silently
        // did nothing because onOk never fired. The App-context variant
        // inherits the theme, z-index stack, and portal so it shows.
        modal.confirm({
            title,
            content,
            okText: t('mowerStatus.confirm'),
            okType: "danger",
            cancelText: t('mowerStatus.cancel'),
            onOk,
        });
    };

    const hostItems: MenuProps["items"] = [
        {
            key: "reboot",
            icon: <DesktopOutlined/>,
            label: t('mowerStatus.restartPi'),
            onClick: () => confirmAction(t('mowerStatus.restartPi'), t('mowerStatus.restartPiConfirm'), rebootSystem),
        },
        {
            key: "shutdown",
            icon: <PoweroffOutlined/>,
            label: t('mowerStatus.shutdownPi'),
            danger: true,
            onClick: () => confirmAction(t('mowerStatus.shutdownPi'), t('mowerStatus.shutdownPiConfirm'), shutdownSystem),
        },
    ];

    // Beginner-safe items at the top; destructive system/hardware actions
    // (board reset, Host reboot, shutdown) are tucked into an "Avancé"
    // submenu so they're not hit by accident. Command logic unchanged.
    const powerMenuItems: MenuProps["items"] = [
        {
            key: "restart-mowgli",
            icon: <ReloadOutlined/>,
            label: reconnect ? t('mowerStatus.restartingMowgli') : t('mowerStatus.restartMowgli'),
            disabled: !!reconnect,
            onClick: () => confirmAction(t('mowerStatus.restartMowgli'), t('mowerStatus.restartMowgliConfirm'), restartMowgli),
        },
        {type: "divider"},
        {
            key: "advanced",
            icon: <SettingOutlined/>,
            label: t('mowerStatus.advanced'),
            children: [
                {
                    key: "reboot-board",
                    icon: <ReloadOutlined/>,
                    label: t('mowerStatus.restartBoard'),
                    onClick: () => confirmAction(
                        t('mowerStatus.restartBoard'),
                        t('mowerStatus.restartBoardConfirm'),
                        rebootBoardAction),
                },
                ...hostItems,
            ],
        },
    ];

    return <>
        {/* Blocking overlay while a whole-stack restart or Host reboot takes the
            GUI down. The page reconnects and reloads via watchForGuiAndReload. */}
        <Modal
            open={!!reconnect}
            closable={false}
            maskClosable={false}
            keyboard={false}
            footer={null}
            title={reconnect?.title}
        >
            <Space>
                <Spin/>
                <Typography.Text>{reconnect?.body}</Typography.Text>
            </Space>
        </Modal>
        <Dropdown menu={{items: hostOnly ? hostItems : powerMenuItems}}
            trigger={["click"]} placement={hostOnly ? "bottomLeft" : "bottomRight"}
            open={powerMenuOpen} onOpenChange={setPowerMenuOpen}>
            {children}
        </Dropdown>
    </>;
}
