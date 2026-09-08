import {useRef, useState} from "react";
import {Alert, Button, Card, Flex, Modal, Typography} from "antd";
import {PoweroffOutlined, ReloadOutlined} from "@ant-design/icons";
import {useTranslation} from "react-i18next";
import {useApi} from "../hooks/useApi.ts";

type PowerAction = "reboot" | "shutdown";

export function SystemPowerCard() {
    const {t} = useTranslation();
    const api = useApi();
    const [action, setAction] = useState<PowerAction | null>(null);
    const [pending, setPending] = useState(false);
    const [submitted, setSubmitted] = useState<PowerAction | null>(null);
    const [error, setError] = useState(false);
    const inFlight = useRef(false);

    const submit = async () => {
        if (!action || inFlight.current) return;
        inFlight.current = true;
        setPending(true);
        setError(false);
        try {
            await api.request({path: `/system/${action}`, method: "POST"});
            setSubmitted(action);
            setAction(null);
        } catch {
            // A power action can disconnect HTTP before its response arrives.
            // Do not claim failure or automatically resend a destructive request.
            setError(true);
        } finally {
            inFlight.current = false;
            setPending(false);
        }
    };

    return <>
        <Card size="small" title={t("systemPower.title")}>
            <Typography.Paragraph type="secondary">
                {t("systemPower.description")}
            </Typography.Paragraph>
            {submitted ? <Alert
                type="info"
                showIcon
                message={t(`systemPower.${submitted}Requested`)}
                description={t(`systemPower.${submitted}Result`)}
                action={submitted === "reboot" ? <Button onClick={() => window.location.reload()}>
                    {t("systemPower.reload")}
                </Button> : undefined}
            /> : <Flex gap={8} wrap>
                <Button icon={<ReloadOutlined aria-hidden/>} disabled={pending} onClick={() => {
                    setError(false);
                    setAction("reboot");
                }}>{t("systemPower.reboot")}</Button>
                <Button danger icon={<PoweroffOutlined aria-hidden/>} disabled={pending} onClick={() => {
                    setError(false);
                    setAction("shutdown");
                }}>{t("systemPower.shutdown")}</Button>
            </Flex>}
        </Card>
        <Modal
            open={action !== null}
            title={action ? t(`systemPower.${action}Title`) : ""}
            okText={action ? t(`systemPower.${action}`) : ""}
            cancelText={t("systemPower.cancel")}
            okButtonProps={{danger: action === "shutdown", disabled: error,
                "aria-label": action ? t(`systemPower.${action}`) : undefined}}
            cancelButtonProps={{disabled: pending, autoFocus: true}}
            confirmLoading={pending}
            closable={!pending}
            keyboard={!pending}
            maskClosable={false}
            onCancel={() => setAction(null)}
            onOk={submit}
        >
            <Typography.Paragraph>{action ? t(`systemPower.${action}Confirm`) : ""}</Typography.Paragraph>
            <Typography.Paragraph>{t("systemPower.parkFirst")}</Typography.Paragraph>
            {error && <Alert type="warning" showIcon message={t("systemPower.unconfirmed")}
                description={t("systemPower.unconfirmedHint")}/>}
        </Modal>
    </>;
}
