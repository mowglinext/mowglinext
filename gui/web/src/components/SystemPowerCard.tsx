import {Button, Card, Typography} from "antd";
import {DownOutlined, PoweroffOutlined} from "@ant-design/icons";
import {useTranslation} from "react-i18next";
import {PowerMenu} from "./PowerMenu.tsx";

export function SystemPowerCard() {
    const {t} = useTranslation();
    return <Card size="small" title={t("systemPower.title")}>
        <Typography.Paragraph type="secondary">{t("systemPower.description")}</Typography.Paragraph>
        <PowerMenu hostOnly>
            <Button icon={<PoweroffOutlined aria-hidden/>}>
                {t("systemPower.open")} <DownOutlined aria-hidden/>
            </Button>
        </PowerMenu>
    </Card>;
}
