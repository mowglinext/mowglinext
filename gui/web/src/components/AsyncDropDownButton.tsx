import {App, Dropdown} from "antd";
import * as React from "react";
import {DropdownButtonProps} from "antd/es/dropdown";
import {useTranslation} from "react-i18next";

export const AsyncDropDownButton: React.FC<DropdownButtonProps & {
    menu: DropdownButtonProps["menu"] & {
        onAsyncClick: (event: any) => Promise<any>
    }
}> = (props) => {
    const {t} = useTranslation();
    const {notification} = App.useApp();
    const [loading, setLoading] = React.useState(false)
    const handleClick = (event: any) => {
        if (props.menu.onAsyncClick === undefined) return;
        setLoading(true)
        props.menu.onAsyncClick(event).then(() => {
            setLoading(false)
        }).catch((e) => {
            setLoading(false)
            if (console.error)
                console.error(e);
            notification.error({
                message: t('asyncButton.errorOccurred'),
                description: e?.message,
            })
        })
    }
    const {menu, ...rest} = props
    return <Dropdown.Button loading={loading} {...rest} menu={{
        items: menu.items,
        onClick: handleClick,
    }}>{props.children}</Dropdown.Button>
}

export default AsyncDropDownButton;
