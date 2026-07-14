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
    // Controlled so the MAIN (left) segment opens the menu too — by default
    // Dropdown.Button's main button is a plain action button, and none of our
    // consumers give it an action, so it did nothing.
    const [menuOpen, setMenuOpen] = React.useState(false)
    const handleClick = (event: any) => {
        setMenuOpen(false)
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
    return <Dropdown.Button
        loading={loading}
        {...rest}
        open={menuOpen}
        onOpenChange={setMenuOpen}
        onClick={() => setMenuOpen(prev => !prev)}
        menu={{
            items: menu.items,
            onClick: handleClick,
        }}>{props.children}</Dropdown.Button>
}

export default AsyncDropDownButton;
