import {requestErrorMessage} from "../utils/errors";
import {App, Dropdown, type MenuProps} from "antd";
import * as React from "react";
import {DropdownButtonProps} from "antd/es/dropdown";
import {useTranslation} from "react-i18next";

type MenuClickEvent = Parameters<NonNullable<MenuProps["onClick"]>>[0];

export const AsyncDropDownButton: React.FC<DropdownButtonProps & {
    menu: DropdownButtonProps["menu"] & {
        onAsyncClick: (event: MenuClickEvent) => Promise<unknown>
    }
}> = (props) => {
    const {t} = useTranslation();
    const {notification} = App.useApp();
    const [loading, setLoading] = React.useState(false)
    // Controlled so the MAIN (left) segment opens the menu too — by default
    // Dropdown.Button's main button is a plain action button, and none of our
    // consumers give it an action, so it did nothing.
    const [menuOpen, setMenuOpen] = React.useState(false)
    const handleClick = (event: MenuClickEvent) => {
        setMenuOpen(false)
        if (props.menu.onAsyncClick === undefined) return;
        setLoading(true)
        props.menu.onAsyncClick(event).then(() => {
            setLoading(false)
        }).catch((e: unknown) => {
            setLoading(false)
            if (console.error)
                console.error(e);
            notification.error({
                message: t('asyncButton.errorOccurred'),
                description: requestErrorMessage(e),
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
