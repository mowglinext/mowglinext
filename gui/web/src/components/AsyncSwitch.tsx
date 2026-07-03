import {App, Switch, SwitchProps} from "antd";
import * as React from "react";
import {useTranslation} from "react-i18next";

export const AsyncSwitch: React.FC<SwitchProps & {
    onAsyncChange: (checked: boolean, event: React.MouseEvent<HTMLButtonElement> | React.KeyboardEvent<HTMLButtonElement>) => Promise<any>
}> = (props) => {
    const {t} = useTranslation();
    const {notification} = App.useApp();
    const {onAsyncChange, onChange, ...rest} = props;
    const [loading, setLoading] = React.useState(false)
    const handleChange = (checked: boolean, event: React.MouseEvent<HTMLButtonElement> | React.KeyboardEvent<HTMLButtonElement>) => {
        if (onAsyncChange === undefined) return;
        // Keep any caller-supplied onChange as a side-effect, but the async
        // handler is what owns loading + error reporting (matches AsyncButton).
        onChange?.(checked, event)
        setLoading(true)
        onAsyncChange(checked, event).then(() => {
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
    return <Switch loading={loading} onChange={handleChange} {...rest}/>
}

export default AsyncSwitch;
