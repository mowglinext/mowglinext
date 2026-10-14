import {errorMessage} from "../utils/errors";
import {observeAsync} from "../utils/async";
import {useApi} from "./useApi.ts";
import {App} from "antd";
import {useEffect, useEffectEvent, useState} from "react";
import {useTranslation} from "react-i18next";

export const useConfig = (keys: string[]) => {
    const guiApi = useApi()
    const {notification} = App.useApp();
    const {t} = useTranslation();
    const [config, setConfig] = useState<Record<string, any>>({})
    const handleSetConfig = async (newConfig: Record<string, any>) => {
        try {
            const offsetConfig = await guiApi.config.keysSetCreate(newConfig)
            if (offsetConfig.error) {
                throw new Error(offsetConfig.error.error ?? "")
            }
            setConfig(oldConfig => ({
                ...oldConfig,
                ...offsetConfig.data
            }))
        } catch (e: any) {
            notification.error({
                message: t('hookNotifications.failedToSaveConfig'),
                description: e.message,
            })
        }
    }
    const reportLoadError = useEffectEvent((error: unknown) => {
        notification.error({message: t('hookNotifications.failedToLoadConfig'), description: errorMessage(error)});
    });

    // Both consumers declare a fixed mount-time key set (one passes a fresh
    // literal each render). Keep it as initialization input, not a refetch trigger.
    const [requestedKeys] = useState(() => Object.fromEntries(keys.map(key => [key, ""])));
    useEffect(() => {
        observeAsync((async () => {
            try {
                const offsetConfig = await guiApi.config.keysGetCreate(requestedKeys)
                if (offsetConfig.error) {
                    throw new Error(offsetConfig.error.error ?? "")
                }
                setConfig(offsetConfig.data)
            } catch (e: any) {
                reportLoadError(e)
            }
        })())
    }, [guiApi.config, requestedKeys])
    return {config, setConfig: handleSetConfig}
}