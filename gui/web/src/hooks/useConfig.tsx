import {decodeStringRecord} from "../utils/json";
import {apiErrorMessage, errorMessage, requestErrorMessage} from "../utils/errors";
import {observeAsync} from "../utils/async";
import {useApi} from "./useApi.ts";
import {App} from "antd";
import {useEffect, useEffectEvent, useState} from "react";
import {useTranslation} from "react-i18next";

export const useConfig = (keys: string[]) => {
    const guiApi = useApi()
    const {notification} = App.useApp();
    const {t} = useTranslation();
    const [config, setConfig] = useState<Record<string, string>>({})
    const handleSetConfig = async (newConfig: Record<string, string>) => {
        try {
            const offsetConfig = await guiApi.config.keysSetCreate(decodeStringRecord(newConfig))
            if (offsetConfig.error) {
                throw new Error(apiErrorMessage(offsetConfig.error) ?? "")
            }
            const savedConfig = decodeStringRecord(offsetConfig.data);
            setConfig(oldConfig => ({
                ...oldConfig,
                ...savedConfig
            }))
        } catch (e: unknown) {
            // Awaiting callers own the operation-specific failure toast. Never
            // resolve a failed write: useSettings would otherwise announce success.
            throw e instanceof Error ? e : new Error(requestErrorMessage(e) ?? errorMessage(e));
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
                    throw new Error(apiErrorMessage(offsetConfig.error) ?? "")
                }
                setConfig(decodeStringRecord(offsetConfig.data))
            } catch (e: unknown) {
                reportLoadError(e)
            }
        })())
    }, [guiApi.config, requestedKeys])
    return {config, setConfig: handleSetConfig}
}
