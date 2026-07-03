import {useApi} from "./useApi.ts";
import {App} from "antd";
import {useEffect, useState} from "react";
import {useTranslation} from "react-i18next";

export const useEnv = () => {
    const guiApi = useApi()
    const {notification} = App.useApp();
    const {t} = useTranslation();
    const [env, setEnv] = useState<Record<string, any>>({})
    useEffect(() => {
        (async () => {
            try {
                const envs = await guiApi.config.envsList()
                if (envs.error) {
                    throw new Error(envs.error.error ?? "")
                }
                setEnv(envs.data)
            } catch (e: any) {
                notification.error({
                    message: t('hookNotifications.failedToLoadConfig'),
                    description: e.message,
                })
            }
        })()
    }, [])
    return env
}