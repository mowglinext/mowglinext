import {errorMessage} from "../utils/errors";
import {observeAsync} from "../utils/async";
import {useApi} from "./useApi.ts";
import {App} from "antd";
import {useEffect, useEffectEvent, useState} from "react";
import {useTranslation} from "react-i18next";

export const useEnv = () => {
    const guiApi = useApi()
    const {notification} = App.useApp();
    const {t} = useTranslation();
    const [env, setEnv] = useState<Record<string, any>>({})
    const reportLoadError = useEffectEvent((error: unknown) => {
        notification.error({message: t('hookNotifications.failedToLoadConfig'), description: errorMessage(error)});
    });

    useEffect(() => {
        observeAsync((async () => {
            try {
                const envs = await guiApi.config.envsList()
                if (envs.error) {
                    throw new Error(envs.error.error ?? "")
                }
                setEnv(envs.data)
            } catch (e: any) {
                reportLoadError(e)
            }
        })())
    }, [guiApi.config])
    return env
}