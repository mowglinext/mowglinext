import {decodeConfigEnvironment} from "../utils/configEnvironment";
import type {ApiGetConfigResponse} from "../api/Api";
import {apiErrorMessage, errorMessage} from "../utils/errors";
import {observeAsync} from "../utils/async";
import {useApi} from "./useApi.ts";
import {App} from "antd";
import {useEffect, useEffectEvent, useState} from "react";
import {useTranslation} from "react-i18next";

export const useEnv = () => {
    const guiApi = useApi()
    const {notification} = App.useApp();
    const {t} = useTranslation();
    const [env, setEnv] = useState<ApiGetConfigResponse>({})
    const reportLoadError = useEffectEvent((error: unknown) => {
        notification.error({message: t('hookNotifications.failedToLoadConfig'), description: errorMessage(error)});
    });

    useEffect(() => {
        observeAsync((async () => {
            try {
                const envs = await guiApi.config.envsList()
                if (envs.error) {
                    throw new Error(apiErrorMessage(envs.error) ?? "")
                }
                setEnv(decodeConfigEnvironment(envs.data))
            } catch (e: unknown) {
                reportLoadError(e)
            }
        })())
    }, [guiApi.config])
    return env
}
