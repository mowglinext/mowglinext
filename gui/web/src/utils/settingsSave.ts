import { Api } from "../api/Api.ts";
import {apiErrorMessage, errorMessage} from "./errors";

type GuiApi = Api<unknown>;

type NotifyApi = {
    success: (args: { message: string; description?: string }) => void;
    error: (args: { message: string; description?: string }) => void;
};

export type SavePartialOptions = {
    successMessage?: string;
    successDescription?: string;
    errorMessage?: string;
    silentSuccess?: boolean;
    markRestartRequired?: boolean;
};

/**
 * Shared implementation behind useSettingsManager.savePartialValues and
 * useSettingsSchema.savePartialValues: diff the partial payload against the
 * currently-saved values, POST only the changed keys to /settings/yaml, then
 * let the caller merge the changed keys into its own state. Returns true on
 * success (including the zero-changes no-op) and false on failure.
 */
export const savePartialSettingsValues = async (params: {
    guiApi: GuiApi;
    notification: NotifyApi;
    currentValues: Record<string, unknown>;
    partialValues: Record<string, unknown>;
    applyChanged: (changed: Record<string, unknown>) => void;
    setBusy: (busy: boolean) => void;
    markRestartRequired?: () => void;
    defaultSuccessMessage: string;
    defaultErrorMessage: string;
    options?: SavePartialOptions;
}): Promise<boolean> => {
    const {
        guiApi, notification, currentValues, partialValues,
        applyChanged, setBusy, markRestartRequired, options,
    } = params;
    try {
        const changedPayload: Record<string, unknown> = {};
        for (const [key, value] of Object.entries(partialValues)) {
            if (JSON.stringify(value) !== JSON.stringify(currentValues[key])) {
                changedPayload[key] = value;
            }
        }

        if (Object.keys(changedPayload).length === 0) {
            return true;
        }

        setBusy(true);
        const res = await guiApi.settings.yamlCreate(changedPayload);
        if (res.error) {
            throw new Error(apiErrorMessage(res.error));
        }

        applyChanged(changedPayload);

        if (options?.markRestartRequired ?? true) {
            markRestartRequired?.();
        }

        if (!options?.silentSuccess) {
            notification.success({
                message: options?.successMessage ?? params.defaultSuccessMessage,
                description: options?.successDescription,
            });
        }
        return true;
    } catch (e) {
        notification.error({
            message: options?.errorMessage ?? params.defaultErrorMessage,
            description: errorMessage(e),
        });
        return false;
    } finally {
        setBusy(false);
    }
};
