import { useApi } from "./useApi.ts";
import { App } from "antd";
import { useCallback, useEffect, useState } from "react";

export type JSONSchema = {
    type?: string;
    title?: string;
    description?: string;
    properties?: Record<string, JSONSchemaProperty>;
    allOf?: JSONSchemaCondition[];
    required?: string[];
};

export type JSONSchemaProperty = {
    type?: string;
    title?: string;
    description?: string;
    default?: any;
    enum?: any[];
    minimum?: number;
    maximum?: number;
    "x-environment-variable"?: string;
    "x-yaml-node"?: string;
    "x-remap-values"?: Record<string, any>;
    "x-section"?: string;
    properties?: Record<string, JSONSchemaProperty>;
    allOf?: JSONSchemaCondition[];
    additionalProperties?: JSONSchemaProperty | boolean;
};

export type JSONSchemaCondition = {
    if?: {
        properties?: Record<string, { const?: any; enum?: any[] }>;
    };
    then?: {
        properties?: Record<string, JSONSchemaProperty>;
        allOf?: JSONSchemaCondition[];
    };
    else?: {
        properties?: Record<string, JSONSchemaProperty>;
    };
};

export const useSettingsSchema = () => {
    const guiApi = useApi();
    const { notification } = App.useApp();
    const [schema, setSchema] = useState<JSONSchema | null>(null);
    const [values, setValues] = useState<Record<string, any>>({});
    const [loading, setLoading] = useState(false);
    const [restartRequired, setRestartRequired] = useState(false);

    useEffect(() => {
        (async () => {
            try {
                setLoading(true);
                const [schemaRes, valuesRes] = await Promise.all([
                    guiApi.settings.schemaList(),
                    guiApi.settings.yamlList(),
                ]);
                if (schemaRes.error) {
                    throw new Error((schemaRes.error as any).error);
                }
                setSchema(schemaRes.data as JSONSchema);
                if (!valuesRes.error) {
                    setValues((valuesRes.data as Record<string, any>) || {});
                }
            } catch (e: any) {
                notification.error({
                    message: "Failed to load settings schema",
                    description: e.message,
                });
            } finally {
                setLoading(false);
            }
        })();
    }, []);

    const saveValues = useCallback(
        async (newValues: Record<string, any>) => {
            try {
                setLoading(true);
                const res = await guiApi.settings.yamlCreate(newValues);
                if (res.error) {
                    throw new Error((res.error as any).error);
                }
                setValues(newValues);
                setRestartRequired(true);
                notification.success({
                    message: "Settings saved",
                    description: "Restart ROS2 to apply the new configuration.",
                });
            } catch (e: any) {
                notification.error({
                    message: "Failed to save settings",
                    description: e.message,
                });
            } finally {
                setLoading(false);
            }
        },
        [guiApi, notification]
    );

    const savePartialValues = useCallback(
        async (
            partialValues: Record<string, any>,
            options?: {
                successMessage?: string;
                successDescription?: string;
                errorMessage?: string;
                silentSuccess?: boolean;
            },
        ) => {
            try {
                const changedPayload: Record<string, any> = {};
                for (const [key, value] of Object.entries(partialValues)) {
                    if (JSON.stringify(value) !== JSON.stringify(values[key])) {
                        changedPayload[key] = value;
                    }
                }

                if (Object.keys(changedPayload).length === 0) {
                    return true;
                }

                setLoading(true);
                const res = await guiApi.settings.yamlCreate(changedPayload);
                if (res.error) {
                    throw new Error((res.error as any).error);
                }
                setValues((prev) => ({ ...prev, ...changedPayload }));
                if (!options?.silentSuccess) {
                    notification.success({
                        message: options?.successMessage ?? "Settings saved",
                        description: options?.successDescription,
                    });
                }
                return true;
            } catch (e: any) {
                notification.error({
                    message: options?.errorMessage ?? "Failed to save settings",
                    description: e.message,
                });
                return false;
            } finally {
                setLoading(false);
            }
        },
        [guiApi, notification, values],
    );

    return { schema, values, saveValues, savePartialValues, loading, restartRequired };
};
