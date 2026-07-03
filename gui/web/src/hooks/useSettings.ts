import {useApi} from "./useApi.ts";
import {App} from "antd";
import {useEffect, useState} from "react";
import {useTranslation} from "react-i18next";
import {useConfig} from "./useConfig.tsx";

export enum SettingValueType {
    String = "string",
    Int = "int",
    Float = "float",
    Boolean = "boolean",
    Lat = "lat",
    Lon = "lon",
    Select = "select",
}

export enum SettingType {
    ConfigFile = 0,
    Db = 1,
}

export type Setting = { settingType: SettingType, description: string, help?: string, section: string } & ({
    type: SettingValueType.String,
    defaultValue: string,
} | {
    type: SettingValueType.Boolean,
    defaultValue: boolean,
} | {
    type: SettingValueType.Int,
    defaultValue: number,
} | {
    type: SettingValueType.Float,
    defaultValue: number,
} | {
    type: SettingValueType.Lat,
    defaultValue: number,
} | {
    type: SettingValueType.Lon,
    defaultValue: number,
} | {
    type: SettingValueType.Select,
    defaultValue: string,
    options: { id: string, label: string }[],
})
export const SettingsDesc: Record<string, Setting> = {
    "system.api.addr": {
        settingType: SettingType.Db,
        section: "API",
        type: SettingValueType.String, defaultValue: ":4006", description: "API address"
    },
    "system.api.webDirectory": {
        settingType: SettingType.Db,
        section: "API",
        type: SettingValueType.String, defaultValue: "/app/web", description: "API web directory"
    },
    "system.map.enabled": {
        settingType: SettingType.Db,
        section: "Map",
        type: SettingValueType.Boolean,
        defaultValue: false,
        description: "Enable map tiles",
    },
    "system.map.tileServer": {
        settingType: SettingType.Db,
        section: "Map",
        type: SettingValueType.String,
        defaultValue: "",
        description: "Tile server URL",
    },
    "system.map.tileUri": {
        settingType: SettingType.Db,
        section: "Map",
        type: SettingValueType.String,
        defaultValue: "/tiles/vt/lyrs=s,h&x={x}&y={y}&z={z}",
        description: "Tile server URI",
    },
    "system.mower.configFile": {
        settingType: SettingType.Db,
        section: "Mower",
        type: SettingValueType.String,
        defaultValue: "/config/mower_config.sh",
        description: "Mower config file",
    },
    "system.mqtt.enabled": {
        settingType: SettingType.Db,
        section: "GUI MQTT",
        type: SettingValueType.Boolean,
        defaultValue: false,
        description: "Enable GUI MQTT",
    },
    "system.mqtt.host": {
        settingType: SettingType.Db,
        section: "GUI MQTT",
        type: SettingValueType.String,
        defaultValue: ":1883",
        description: "GUI MQTT host",
    },
    "system.mqtt.prefix": {
        settingType: SettingType.Db,
        section: "GUI MQTT",
        type: SettingValueType.String,
        defaultValue: "/gui",
        description: "GUI MQTT prefix",
    },
    "system.homekit.enabled": {
        settingType: SettingType.Db,
        section: "HomeKit",
        type: SettingValueType.Boolean,
        defaultValue: false,
        description: "Enable HomeKit",
    },
    "system.homekit.pincode": {
        settingType: SettingType.Db,
        section: "HomeKit",
        type: SettingValueType.String,
        defaultValue: "00102003",
        description: "HomeKit Pin Code",
    },
    "system.ros.nodeName": {
        settingType: SettingType.Db,
        section: "ROS",
        type: SettingValueType.String,
        defaultValue: "mowglinext",
        description: "ROS node name",
    },
    "system.ros.masterUri": {
        settingType: SettingType.Db,
        section: "ROS",
        type: SettingValueType.String,
        defaultValue: "http://localhost:11311",
        description: "ROS master URI",
    },
    "system.ros.nodeHost": {
        settingType: SettingType.Db,
        section: "ROS",
        type: SettingValueType.String,
        defaultValue: "localhost",
        description: "ROS node host",
    }
}
export type SettingsConfig = {
    [P in keyof typeof SettingsDesc]: Record<P, typeof SettingsDesc[P]['defaultValue']>;
}
const SettingKeysFromDB = Object.keys(SettingsDesc).filter((key) => {
    return SettingsDesc[key].settingType === SettingType.Db
})
const flattenConfig = (newConfig: Record<string, any>): Record<string, any> => {
    const flatConfig: Record<string, any> = {}
    Object.keys(newConfig).forEach((key) => {
        // If the value is an object, flatten it recursively
        if (typeof newConfig[key] === "object") {
            const flat = flattenConfig(newConfig[key])
            Object.keys(flat).forEach((subKey) => {
                flatConfig[`${key}.${subKey}`] = flat[subKey]
            })
        } else {
            flatConfig[key] = newConfig[key]
        }
    })
    return flatConfig;
};
export const useSettings = () => {
    const guiApi = useApi()
    const {notification} = App.useApp();
    const {t} = useTranslation();
    const db = useConfig(SettingKeysFromDB)
    const [loading, setLoading] = useState<boolean>(false)
    const [settings, setSettings] = useState<Record<keyof typeof SettingsDesc, any>>({})
    useEffect(() => {
        if (db.config) {
            const newSettings: Record<string, any> = {}
            Object.keys(db.config).forEach((key) => {
                if (SettingsDesc[key]?.type === SettingValueType.Boolean) {
                    if (db.config[key] === "true") {
                        newSettings[key] = true
                    } else if (db.config[key] === "false") {
                        newSettings[key] = false
                    }
                } else {
                    newSettings[key] = db.config[key]
                }
            })
            setSettings((prev) => {
                return {...prev, ...newSettings}
            })
        }
    }, [db.config]);
    useEffect(() => {
        (async () => {
            try {
                setLoading(true)
                // Load shell config settings
                const settingsList = await guiApi.settings.settingsList()
                if (settingsList.error) {
                    throw new Error(settingsList.error.error)
                }
                const fetchedSettings = settingsList.data.settings ?? {};
                const newSettings: Record<string, any> = {}
                Object.keys(fetchedSettings).forEach((key) => {
                    if (SettingsDesc[key]?.type === SettingValueType.Boolean) {
                        if (fetchedSettings[key] === "True" || fetchedSettings[key] == "1") {
                            newSettings[key] = true
                        } else if (fetchedSettings[key] === "False" || fetchedSettings[key] == "0") {
                            newSettings[key] = false
                        }
                    } else {
                        newSettings[key] = fetchedSettings[key]
                    }
                })
                // Load YAML config settings (new snake_case keys)
                try {
                    const yamlSettings = await guiApi.settings.yamlList()
                    if (!yamlSettings.error && yamlSettings.data) {
                        Object.assign(newSettings, yamlSettings.data)
                    }
                } catch {
                    // YAML endpoint may not exist on older backends
                }
                setSettings((prev) => {
                    return {...prev, ...newSettings}
                })
                setLoading(false)
            } catch (e: any) {
                notification.error({
                    message: t("useSettingsHook.loadFailed"),
                    description: e.message,
                })
                setLoading(false)
            }
        })()
    }, [])
    const handleSetConfig = async (newConfig: SettingsConfig) => {
        try {
            newConfig = flattenConfig(newConfig)
            setLoading(true)
            const configFiltered = Object.keys(newConfig).reduce((acc, key) => {
                if (SettingsDesc[key]?.settingType === SettingType.ConfigFile) {
                    acc[key] = newConfig[key]
                }
                return acc
            }, {} as SettingsConfig)
            const dbFiltered = Object.keys(newConfig).reduce((acc, key) => {
                if (SettingsDesc[key]?.settingType === SettingType.Db) {
                    if (SettingsDesc[key]?.type === SettingValueType.Boolean) {
                        acc[key] = newConfig[key].toString()
                    } else {
                        acc[key] = newConfig[key]
                    }
                }
                return acc
            }, {} as Record<string, any>)
            const res = await guiApi.settings.settingsCreate(configFiltered)
            if (res.error) {
                throw new Error(res.error.error)
            }
            await db.setConfig(dbFiltered)
            notification.success({
                message: t("useSettingsHook.saved"),
            })
            setLoading(false)
        } catch (e: any) {
            notification.error({
                message: t("useSettingsHook.saveFailed"),
                description: e.message,
            })
            setLoading(false)
        }
    };
    return {settings, setSettings: handleSetConfig, loading}
}
