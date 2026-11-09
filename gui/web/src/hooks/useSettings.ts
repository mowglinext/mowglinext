import {errorMessage, requestErrorMessage} from "../utils/errors";
import {observeAsync} from "../utils/async";
import {useApi} from "./useApi.ts";
import {App} from "antd";
import {useEffect, useEffectEvent, useMemo, useRef, useState} from "react";
import {useTranslation} from "react-i18next";
import {useConfig} from "./useConfig.tsx";
import {parseBoolish} from "../utils/settingsValues.ts";
import {decodeSettings, flattenConfig, legacyEndpointMissing, type SettingsConfig} from '../utils/settingsContract';
import {isJsonObject} from '../utils/json';
export type {SettingsConfig} from '../utils/settingsContract';

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
const SettingKeysFromDB = Object.keys(SettingsDesc).filter(key => SettingsDesc[key].settingType === SettingType.Db);
export const useSettings = () => {
    const guiApi = useApi();
    const {notification} = App.useApp();
    const {t} = useTranslation();
    const db = useConfig(SettingKeysFromDB);
    const active = useRef(true);
    const saveLock = useRef(false);
    const [fetching, setFetching] = useState(true);
    const [saving, setSaving] = useState(false);
    const [fileSettings, setFileSettings] = useState<SettingsConfig>({});
    // DB descriptors own these keys regardless of API response arrival order.
    const settings = useMemo(() => {
        const entries: Array<[string, SettingsConfig[string]]> = Object.entries(fileSettings)
            .filter(([key]) => !SettingKeysFromDB.includes(key));
        for (const [key, value] of Object.entries(db.config)) {
            entries.push([key, SettingsDesc[key]?.type === SettingValueType.Boolean ? parseBoolish(value) ?? value : value]);
        }
        return Object.fromEntries(entries);
    }, [fileSettings, db.config]);
    const reportLoadError = useEffectEvent((error: unknown) => {
        notification.error({message: t('useSettingsHook.loadFailed'), description: requestErrorMessage(error) ?? errorMessage(error)});
    });
    useEffect(() => {
        let current = true;
        active.current = true;
        observeAsync((async () => {
            const results = await Promise.allSettled([guiApi.settings.settingsList(), guiApi.settings.yamlList()]);
            if (!current) return;
            let next: SettingsConfig = {};
            for (const [index, result] of results.entries()) {
                try {
                    if (result.status === 'rejected') throw result.reason instanceof Error ? result.reason : new Error(requestErrorMessage(result.reason) ?? 'Settings request failed');
                    const response = result.value;
                    if (response.error || response.ok === false) throw new Error(requestErrorMessage(response) ?? 'Settings request failed');
                    const raw: unknown = response.data;
                    if (index === 0 && (!isJsonObject(raw) || !('settings' in raw))) throw new Error('Invalid settings payload');
                    const data = decodeSettings(index === 0 && isJsonObject(raw) ? raw.settings : raw);
                    next = {...next, ...data};
                } catch (error) {
                    // Only an absent YAML endpoint is legacy compatibility.
                    const missing = legacyEndpointMissing(result.status === 'rejected' ? result.reason : result.value);
                    if (index !== 1 || !missing) reportLoadError(error);
                }
            }
            setFileSettings(next);
            setFetching(false);
        })());
        return () => {current = false; active.current = false;};
    }, [guiApi.settings]);
    const handleSetConfig = async (newConfig: SettingsConfig) => {
        if (!active.current || saveLock.current) return;
        saveLock.current = true;
        setSaving(true);
        try {
            const flat = flattenConfig(newConfig);
            const entries: Array<[string, string]> = [];
            for (const [key, value] of Object.entries(flat)) {
                if (!Object.prototype.hasOwnProperty.call(SettingsDesc, key) || SettingsDesc[key].settingType !== SettingType.Db) continue;
                if (SettingsDesc[key].type === SettingValueType.Boolean && typeof value === 'boolean') entries.push([key, String(value)]);
                else if (typeof value === 'string') entries.push([key, value]);
                else throw new Error(`Configuration value for ${key} must be a string`);
            }
            if (entries.length === 0) return; // This hook does not persist YAML.
            await db.setConfig(Object.fromEntries(entries));
            if (active.current) notification.success({message: t('useSettingsHook.saved')});
        } catch (error) {
            if (active.current) notification.error({message: t('useSettingsHook.saveFailed'), description: requestErrorMessage(error) ?? errorMessage(error)});
        } finally {saveLock.current = false; if (active.current) setSaving(false);}
    };
    return {settings, setSettings: handleSetConfig, loading: fetching || db.loading || saving};
};
