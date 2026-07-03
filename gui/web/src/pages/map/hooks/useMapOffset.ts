import {useEffect, useState} from "react";
import {useTranslation} from "react-i18next";
import type {NotificationInstance} from "antd/es/notification/interface";

let offsetXTimeout: ReturnType<typeof setTimeout> | null = null;
let offsetYTimeout: ReturnType<typeof setTimeout> | null = null;

interface UseMapOffsetOptions {
    config: Record<string, string>;
    setConfig: (cfg: Record<string, string>) => Promise<void>;
    notification: NotificationInstance;
}

export function useMapOffset({config, setConfig, notification}: UseMapOffsetOptions) {
    const {t} = useTranslation();
    const [offsetX, setOffsetX] = useState(0);
    const [offsetY, setOffsetY] = useState(0);

    useEffect(() => {
        const offX = parseFloat(config["gui.map.offset.x"] ?? "0");
        const offY = parseFloat(config["gui.map.offset.y"] ?? "0");
        if (!isNaN(offX)) setOffsetX(offX);
        if (!isNaN(offY)) setOffsetY(offY);
    }, [config]);

    const handleOffsetX = (value: number) => {
        if (offsetXTimeout != null) clearTimeout(offsetXTimeout);
        offsetXTimeout = setTimeout(() => {
            (async () => {
                try {
                    await setConfig({"gui.map.offset.x": value.toString()});
                } catch (e: any) {
                    notification.error({message: t('mapOffset.saveError'), description: e.message});
                }
            })();
        }, 1000);
        setOffsetX(value);
    };

    const handleOffsetY = (value: number) => {
        if (offsetYTimeout != null) clearTimeout(offsetYTimeout);
        offsetYTimeout = setTimeout(() => {
            (async () => {
                try {
                    await setConfig({"gui.map.offset.y": value.toString()});
                } catch (e: any) {
                    notification.error({message: t('mapOffset.saveError'), description: e.message});
                }
            })();
        }, 1000);
        setOffsetY(value);
    };

    return {offsetX, offsetY, handleOffsetX, handleOffsetY};
}
