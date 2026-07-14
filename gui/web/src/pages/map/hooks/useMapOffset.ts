import {useEffect, useRef, useState} from "react";
import {useTranslation} from "react-i18next";
import type {NotificationInstance} from "antd/es/notification/interface";

interface UseMapOffsetOptions {
    config: Record<string, string>;
    setConfig: (cfg: Record<string, string>) => Promise<void>;
    notification: NotificationInstance;
}

export function useMapOffset({config, setConfig, notification}: UseMapOffsetOptions) {
    const {t} = useTranslation();
    const [offsetX, setOffsetX] = useState(0);
    const [offsetY, setOffsetY] = useState(0);
    // Instance-scoped debounce timers. Previously module-scoped, so the full
    // MapPage and the MiniMap's compact MapPage shared one timer each and
    // clobbered each other's pending writes.
    const offsetXTimeoutRef = useRef<ReturnType<typeof setTimeout> | null>(null);
    const offsetYTimeoutRef = useRef<ReturnType<typeof setTimeout> | null>(null);

    useEffect(() => {
        const offX = parseFloat(config["gui.map.offset.x"] ?? "0");
        const offY = parseFloat(config["gui.map.offset.y"] ?? "0");
        if (!isNaN(offX)) setOffsetX(offX);
        if (!isNaN(offY)) setOffsetY(offY);
    }, [config]);

    const handleOffsetX = (value: number) => {
        if (offsetXTimeoutRef.current != null) clearTimeout(offsetXTimeoutRef.current);
        offsetXTimeoutRef.current = setTimeout(() => {
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
        if (offsetYTimeoutRef.current != null) clearTimeout(offsetYTimeoutRef.current);
        offsetYTimeoutRef.current = setTimeout(() => {
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
