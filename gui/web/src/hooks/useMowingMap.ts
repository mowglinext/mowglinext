import {useEffect, useState} from "react";
import {Map as MapType} from "../types/ros.ts";
import {useWS} from "./useWS.ts";

/**
 * Subscribes to the merged /map view (working_area + navigation_area + dock
 * pose + obstacles). Mirrors the inline stream that SchedulePage and MapPage
 * both maintain locally -- factored out so the dashboard widget can reuse the
 * same data.
 */
export const useMowingMap = () => {
    const [map, setMap] = useState<MapType>({});
    const stream = useWS<string>(
        () => { /* closed */ },
        () => { /* connected */ },
        (data) => {
            try { setMap(JSON.parse(data) as MapType); }
            catch { /* ignore */ }
        },
    );
    useEffect(() => {
        stream.start("/api/mowglinext/subscribe/map");
        return () => { stream.stop(); };
    }, []);
    return map;
};
