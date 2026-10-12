import {useCallback, useEffect, useRef, useState} from "react";
import {useApi} from "../../../hooks/useApi.ts";
import {
    forgetOriginal,
    parseStoredOriginals,
    rememberOriginal,
    type ObstacleOriginal,
    type XY,
} from "../utils/obstacleOriginals.ts";

/// GUI config-store key (the generic /config/keys routes; bitcask on the robot,
/// so every browser and the map backup see the same list).
export const OBSTACLE_ORIGINALS_KEY = "map.obstacleOriginals";

/// Owns the persisted "outline before the recorded-obstacle shrink" records.
/// The list is kept in a ref as well as state: the map editor reads it
/// synchronously inside an event handler, right after writing it.
export const useObstacleOriginals = () => {
    const api = useApi();
    const [records, setRecords] = useState<ObstacleOriginal[]>([]);
    const latest = useRef<ObstacleOriginal[]>([]);

    const persist = useCallback(async (next: ObstacleOriginal[]) => {
        latest.current = next;
        setRecords(next);
        try {
            await api.config.keysSetCreate({[OBSTACLE_ORIGINALS_KEY]: JSON.stringify(next)});
        } catch {
            // Best effort: losing this only means a later obstacle -> area
            // conversion keeps the shrunk outline instead of restoring it.
        }
    }, [api]);

    useEffect(() => {
        void (async () => {
            try {
                const res = await api.config.keysGetCreate({[OBSTACLE_ORIGINALS_KEY]: ""});
                const stored = parseStoredOriginals((res.data as Record<string, string> | undefined)?.[OBSTACLE_ORIGINALS_KEY]);
                latest.current = stored;
                setRecords(stored);
            } catch {
                // config store not reachable yet; start empty
            }
        })();
    }, [api]);

    /// The current list, for a synchronous read in an event handler.
    const getRecords = useCallback(() => latest.current, []);
    const remember = useCallback(
        (record: ObstacleOriginal) => persist(rememberOriginal(latest.current, record)), [persist]);
    const forget = useCallback(
        (shrunk: XY[]) => persist(forgetOriginal(latest.current, shrunk)), [persist]);

    return {records, getRecords, remember, forget, replaceAll: persist};
};
