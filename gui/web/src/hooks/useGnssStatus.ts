import {useEffect, useRef, useState} from "react";
import {GnssStatus} from "../types/ros.ts";
import {useWS} from "./useWS.ts";

export const useGnssStatus = () => {
    const [gnssStatus, setGnssStatus] = useState<GnssStatus>({});
    const parseWarningLoggedRef = useRef(false);
    const ignoreStreamEvent = () => {};
    const gnssStatusStream = useWS<string>(ignoreStreamEvent, ignoreStreamEvent, (payload) => {
        try {
            setGnssStatus(JSON.parse(payload) as GnssStatus);
            parseWarningLoggedRef.current = false;
        } catch (error) {
            if (!parseWarningLoggedRef.current) {
                console.warn("Ignoring malformed /gps/status frame", error);
                parseWarningLoggedRef.current = true;
            }
        }
    });
    useEffect(() => {
        gnssStatusStream.start("/api/mowglinext/subscribe/gnssStatus");
        return () => {
            gnssStatusStream.stop();
        };
    }, []);
    return gnssStatus;
};
