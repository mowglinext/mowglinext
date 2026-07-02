import {useEffect, useMemo, useRef, useState} from "react";
import {GnssStatus} from "../types/ros.ts";
import {useWS} from "./useWS.ts";
import {useDiagnostics} from "./useDiagnostics.ts";
import {deriveGnssStatusFromDiagnostics, hasTypedGnssStatusSample} from "../utils/gpsStatus.ts";

export const useGnssStatus = () => {
    const [gnssStatus, setGnssStatus] = useState<GnssStatus>({});
    const parseWarningLoggedRef = useRef(false);
    const {diagnostics} = useDiagnostics();
    const ignoreStreamEvent = () => {};
    const gnssStatusStream = useWS<string>(ignoreStreamEvent, ignoreStreamEvent, (payload) => {
        try {
            setGnssStatus(payload as unknown as GnssStatus);
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

    const diagnosticFallback = useMemo(
        () => deriveGnssStatusFromDiagnostics(diagnostics),
        [diagnostics],
    );

    return hasTypedGnssStatusSample(gnssStatus)
        ? gnssStatus
        : (diagnosticFallback ?? gnssStatus);
};
