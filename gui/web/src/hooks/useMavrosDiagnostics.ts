import { useEffect, useState } from "react";
import { isDiagnosticStale, useDiagnostics, type DiagnosticArray } from "./useDiagnostics.ts";

export const mavrosDiagnosticFields = (diagnostics: DiagnosticArray, name: string, now: number) => {
    const entry = diagnostics.status?.find((item) => item.name === name);
    if (!entry || isDiagnosticStale(entry, now)) return null;
    return { entry, fields: Object.fromEntries(entry.values.map(({ key, value }) => [key, value])) };
};

export const useMavrosDiagnostics = () => {
    const { diagnostics } = useDiagnostics();
    const [now, setNow] = useState(Date.now);
    useEffect(() => {
        const timer = window.setInterval(() => setNow(Date.now()), 1000);
        return () => window.clearInterval(timer);
    }, []);
    return {
        safety: mavrosDiagnosticFields(diagnostics, "mowgli_mavros_bridge/hardware_emergency_stop", now),
        lift: mavrosDiagnosticFields(diagnostics, "mowgli_mavros_bridge/wheel_lift", now),
        odometry: mavrosDiagnosticFields(diagnostics, "mavros_esc_wheel_odometry/source", now),
    };
};
