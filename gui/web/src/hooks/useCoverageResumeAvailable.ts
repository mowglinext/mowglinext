import {useEffect, useState} from "react";
import {useWS} from "./useWS.ts";

// Latched std_msgs/Bool from behavior_tree_node: true when a prior mowing
// session was interrupted and can be resumed. The GUI uses it to offer a
// "Start fresh" choice next to Start, instead of silently resuming mid-path
// (the "starts at 2nd/3rd line" report). Modeled on useEmergency.
export const useCoverageResumeAvailable = (): boolean => {
    const [available, setAvailable] = useState<boolean>(false);
    const stream = useWS<string>(
        () => {
            // stream closed — keep the last value
        },
        () => {
            // stream connected
        },
        (e) => {
            // std_msgs/Bool arrives as {data: boolean}
            setAvailable(!!(e as any)?.data);
        });
    useEffect(() => {
        stream.start("/api/mowglinext/subscribe/coverageResumeAvailable");
        return () => {
            stream.stop();
        };
    }, []);
    return available;
};
