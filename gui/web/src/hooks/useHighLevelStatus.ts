import {useEffect, useState} from "react";
import {HighLevelStatus} from "../types/ros.ts";
import {useWS} from "./useWS.ts";

export const useHighLevelStatus = () => {
    const [highLevelStatus, setHighLevelStatus] = useState<HighLevelStatus>({})
    const highLevelStatusStream = useWS<string>(
        () => {},
        () => {},
        (e) => {
            setHighLevelStatus((e as any))
        })
    const {start, stop} = highLevelStatusStream;
    useEffect(() => {
        start("/api/mowglinext/subscribe/highLevelStatus",)
        return () => {
            stop()
        }
    }, [start, stop]);
    return {highLevelStatus, stop: highLevelStatusStream.stop, start: highLevelStatusStream.start};
}