import {useEffect, useState} from "react";
import {HighLevelStatus} from "../types/ros.ts";
import {useWS} from "./useWS.ts";

export const useHighLevelStatus = () => {
    const [highLevelStatus, setHighLevelStatus] = useState<HighLevelStatus>({})
    const highLevelStatusStream = useWS<string>(() => {
            console.log({
                message: "High Level Status Stream closed",
            })
        }, () => {
            console.log({
                message: "High Level Status Stream connected",
            })
        },
        (e) => {
            setHighLevelStatus((e as any))
        })
    useEffect(() => {
        highLevelStatusStream.start("/api/mowglinext/subscribe/highLevelStatus",)
        return () => {
            highLevelStatusStream.stop()
        }
    }, []);
    return {highLevelStatus, stop: highLevelStatusStream.stop, start: highLevelStatusStream.start};
}