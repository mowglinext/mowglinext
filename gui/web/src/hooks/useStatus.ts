import {useEffect, useState} from "react";
import {Status} from "../types/ros.ts";
import {useWS} from "./useWS.ts";

export const useStatus = () => {
    const [status, setStatus] = useState<Status>({})
    const statusStream = useWS<string>(() => {
            console.log({
                message: "Status Stream closed",
            })
        }, () => {
            console.log({
                message: "Status Stream connected",
            })
        },
        (e) => {
            setStatus((e as any))
        })
    useEffect(() => {
        statusStream.start("/api/mowglinext/subscribe/status",)
        return () => {
            statusStream.stop()
        }
    }, []);
    return status;
};