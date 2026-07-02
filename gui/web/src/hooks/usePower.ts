import {useEffect, useState} from "react";
import {Power} from "../types/ros.ts";
import {useWS} from "./useWS.ts";

export const usePower = () => {
    const [power, setPower] = useState<Power>({})
    const powerStream = useWS<string>(() => {
            console.log({
                message: "Power Stream closed",
            })
        }, () => {
            console.log({
                message: "Power Stream connected",
            })
        },
        (e) => {
            setPower((e as any))
        })
    useEffect(() => {
        powerStream.start("/api/mowglinext/subscribe/power",)
        return () => {
            powerStream.stop()
        }
    }, []);
    return power;
};
