import {useEffect, useState} from "react";
import {WheelTick} from "../types/ros.ts";
import {useWS} from "./useWS.ts";

export const useWheelTicks = () => {
    const [wheelTicks, setWheelTicks] = useState<WheelTick>({})
    const ticksStream = useWS<string>(() => {
            console.log({
                message: "Wheel Ticks Stream closed",
            })
        }, () => {
            console.log({
                message: "Wheel Ticks Stream connected",
            })
        },
        (e) => {
            setWheelTicks((e as any))
        })
    useEffect(() => {
        ticksStream.start("/api/mowglinext/subscribe/ticks",)
        return () => {
            ticksStream.stop()
        }
    }, []);
    return wheelTicks;
};