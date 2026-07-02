import {useEffect, useState} from "react";
import {Emergency} from "../types/ros.ts";
import {useWS} from "./useWS.ts";

export const useEmergency = () => {
    const [emergency, setEmergency] = useState<Emergency>({})
    const emergencyStream = useWS<string>(() => {
            console.log({
                message: "Emergency Stream closed",
            })
        }, () => {
            console.log({
                message: "Emergency Stream connected",
            })
        },
        (e) => {
            setEmergency((e as any))
        })
    useEffect(() => {
        emergencyStream.start("/api/mowglinext/subscribe/emergency",)
        return () => {
            emergencyStream.stop()
        }
    }, []);
    return emergency;
};
