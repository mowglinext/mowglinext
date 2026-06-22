import {useEffect, useState} from "react";
import {AbsolutePose} from "../types/ros.ts";
import {useWS} from "./useWS.ts";

export const usePose = () => {
    const [pose, setPose] = useState<AbsolutePose>({})
    const poseStream = useWS<string>(() => {
            console.log({
                message: "POSE Stream closed",

            })
        }, () => {
            console.log({
                message: "POSE Stream connected",
            })
        },
        (e) => {
            setPose((e as any))
        })
    useEffect(() => {
        poseStream.start("/api/mowglinext/subscribe/pose",)
        return () => {
            poseStream.stop()
        }
    }, []);
    return pose;
};