import {useMemo} from "react";
import {useTranslation} from "react-i18next";
import type {RobotGeometry} from "../../../hooks/useRobotDescription";
import {useMowerVisual} from "../../../hooks/useMowerVisual";
import {assemblyBounds} from "../../../components/robot/assemblyBounds";
import {LayeredMower} from "../../../components/robot/LayeredMower";
import {MapPlaneMarker} from "./MapPlaneMarker";

export function AssembledMowerMarker({robot,longitude,latitude,headingRad}: {
    robot:RobotGeometry;longitude:number;latitude:number;headingRad:number;
}) {
    const {t}=useTranslation();
    const [visual]=useMowerVisual();
    const bounds = useMemo(() => assemblyBounds(robot,"top",.005), [robot]);
    if (!robot.fromUrdf) return null;
    return <MapPlaneMarker bounds={bounds} longitude={longitude} latitude={latitude}
        headingRad={headingRad} label={t("mowerVisual.mapAlt")} testId="assembled-mower-marker" zIndex={999}>
        <LayeredMower artwork="map" robot={robot} {...visual} internalDetails={visual.transparent}/>
    </MapPlaneMarker>;
}
