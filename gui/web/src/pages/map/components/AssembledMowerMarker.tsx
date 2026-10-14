import {useMemo} from "react";
import {useTranslation} from "react-i18next";
import type {RobotGeometry} from "../../../hooks/useRobotDescription";
import {useMowerVisual} from "../../../hooks/useMowerVisual";
import {assemblyBounds} from "../../../components/robot/assemblyBounds";
import {LayeredMower} from "../../../components/robot/LayeredMower";
import {MapPlaneMarker} from "./MapPlaneMarker";

export function AssembledMowerMarker({values,robot,longitude,latitude,headingRad}: {
    values:Record<string,unknown>;robot:RobotGeometry;longitude:number;latitude:number;headingRad:number;
}) {
    const {t}=useTranslation();
    const [visual]=useMowerVisual(values);
    const bounds = useMemo(() => assemblyBounds(robot,"top",.005), [robot]);
    if (!robot.fromUrdf) return null;
    return <MapPlaneMarker bounds={bounds} longitude={longitude} latitude={latitude}
        headingRad={headingRad} label={t("mowerVisual.mapAlt")} testId="assembled-mower-marker" zIndex={999}>
        {/* Navigation shows only the solid shell, drive wheels and external sensors.
            Underbody parts and console details belong in the layout editor. */}
        <LayeredMower artwork="map" robot={robot} style={visual.style} colour={visual.colour} transparent={false} layoutDetails={false}/>
    </MapPlaneMarker>;
}
