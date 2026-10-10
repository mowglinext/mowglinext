import {useTranslation} from "react-i18next";
import {DockGraphic} from "../../../components/robot/DockGraphic";
import {dockBounds} from "../../../components/robot/dockGeometry";
import {MapPlaneMarker} from "./MapPlaneMarker";

export function StyledDockMarker({longitude,latitude,headingRad}: {longitude:number;latitude:number;headingRad:number}) {
    const {t}=useTranslation();
    return <MapPlaneMarker bounds={dockBounds("top")} longitude={longitude} latitude={latitude} headingRad={headingRad}
        label={t("mowerVisual.dockStyle")} testId="styled-dock-marker" zIndex={998}>
        <DockGraphic/>
    </MapPlaneMarker>;
}
