import {DEFAULT_DOCK_STYLE} from "../../../components/robot/dockAssets";
import {APPEARANCE_REGISTRY} from "../../../components/robot/appearanceRegistry";
import {useTranslation} from "react-i18next";
import {DockGraphic} from "../../../components/robot/DockGraphic";
import {dockBounds, type DockVisualGeometry, type LayeredDockStyle} from "../../../components/robot/dockGeometry";
import {MapPlaneMarker} from "./MapPlaneMarker";

export function StyledDockMarker({longitude,latitude,headingRad,style=DEFAULT_DOCK_STYLE,geometry}: {longitude:number;latitude:number;headingRad:number;style?:LayeredDockStyle;geometry?:DockVisualGeometry}) {
    const {i18n}=useTranslation();
    return <MapPlaneMarker bounds={dockBounds("top",style,geometry)} longitude={longitude} latitude={latitude} headingRad={headingRad}
        label={APPEARANCE_REGISTRY.docks[style].label[i18n.resolvedLanguage?.startsWith("fr") ? "fr" : "en"]} testId="styled-dock-marker" zIndex={998}>
        <DockGraphic geometry={geometry} artwork="map" style={style}/>
    </MapPlaneMarker>;
}
