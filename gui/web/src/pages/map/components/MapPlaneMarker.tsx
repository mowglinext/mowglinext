import {useEffect, useState, type ReactNode} from "react";
import {Marker, useMap} from "react-map-gl/mapbox";
import type {Bounds} from "../../../utils/robotModel";
import {mapPlaneTransform} from "./mapPlaneTransform";

/** Project a metre plane at a ROS pose, retaining heading, camera pitch and
 * perspective. The anchor is independent of the artwork's visible bounds. */
export function MapPlaneMarker({bounds:b,longitude,latitude,headingRad,label,testId,zIndex,children}: {
    bounds:Bounds;longitude:number;latitude:number;headingRad:number;label:string;
    testId:string;zIndex:number;children:ReactNode;
}) {
    const {current: map} = useMap();
    const [placement,setPlacement] = useState<{width:number;height:number;transform:string}>();
    useEffect(() => {
        if (!map) return;
        const update = () => {
            const anchor=map.project([longitude,latitude]);
            const corners=[[b.x,b.y],[b.x+b.width,b.y],[b.x+b.width,b.y+b.height],[b.x,b.y+b.height]].map(([sx,sy])=>{
                const east=-sy*Math.cos(headingRad)+sx*Math.sin(headingRad);
                const north=-sy*Math.sin(headingRad)-sx*Math.cos(headingRad);
                const point=map.project([longitude+east/(6378137*Math.cos(latitude*Math.PI/180))*180/Math.PI,
                    latitude+north/6378137*180/Math.PI]);
                return {x:point.x-anchor.x,y:point.y-anchor.y};
            });
            const scale=512*2**map.getZoom()/(2*Math.PI*6378137*Math.cos(latitude*Math.PI/180));
            const width=b.width*scale,height=b.height*scale;
            const transform=mapPlaneTransform(corners,width,height);
            // A stationary camera/pose can emit repeated move/resize events.
            // Keep the SVG subtree untouched when its projection is identical.
            setPlacement(previous => transform
                ? previous?.width === width && previous.height === height && previous.transform === transform
                    ? previous : {width,height,transform}
                : undefined);
        };
        update();
        map.on("move",update);
        map.on("resize",update);
        return () => {map.off("move",update);map.off("resize",update);};
    }, [map,latitude,longitude,headingRad,b.x,b.y,b.width,b.height]);
    if (!placement) return null;
    return <Marker longitude={longitude} latitude={latitude} anchor="center"
        rotationAlignment="viewport" pitchAlignment="viewport"
        style={{width:0,height:0,zIndex,pointerEvents:"none"}}>
        <svg data-testid={testId} role="img" aria-label={label}
            viewBox={[b.x,b.y,b.width,b.height].join(" ")}
            style={{position:"absolute",left:0,top:0,...placement,transformOrigin:"0 0",overflow:"visible"}}>
            {children}
        </svg>
    </Marker>;
}
