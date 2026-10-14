import {ArtworkViewport} from "./ArtworkViewport";
import {artworkUrl} from "./artworkUrl";
import {useState, type ReactNode} from "react";
import type {RobotView} from "../../utils/robotModel";
import {PARTS, partBounds, type PartId} from "./partAssets";

export function PartSprite({part,view,width,height,fallback,artwork="full"}: {
    part: PartId; view: RobotView; width:number; height:number; fallback?:ReactNode; artwork?:"full"|"map";
}) {
    const [loaded,setLoaded]=useState(false);
    const asset=PARTS[part], region=asset[view];
    const sourceSize=asset.sizes?.[view] ?? asset.size;
    const [x,y,w,h]=region.crop;
    // The caster's reference is its tyre, not its taller fork. This keeps wheel
    // diameter and axle height true to URDF even with an asymmetric drawing.
    const bounds=partBounds(part,view,width,height);
    return <g data-part-art={part}>
        {!loaded && fallback}
        <ArtworkViewport {...bounds} crop={region.crop}>
            <image href={artworkUrl(artwork === "map" ? `map/${part}-${view}.webp` : asset.files[view])}
                x={artwork === "map" ? x : 0} y={artwork === "map" ? y : 0}
                width={artwork === "map" ? w : sourceSize[0]} height={artwork === "map" ? h : sourceSize[1]}
                onLoad={()=>setLoaded(true)} onError={()=>setLoaded(false)}/>
        </ArtworkViewport>
    </g>;
}
