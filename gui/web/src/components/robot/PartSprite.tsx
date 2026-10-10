import {useState, type ReactNode} from "react";
import type {RobotView} from "../../utils/robotModel";
import {PARTS, partBounds, type PartId} from "./partAssets";

export function PartSprite({part,view,width,height,fallback}: {
    part: PartId; view: RobotView; width:number; height:number; fallback?:ReactNode;
}) {
    const [loaded,setLoaded]=useState(false);
    const asset=PARTS[part], region=asset[view];
    const [x,y,w,h]=region.crop;
    // The caster's reference is its tyre, not its taller fork. This keeps wheel
    // diameter and axle height true to URDF even with an asymmetric drawing.
    const bounds=partBounds(part,view,width,height);
    return <g data-part-art={part}>
        {!loaded && fallback}
        <svg {...bounds}
            viewBox={[x,y,w,h].join(" ")} preserveAspectRatio="none" overflow="hidden">
            <image href={`/assets/robots/layered/${part}.png`} width={asset.size[0]} height={asset.size[1]}
                onLoad={()=>setLoaded(true)} onError={()=>setLoaded(false)}/>
        </svg>
    </g>;
}
