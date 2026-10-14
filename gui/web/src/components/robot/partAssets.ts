import {APPEARANCE_REGISTRY} from "./appearanceRegistry";
export type PartId = keyof typeof APPEARANCE_REGISTRY.parts;
type Region = {crop:readonly [number,number,number,number];reference?:readonly [number,number,number,number]};
export type PartAtlas = {files: Record<"top"|"side",string>; mapMaxSize: number; size:readonly [number,number];sizes?:Partial<Record<"top"|"side",readonly [number,number]>>;top:Region;side:Region};
export const PARTS: Record<PartId,PartAtlas> = APPEARANCE_REGISTRY.parts;

export function partBounds(part:PartId,view:"top"|"side",width:number,height:number) {
    const region=PARTS[part][view];
    const [x,y,w,h]=region.crop;
    const [rx,ry,rw,rh]=region.reference ?? region.crop;
    return {x:(x-rx-rw/2)*width/rw,y:(y-ry-rh/2)*height/rh,width:w*width/rw,height:h*height/rh};
}
