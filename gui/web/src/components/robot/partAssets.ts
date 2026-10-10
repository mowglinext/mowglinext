export type PartId = "drive-wheel" | "caster" | "gps" | "lidar" | "imu" | "blade" | "dock";
type Region = {crop:[number,number,number,number];reference?:[number,number,number,number]};
export type PartAtlas = {size:[number,number];top:Region;side:Region};
export const PARTS: Record<PartId,PartAtlas> = {
    // Blade reference is the cutting envelope centred on the axle, not the
    // asymmetric visible crop. Radius therefore remains the URDF cutting radius.
    blade:{size:[1774,887],top:{crop:[40,31,787,751],reference:[10,30,844,844]},side:{crop:[896,371,853,142],reference:[896,435,853,40]}},
    dock:{size:[1536,1024],top:{crop:[33,74,560,806]},side:{crop:[635,412,883,250]}},
    "drive-wheel":{size:[1774,887],top:{crop:[258,103,251,682]},side:{crop:[1058,138,629,620]}},
    caster:{size:[1536,1024],top:{crop:[142,56,384,909],reference:[192,56,282,909]},side:{crop:[718,115,724,803],reference:[750,240,690,677]}},
    gps:{size:[1774,887],top:{crop:[152,128,627,611]},side:{crop:[995,364,675,166]}},
    lidar:{size:[1983,793],top:{crop:[176,112,529,544]},side:{crop:[1257,219,563,337]}},
    imu:{size:[1983,793],top:{crop:[263,178,405,388]},side:{crop:[1263,354,457,77]}},
};

export function partBounds(part:PartId,view:"top"|"side",width:number,height:number) {
    const region=PARTS[part][view];
    const [x,y,w,h]=region.crop;
    const [rx,ry,rw,rh]=region.reference ?? region.crop;
    return {x:(x-rx-rw/2)*width/rw,y:(y-ry-rh/2)*height/rh,width:w*width/rw,height:h*height/rh};
}
