import {expect,it} from "vitest";
import {mapPlaneTransform} from "./mapPlaneTransform";
it("maps every corner to a perspective quad",()=>{
    const corners=[{x:-20,y:-40},{x:60,y:-30},{x:80,y:70},{x:-50,y:80}];
    const css=mapPlaneTransform(corners,100,120)!;
    const m=css.slice(9,-1).split(",").map(Number);
    for(const [i,[x,y]] of [[0,0],[100,0],[100,120],[0,120]].entries()){
        const w=m[3]*x+m[7]*y+m[15];
        expect((m[0]*x+m[4]*y+m[12])/w).toBeCloseTo(corners[i].x);
        expect((m[1]*x+m[5]*y+m[13])/w).toBeCloseTo(corners[i].y);
    }
});
it("rejects invalid or degenerate projections",()=>{
    expect(mapPlaneTransform([],1,1)).toBeNull();
    expect(mapPlaneTransform(Array.from({length:4},()=>({x:0,y:0})),1,1)).toBeNull();
});
