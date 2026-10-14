/** Homography from an SVG pixel rectangle to four projected map-plane corners
 * (top-left, top-right, bottom-right, bottom-left), relative to the pose anchor.
 * A simple rotateX/rotateZ loses perspective across the mower at close zoom. */
export function mapPlaneTransform(p: {x:number;y:number}[], width:number, height:number): string | null {
    if(p.length!==4 || ![width,height,...p.flatMap(v=>[v.x,v.y])].every(Number.isFinite) || width<=0 || height<=0) return null;
    const [a,b,c,d]=p;
    const dx1=b.x-c.x,dx2=d.x-c.x,dx3=a.x-b.x+c.x-d.x;
    const dy1=b.y-c.y,dy2=d.y-c.y,dy3=a.y-b.y+c.y-d.y;
    const det=dx1*dy2-dx2*dy1;
    if(Math.abs(det)<1e-9) return null;
    const g=(dx3*dy2-dx2*dy3)/det,h=(dx1*dy3-dx3*dy1)/det;
    return "matrix3d("+[
        (b.x-a.x+g*b.x)/width,(b.y-a.y+g*b.y)/width,0,g/width,
        (d.x-a.x+h*d.x)/height,(d.y-a.y+h*d.y)/height,0,h/height,
        0,0,1,0,a.x,a.y,0,1,
    ].join(",")+")";
}
