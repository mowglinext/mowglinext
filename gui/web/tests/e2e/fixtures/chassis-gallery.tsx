import {createRoot} from "react-dom/client";
import {LayeredMower} from "../../../src/components/robot/LayeredMower";
import {parseRobotUrdf, previewRobotGeometry} from "../../../src/utils/robotModel";
import {ROBOT_URDF} from "../../../src/test/robotUrdf";

// Presentation gallery using the production renderer; not an extra app screen.
const robot=previewRobotGeometry(parseRobotUrdf(ROBOT_URDF)!,{
    chassis_length:.60,chassis_width:.45,chassis_height:.19,chassis_z_offset:-.05,
    wheel_radius:.1,wheel_width:.04,wheel_track:.325,caster_x_offset:.39,caster_track:.28,
});
const styles=[
    ["rounded","Rounded","Soft curves · compact silhouette"],
    ["sculpted","Sculpted","Flowing panels · contoured shoulders"],
    ["utility","Utility","Angular panels · geometric detail"],
    ["yardforce","Yardforce-inspired","Raised rails · familiar proportions"],
] as const;
createRoot(document.getElementById("root")!).render(<main>
    <style>{`
        *{box-sizing:border-box}body{margin:0;background:#061b16;color:#e1f5e9;font-family:Arial,sans-serif}
        main{width:1600px;padding:50px 56px 30px;background:radial-gradient(ellipse at 85% 0,#174333 0,transparent 55%),#061b16}
        header{display:flex;align-items:flex-end;justify-content:space-between;margin-bottom:32px}
        .eyebrow{font-size:13px;letter-spacing:3px;color:#79deb0;margin-bottom:12px}h1{margin:0;font-size:42px;font-weight:500;letter-spacing:-1px}
        header p{font-size:15px;line-height:1.7;color:#a4c6b7;margin:0;text-align:right}
        .grid{display:grid;grid-template-columns:1fr 1fr;gap:24px}
        article{border:1px solid #315344;border-radius:24px;background:linear-gradient(145deg,#122b22,#091d17);padding:25px 28px 20px}
        .title{display:flex;align-items:center;gap:14px}.number{border:1px solid #466954;border-radius:50%;width:32px;height:32px;display:grid;place-items:center;font-size:12px;color:#7ee6b3}
        h2{font-size:25px;font-weight:500;margin:0}article p{color:#96b4a5;font-size:13px;margin:10px 0 12px 46px}
        .views{display:grid;grid-template-columns:240px 1fr;gap:16px;align-items:center}.view{min-width:0}.view svg{display:block;width:100%;height:295px}
        .caption{font-size:11px;letter-spacing:1.5px;color:#91b7a3;text-align:center;margin:8px 0 0}
        footer{display:flex;justify-content:space-between;color:#85a995;font-size:13px;margin-top:25px}
    `}</style>
    <header><div><div className="eyebrow">MOWGLINEXT / CHASSIS COLLECTION</div><h1>Four styles. One robot model.</h1></div>
        <p>Graphite shells · mint details · rear stop button<br/>Same 600 × 450 × 190 mm geometry in every view</p></header>
    <div className="grid">{styles.map(([style,label,description],i)=><article key={style}>
        <div className="title"><span className="number">0{i+1}</span><h2>{label}</h2></div><p>{description}</p>
        <div className="views"><div className="view"><svg viewBox="-.255 -.51 .51 .66" aria-label={label+" top"}>
            <LayeredMower robot={robot} style={style} transparent={false} view="top" sensors={false} internalDetails={false}/>
        </svg><div className="caption">TOP · FRONT ↑</div></div>
        <div className="view"><svg viewBox="-.51 -.28 .66 .48" aria-label={label+" side"}>
            <path d="M -.49 .102 H .14" stroke="#476c59" strokeWidth=".001" strokeDasharray=".012 .012"/>
            <LayeredMower robot={robot} style={style} transparent={false} view="side" sensors={false} internalDetails={false}/>
        </svg><div className="caption">SIDE · FRONT ←</div></div></div>
    </article>)}</div>
    <footer><span>Choose once in Hardware · shared by Sensors and Map</span><span>Production renderer · illustrative shell artwork</span></footer>
</main>);
