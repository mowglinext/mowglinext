import {APPEARANCE_REGISTRY} from "../../../src/components/robot/appearanceRegistry";
import {MOWER_STYLES, SHELLS} from "../../../src/components/robot/shellAssets";
import {DOCK_ARTWORK, type LayeredDockStyle} from "../../../src/components/robot/dockAssets";
import {dockVisualGeometry} from "../../../src/components/robot/dockGeometry";
import {DockGraphic} from "../../../src/components/robot/DockGraphic";
import {createRoot} from "react-dom/client";
import {LayeredMower} from "../../../src/components/robot/LayeredMower";
import {parseRobotUrdf, previewRobotGeometry} from "../../../src/utils/robotModel";
import {ROBOT_URDF} from "../../../src/test/robotUrdf";

const paint = new URLSearchParams(location.search).has("paint");
// Presentation gallery using the production renderer; not an extra app screen.
const robot=previewRobotGeometry(parseRobotUrdf(ROBOT_URDF)!,{
    chassis_length:.60,chassis_width:.45,chassis_height:.19,chassis_z_offset:-.05,
    wheel_radius:.1,wheel_width:.04,wheel_track:.325,caster_x_offset:.39,caster_track:.28,
});
const rm1000Robot=previewRobotGeometry(robot,{chassis_center_x:.15,caster_x_offset:.36});
const worxRobot=previewRobotGeometry(robot,{chassis_length:.54,chassis_width:.36,chassis_height:.14,chassis_z_offset:-.07,chassis_center_x:.16,wheel_radius:.11,wheel_track:.32,caster_x_offset:.34,caster_track:.22,caster_radius:.025});
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
    <header><div><div className="eyebrow">MOWGLINEXT / CHASSIS COLLECTION</div><h1>{MOWER_STYLES.length} styles. One robot model.</h1></div>
        <p>{paint ? "Masked body colours · preserved trim · original shading" : "Graphite shells · mint details · paired docks"}<br/>600 × 450 × 190 mm examples; shallow Worx 540 × 360 × 140 mm</p></header>
    <div className="grid">{MOWER_STYLES.map((style,i)=><article key={style}>
        <div className="title"><span className="number">0{i+1}</span><h2>{SHELLS[style].label.en}</h2></div><p>Shared dimensions · matching dock</p>
        <div className="views"><div className="view"><svg viewBox="-.255 -.51 .51 .66" aria-label={SHELLS[style].label.en+" top"}>
            <LayeredMower robot={style === "rm1000" ? rm1000Robot : style === "worx" ? worxRobot : robot} style={style} colour={paint ? SHELLS[style].paint?.suggestedColour : undefined} transparent={false} view="top" sensors={false}/>
        </svg><div className="caption">TOP · FRONT ↑</div></div>
        <div className="view"><svg viewBox="-.51 -.28 .66 .48" aria-label={SHELLS[style].label.en+" side"}>
            <path d="M -.49 .102 H .14" stroke="#476c59" strokeWidth=".001" strokeDasharray=".012 .012"/>
            <LayeredMower robot={style === "rm1000" ? rm1000Robot : style === "worx" ? worxRobot : robot} style={style} colour={paint ? SHELLS[style].paint?.suggestedColour : undefined} transparent={false} view="side" sensors={false}/>
        </svg><div className="caption">SIDE · FRONT ←</div></div></div>
    </article>)}{(Object.keys(DOCK_ARTWORK) as LayeredDockStyle[]).map((dock,i)=><article key={dock}>
        <div className="title"><span className="number">0{i+MOWER_STYLES.length+1}</span><h2>{APPEARANCE_REGISTRY.docks[dock].label.en}</h2></div>
        <p>Selected with the hardware appearance · one shared system</p>
        <div className="views"><div className="view"><svg viewBox="-.26 -.60 .52 .73" aria-label="Paired dock top">
            <DockGraphic style={dock} geometry={dockVisualGeometry({},0,dock,.16)}/>
        </svg><div className="caption">TOP · FRONT ↑</div></div>
        <div className="view"><svg viewBox="-.60 -.28 .73 .48" aria-label="Paired dock side">
            <DockGraphic style={dock} view="side" geometry={dockVisualGeometry({},0,dock,.16)}/>
        </svg><div className="caption">SIDE · FRONT ←</div></div></div>
    </article>)}</div>
    <footer><span>Choose once in Hardware · shared by Sensors and Map</span><span>Production renderer · illustrative shell artwork</span></footer>
</main>);
