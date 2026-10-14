import {createRoot} from "react-dom/client";
import {LayeredMower} from "../../../src/components/robot/LayeredMower";
import {parseRobotUrdf,previewRobotGeometry} from "../../../src/utils/robotModel";
import {ROBOT_URDF} from "../../../src/test/robotUrdf";
// Illustration proportions only. Selecting a shell never applies geometry.
const robot=previewRobotGeometry(parseRobotUrdf(ROBOT_URDF)!,{chassis_length:.75,chassis_width:.59,chassis_height:.297,chassis_center_x:.28,chassis_z_offset:-.070});
const colours=[{name:"Original silver",colour:""},{name:"Mowgli green",colour:"#3f8e79"},{name:"Ocean blue",colour:"#167bc1"},{name:"Warm orange",colour:"#e97822"}];
createRoot(document.getElementById("root")!).render(<main>
<style>{`
*{box-sizing:border-box}body{margin:0;background:#061b16;color:#e7f3ee;font-family:Arial,sans-serif}
main{width:1440px;padding:42px;background:radial-gradient(ellipse at 95% 0,#1c4035,transparent 55%),#061b16}
header{display:flex;justify-content:space-between;align-items:end;margin-bottom:26px}.eyebrow{color:#7edbb0;font-size:12px;letter-spacing:3px}h1{font-size:36px;font-weight:500;margin:10px 0 0}header p{color:#9ab9ab;font-size:14px;line-height:1.7;text-align:right}
.grid{display:grid;grid-template-columns:1fr 1fr;gap:20px}article{padding:25px;border:1px solid #315044;border-radius:22px;background:#10271f}h2{font-size:20px;font-weight:500;margin:0 0 14px}.views{display:grid;grid-template-columns:200px 1fr;align-items:center;gap:16px}svg{width:100%;height:285px;display:block}.caption{font-size:11px;color:#93b4a4;letter-spacing:1px;text-align:center}footer{font-size:13px;color:#9ab9ab;display:flex;justify-content:space-between;margin-top:25px}
[data-layer="wheels"],[data-layer="blade"]{display:none}
`}</style>
<header><div><div className="eyebrow">MOWGLINEXT / NEW SHELL</div><h1>Airseekers Tron-inspired</h1></div><p>Silver cover only · original shading retained<br/>Graphite frame, deck and bumper stay unchanged</p></header>
<div className="grid">{colours.map(({name,colour})=><article key={name}><h2>{name}</h2><div className="views">
<div><svg viewBox="-.315 -.68 .63 .82"><LayeredMower robot={robot} style="tron" colour={colour} view="top" sensors={false} layoutDetails={false} transparent={false}/></svg><div className="caption">TOP · FRONT ↑</div></div>
<div><svg viewBox="-.68 -.265 .82 .47"><LayeredMower robot={robot} style="tron" colour={colour} view="side" sensors={false} layoutDetails={false} transparent={false}/></svg><div className="caption">LEFT SIDE · FRONT ←</div></div>
</div></article>)}</div>
<footer><span>Actual application paint pipeline · no pre-painted colour variants</span><span>Shell-only study · standard dock pairing · illustrative proportions</span></footer>
</main>);
