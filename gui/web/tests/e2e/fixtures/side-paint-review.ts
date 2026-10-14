import {SHELLS, shellProjectionFrame, type MowerStyle} from "../../../src/components/robot/shellAssets";
import {paintedShell} from "../../../src/components/robot/usePaintedShell";
const root=document.getElementById("root")!;
root.innerHTML=`<style>body{margin:0;background:#0b2119;color:#ddf3e7;font:18px Arial}main{width:1500px;padding:30px;box-sizing:border-box}h1{font-size:30px;font-weight:500}h2{font-size:22px}section{border:1px solid #355044;border-radius:18px;padding:24px;margin-top:24px}.row{display:grid;grid-template-columns:1fr 1fr;gap:24px}.row svg,.row img{width:100%;height:290px;object-fit:contain}p{color:#9dbaaa;margin:0}</style><h1>Side panel paint registration</h1><p>Original artwork / production masked colour · same panel seams and shading</p>`;
for(const style of ["rounded","yardforce"] as MowerStyle[]){
 const a=SHELLS[style],frame=shellProjectionFrame(style,"side");
 const section=document.createElement("section");root.append(section);
 section.innerHTML=`<h2>${a.label.en}</h2><div class="row"><div><p>Original</p><svg viewBox="${frame.join(" ")}"><image href="/assets/robots/layered/${a.files.side}" width="${a.size[0]}" height="${a.size[1]}"/></svg></div><div><p>Painted panels · trim preserved</p><img alt="${a.label.en} painted side"/></div></div>`;
 const img=section.querySelector("img")!;
 img.src=await paintedShell(style,"side","full",a.paint!.suggestedColour);await img.decode();
}
root.dataset.ready="true";
