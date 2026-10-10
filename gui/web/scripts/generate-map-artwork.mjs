// Deterministic web-asset preparation; preserves the approved source artwork.
// Run: node --experimental-strip-types scripts/generate-map-artwork.mjs
// Requires the project's pinned Playwright Chromium (npx playwright install chromium).
import {readFile,writeFile,mkdir} from "node:fs/promises";
import {fileURLToPath} from "node:url";
import {chromium} from "@playwright/test";
import {SHELLS} from "../src/components/robot/shellAssets.ts";
import {PARTS} from "../src/components/robot/partAssets.ts";
const source=new URL("../public/assets/robots/layered/",import.meta.url);
await mkdir(new URL("map/",source),{recursive:true});
const browser=await chromium.launch();
try {
    const page=await browser.newPage();
    let total=0;
    for(const [name,asset] of Object.entries({...SHELLS,...PARTS})){
        const png=(await readFile(new URL(name+".png",source))).toString("base64");
        const maxSize=name in SHELLS || name === "dock" ? 384 : name === "blade" ? 256 : 128;
        for(const view of ["top","side"]){
            const crop=Array.isArray(asset[view]) ? asset[view] : asset[view].crop;
            const data=await page.evaluate(async({png,crop,maxSize})=>{
                const img=new Image();img.src="data:image/png;base64,"+png;await img.decode();
                const [x,y,w,h]=crop, scale=Math.min(1,maxSize/Math.max(w,h));
                const canvas=document.createElement("canvas");
                canvas.width=Math.max(1,Math.round(w*scale));canvas.height=Math.max(1,Math.round(h*scale));
                const context=canvas.getContext("2d");context.imageSmoothingQuality="high";
                context.drawImage(img,x,y,w,h,0,0,canvas.width,canvas.height);
                // Alpha is preserved by the WebP encoder, including antialias edges.
                return canvas.toDataURL("image/webp",.85).split(",")[1];
            },{png,crop,maxSize});
            const bytes=Buffer.from(data,"base64");total+=bytes.length;
            await writeFile(new URL(`map/${name}-${view}.webp`,source),bytes);
        }
    }
    console.log(`Wrote cropped map WebP derivatives to ${fileURLToPath(new URL("map/",source))} (${total} bytes total).`);
} finally {await browser.close();}
