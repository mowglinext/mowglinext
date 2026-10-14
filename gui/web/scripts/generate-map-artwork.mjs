// Deterministic web-asset preparation; preserves the approved source artwork.
// Run: node --experimental-strip-types scripts/generate-map-artwork.mjs
// Requires the project's pinned Playwright Chromium (npx playwright install chromium).
import {readFile,writeFile,mkdir} from "node:fs/promises";
import {fileURLToPath} from "node:url";
import {chromium} from "@playwright/test";
import {APPEARANCE_REGISTRY} from "../src/components/robot/appearanceRegistry.ts";
const {shells:SHELLS,parts:PARTS}=APPEARANCE_REGISTRY;
const source=new URL("../public/assets/robots/layered/",import.meta.url);
await mkdir(new URL("map/",source),{recursive:true});
const browser=await chromium.launch();
try {
    const page=await browser.newPage();
    let total=0;
    for(const [name,asset] of Object.entries({...SHELLS,...PARTS})){
        const maxSize=asset.mapMaxSize;
        for(const view of ["top","side"]){
            const png=(await readFile(new URL(asset.files?.[view] ?? name+".png",source))).toString("base64");
            // Preserve registered transparent chassis extents in compact derivatives too.
            const crop=name in SHELLS ? (asset.frames?.[view] ?? asset[view]) : asset[view].crop;
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
            if (name in SHELLS && asset.paint) {
                const mask=(await readFile(new URL(asset.paint.masks[view],source))).toString("base64");
                const maskData=await page.evaluate(async({mask,crop,maxSize})=>{
                    const img=new Image();img.src="data:image/svg+xml;base64,"+mask;await img.decode();
                    const [x,y,w,h]=crop,scale=Math.min(1,maxSize/Math.max(w,h));
                    const canvas=document.createElement("canvas");
                    canvas.width=Math.max(1,Math.round(w*scale));canvas.height=Math.max(1,Math.round(h*scale));
                    const ctx=canvas.getContext("2d");ctx.imageSmoothingQuality="high";
                    ctx.drawImage(img,x,y,w,h,0,0,canvas.width,canvas.height);
                    return canvas.toDataURL("image/png").split(",")[1];
                },{mask,crop,maxSize});
                await writeFile(new URL(`map/${name}-${view}-paint.png`,source),Buffer.from(maskData,"base64"));
            }
            const bytes=Buffer.from(data,"base64");total+=bytes.length;
            await writeFile(new URL(`map/${name}-${view}.webp`,source),bytes);
        }
    }
    console.log(`Wrote cropped map WebP derivatives to ${fileURLToPath(new URL("map/",source))} (${total} bytes total).`);
} finally {await browser.close();}
