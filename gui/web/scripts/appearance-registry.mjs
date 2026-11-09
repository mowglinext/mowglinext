import assert from 'node:assert/strict';
import {readFile, writeFile, access} from 'node:fs/promises';
import {pathToFileURL} from 'node:url';
import {APPEARANCE_REGISTRY} from '../src/components/robot/appearanceRegistry.ts';
const assets = new URL('../public/assets/robots/layered/', import.meta.url);
const schemaFile = new URL('../../asserts/mower_config.schema.json', import.meta.url);
const views = ['top','side'];
const label = (value, id) => {
    for (const language of ['en','fr']) assert.equal(typeof value?.[language], 'string', `${id}: missing ${language} label`);
    assert.ok(value.en.trim() && value.fr.trim(), `${id}: empty label`);
};
const finite = (values, id) => assert.ok(values.every(Number.isFinite), `${id}: non-finite coordinates`);
const rect = (value, size, id) => {
    assert.equal(value?.length,4, `${id}: expected x/y/width/height`); finite(value,id);
    const [x,y,w,h]=value;
    assert.ok(x>=0 && y>=0 && w>0 && h>0 && x+w<=size[0] && y+h<=size[1],`${id}: bounds outside artwork`);
};
export async function validateRegistry(registry) {
    assert.ok(Object.hasOwn(registry.shells,registry.defaultShell),'Unknown default shell');
    assert.ok(Object.hasOwn(registry.docks,registry.defaultDock),'Unknown default dock');
    const appearanceIds=new Set(Object.keys(registry.legacyDocks));
    for(const [id,dock] of Object.entries(registry.docks)) {
        assert.ok(!appearanceIds.has(dock.appearanceId),`${id}: duplicate dock appearance ID`);
        appearanceIds.add(dock.appearanceId);
        assert.ok(Object.hasOwn(registry.parts,dock.part),`${id}: unknown dock part`);
        assert.ok(['rearAxle','chassisCenter'].includes(dock.origin),`${id}: invalid origin`);
        finite([dock.height,dock.offset.forward,dock.offset.left],id);
        assert.ok(dock.height>0,`${id}: non-positive height`); label(dock.label,id);
        for(const anchor of [dock.topAnchor,dock.sideAnchor]) {
            assert.equal(anchor.length,2); finite(anchor,id);
            assert.ok(anchor.every(n=>n>=0 && n<=1),`${id}: anchor outside artwork`);
        }
    }
    const all={...registry.parts,...registry.shells};
    assert.equal(Object.keys(all).length,Object.keys(registry.parts).length+Object.keys(registry.shells).length,'Shell/part IDs collide');
    for(const [id,asset] of Object.entries(all)) {
        assert.match(id,/^[a-z][a-z0-9-]*$/,`${id}: unsafe asset ID`);
        assert.ok(Number.isInteger(asset.mapMaxSize) && asset.mapMaxSize>0 && asset.mapMaxSize<=1024,`${id}: invalid map size`);
        const shell=Object.hasOwn(registry.shells,id);
        if(shell) {
            label(asset.label,id); assert.ok(appearanceIds.has(asset.dock),`${id}: unknown paired dock`);
            for(const value of Object.values(asset.wheelsOverShell ?? {})) assert.equal(typeof value,'boolean',`${id}: invalid wheel layer`);
        }
        for(const view of views) {
            const file=asset.files?.[view];
            assert.match(file ?? '',/^[a-zA-Z0-9_-]+\.png$/,`${id}/${view}: expected local PNG filename`);
            const bytes=await readFile(new URL(file,assets));
            assert.equal(bytes.subarray(1,4).toString(),'PNG',`${file}: not PNG`);
            const size=asset.sizes?.[view] ?? asset.size;
            assert.deepEqual(size,[bytes.readUInt32BE(16),bytes.readUInt32BE(20)],`${file}: canvas dimensions differ`);
            if(shell) rect(asset.cells?.[view],size,`${id}/${view} source cell`);
            if(shell && asset.paint) {
                assert.match(asset.paint.suggestedColour,/^#[0-9a-f]{6}$/i,`${id}: invalid suggested paint`);
                const maskFile=asset.paint.masks?.[view];
                assert.match(maskFile ?? '',/^[a-zA-Z0-9_-]+\.svg$/,`${id}/${view}: expected local SVG mask`);
                const mask=await readFile(new URL(maskFile,assets),'utf8');
                assert.ok(mask.includes(`viewBox="0 0 ${size[0]} ${size[1]}"`),`${id}/${view}: mask canvas differs`);
                assert.ok(!/<(?:script|image|foreignObject)\b|\bhref\s*=|\bon\w+\s*=/i.test(mask),`${id}/${view}: mask must be self-contained paths`);
            }
            rect(shell ? asset[view] : asset[view].crop,size,`${id}/${view}`);
            const reference=shell ? asset.frames?.[view] : asset[view].reference;
            if(reference) rect(reference,size,`${id}/${view} reference`);
        }
    }
}
export const shellEnum = registry => ['',...Object.keys(registry.shells)];
export async function syncSchema(registry, write=false) {
    const text=await readFile(schemaFile,'utf8');
    const schema=JSON.parse(text);
    const expected=shellEnum(registry);
    if(write) {
        const next=text.replace(/("mower_visual_style"\s*:\s*\{[\s\S]*?"enum"\s*:\s*)\[[^\]]*\]/,(_,prefix)=>prefix+JSON.stringify(expected));
        assert.notEqual(next.length,0);
        assert.deepEqual(JSON.parse(next).properties.hardware_settings.properties.mower_visual_style.enum,expected);
        await writeFile(schemaFile,next);
    } else assert.deepEqual(schema.properties.hardware_settings.properties.mower_visual_style.enum,expected,'Run npm run generate:appearances to refresh the generated shell enum');
}
if(process.argv[1] && import.meta.url===pathToFileURL(process.argv[1]).href) {
    await validateRegistry(APPEARANCE_REGISTRY);
    if(process.argv.includes('--write-schema')) await syncSchema(APPEARANCE_REGISTRY,true);
    else {
        for(const id of [...Object.keys(APPEARANCE_REGISTRY.shells),...Object.keys(APPEARANCE_REGISTRY.parts)])
            for(const view of views) await access(new URL(`map/${id}-${view}.webp`,assets));
        for(const [id,shell] of Object.entries(APPEARANCE_REGISTRY.shells))
            if(shell.paint) for(const view of views) await access(new URL(`map/${id}-${view}-paint.png`,assets));
        if(process.argv.includes('--check-schema')) await syncSchema(APPEARANCE_REGISTRY);
    }
    console.log('Appearance registry and artwork validated.');
}
