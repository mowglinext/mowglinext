import {test} from 'node:test';
import assert from 'node:assert/strict';
import {APPEARANCE_REGISTRY} from '../src/components/robot/appearanceRegistry.ts';
import {validateRegistry,shellEnum,syncSchema} from './appearance-registry.mjs';
test('registry assets and generated backend enum agree',async()=>{
    await validateRegistry(APPEARANCE_REGISTRY); await syncSchema(APPEARANCE_REGISTRY);
});
test('adding a shell and dock requires only metadata and artwork, not a new list',async()=>{
    const registry=structuredClone(APPEARANCE_REGISTRY);
    registry.docks['test-dock']={...registry.docks.standard,appearanceId:'test-dock'};
    registry.shells['test-shell']={...registry.shells.rounded,dock:'test-dock',label:{en:'Test shell',fr:'Coque test'},wheelsOverShell:{top:true,side:true}};
    await validateRegistry(registry);
    assert.ok(shellEnum(registry).includes('test-shell'));
});
for(const [name,mutate] of [
    ['invalid suggested colour',r=>{r.shells.rounded.paint.suggestedColour='red';}],
    ['missing paint mask',r=>{r.shells.rounded.paint.masks.top='missing.svg';}],
    ['wrong mask canvas',r=>{r.shells.rounded.paint.masks.top='worx-side-paint.svg';}],
    ['unknown dock',r=>{r.shells.rounded.dock='missing';}],
    ['missing label',r=>{delete r.shells.rounded.label.fr;}],
    ['bad image frame',r=>{r.shells.rounded.frames={side:[0,0,9999,10]};}],
    ['missing dock part',r=>{r.docks.standard.part='missing';}],
    ['missing artwork',r=>{r.shells.rounded.files.top='missing.png';}],
]) test(`rejects ${name}`,async()=>{
    const registry=structuredClone(APPEARANCE_REGISTRY);mutate(registry);
    await assert.rejects(()=>validateRegistry(registry));
});

// These assets cross the frontend/backend/ROS image boundary; never hand-copy defaults.
import {modelGeometryDefaults, syncModelGeometryDefaults} from './model-geometry-defaults.mjs';
test('upgrade geometry assets match the model presets', async () => {
    await syncModelGeometryDefaults();
    assert.deepEqual(modelGeometryDefaults([{value:'new-model', defaults:{chassis_z_offset:-.071, wheel_radius:.12}}]),
        {'new-model':{chassis_z_offset:-.071}});
});
