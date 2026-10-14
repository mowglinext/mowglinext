import {readFile, writeFile} from "node:fs/promises";
import {fileURLToPath} from "node:url";
import {MOWER_MODELS} from "../src/constants/mowerModels.ts";

// Only newly introduced geometry settings opt into preset-backed upgrade defaults.
// Never replay the complete preset over a calibrated robot during an upgrade.
export const UPGRADE_GEOMETRY_KEYS = ["chassis_z_offset"];
export function modelGeometryDefaults(models = MOWER_MODELS) {
    return Object.fromEntries(models.map(model => [model.value,
        Object.fromEntries(UPGRADE_GEOMETRY_KEYS.filter(key =>
            typeof model.defaults[key] === "number").map(key => [key, model.defaults[key]]))]));
}
export async function syncModelGeometryDefaults(write = false) {
    const output = JSON.stringify({
        _comment: "GENERATED from gui/web mowerModels.ts: npm run generate:model-geometry",
        models: modelGeometryDefaults(),
    }, null, 2) + "\n";
    for (const relative of ["../../asserts/model_geometry_defaults.json",
        "../../../ros2/src/mowgli_bringup/config/model_geometry_defaults.json"]) {
        const path = new URL(relative, import.meta.url);
        if (write) await writeFile(path, output);
        else if (await readFile(path, "utf8") !== output)
            throw new Error(`${fileURLToPath(path)} is stale; run npm run generate:model-geometry`);
    }
}
if (process.argv[1] === fileURLToPath(import.meta.url)) {
    await syncModelGeometryDefaults(process.argv.includes("--write"));
}
