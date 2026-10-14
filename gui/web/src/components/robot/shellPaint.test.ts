import {describe, expect, it} from "vitest";
import {normalizeShellColour, paintCoverage, recolourShellPixels} from "./shellPaint";
const source = new Uint8ClampedArray([60,60,60,255, 120,120,120,128, 200,200,200,255, 80,160,110,255, 10,20,30,0]);
describe("paint isolation and shading", () => {
    it("keeps alpha, transparent pixels and unmasked trim exactly", () => {
        const output=recolourShellPixels(source,new Float32Array([1,0,1,0,1]),"#e97822");
        expect([...output].filter((_,i)=>i%4===3)).toEqual([255,128,255,255,0]);
        expect(output.slice(4,8)).toEqual(source.slice(4,8));
        expect(output.slice(12)).toEqual(source.slice(12));
        expect(output[0]).toBeLessThan(output[8]);
        expect(output[0]).toBeGreaterThan(output[2]);
    });
    it("excludes mint accents even inside the registered panel mask", () => {
        const mask=new Uint8ClampedArray(source.length).fill(255);
        const coverage=paintCoverage(source,mask);
        expect(coverage[3]).toBe(0);
        expect(recolourShellPixels(source,coverage,"#167bc1").slice(12,16)).toEqual(source.slice(12,16));
    });
    it("retains original bytes for Original and rejects non-hex input", () => {
        for(const colour of ["", "red", "#abcd", "url(x)"]) {
            expect(normalizeShellColour(colour)).toBe("");
            expect(recolourShellPixels(source,new Float32Array(5).fill(1),colour)).toEqual(source);
        }
        expect(normalizeShellColour("#ABCDEF")).toBe("#abcdef");
    });
    it("blends mask edges without tinting transparent padding", () => {
        const solid=recolourShellPixels(source,new Float32Array(5).fill(1),"#167bc1");
        const edge=recolourShellPixels(source,new Float32Array(5).fill(.5),"#167bc1");
        expect(edge[0]).toBeCloseTo((source[0]+solid[0])/2,0);
        expect(edge.slice(16)).toEqual(source.slice(16));
    });
    it("rejects mismatched masks", () => {
        expect(()=>paintCoverage(source,new Uint8ClampedArray(4))).toThrow();
        expect(()=>recolourShellPixels(source,new Float32Array(2),"#167bc1")).toThrow();
    });
});
