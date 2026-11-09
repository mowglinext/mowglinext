import {act, cleanup, renderHook} from "@testing-library/react";
import {afterEach, beforeEach, expect, it, vi} from "vitest";
import {useMowerVisual, resolveMowerVisual} from "./useMowerVisual";
const key = "mowgli.robot-visual.v1";
let saved: Map<string, string>;
beforeEach(() => {
    saved = new Map();
    vi.stubGlobal("localStorage", {
        getItem: (name: string) => saved.get(name) ?? null,
        setItem: (name: string, value: string) => saved.set(name, value),
    });
});
afterEach(() => {cleanup(); vi.unstubAllGlobals();});
it("reads the saved mower choice regardless of old browser choices", () => {
    saved.set(key, JSON.stringify({style: "utility", dockAppearance: "generic", transparent: true}));
    const {result} = renderHook(() => useMowerVisual({mower_visual_style:"worx"}));
    expect(result.current[0]).toEqual({style:"worx",dockAppearance:"worx",colour:"",transparent:true});
});
it("keeps draft appearance isolated from another consumer of saved settings", () => {
    const first = renderHook(({style}) => useMowerVisual({mower_visual_style:style}),{initialProps:{style:"yardforce"}});
    const second = renderHook(() => useMowerVisual({mower_visual_style:"yardforce"}));
    first.rerender({style:"worx"});
    expect(first.result.current[0].dockAppearance).toBe("worx");
    expect(second.result.current[0].style).toBe("yardforce");
    expect(saved.has(key)).toBe(false);
});
it("shares only local transparency, never persists a shell to browser storage", () => {
    const first = renderHook(() => useMowerVisual({mower_visual_style:"worx"}));
    const second = renderHook(() => useMowerVisual({mower_visual_style:"yardforce"}));
    act(() => first.result.current[1]({transparent:true}));
    expect(second.result.current[0].transparent).toBe(true);
    expect(second.result.current[0].style).toBe("yardforce");
    expect(JSON.parse(saved.get(key)!)).toEqual({transparent:true});
});
it("follows the hardware model for unset or invalid values without writing configuration", () => {
    expect(resolveMowerVisual({mower_model:"BiltemaRM1000",mower_visual_style:""}).style).toBe("rm1000");
    expect(resolveMowerVisual({mower_model:"YardForce500",mower_visual_style:"retired"}).style).toBe("yardforce");
    expect(resolveMowerVisual({}).style).toBe("sculpted");
    expect(saved.size).toBe(0);
});
it("pairs explicit styles even when they differ from the hardware model", () => {
    expect(resolveMowerVisual({mower_model:"BiltemaRM1000",mower_visual_style:"worx"})).toEqual({style:"worx",dockAppearance:"worx"});
});

it("shares configured pigment and safely falls back for invalid colours", () => {
    expect(renderHook(() => useMowerVisual({mower_visual_colour:"#AB1234"})).result.current[0].colour).toBe("#ab1234");
    expect(renderHook(() => useMowerVisual({mower_visual_colour:"url(https://example.com)"})).result.current[0].colour).toBe("");
});
