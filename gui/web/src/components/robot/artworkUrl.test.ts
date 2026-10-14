import {afterEach, expect, it, vi} from "vitest";
import {artworkUrl} from "./artworkUrl";
afterEach(()=>vi.unstubAllEnvs());
it("reuses URLs within a build but changes all asset URLs on a new build",()=>{
    for(const file of ["rounded.png","rounded-side-paint.svg","map/rounded-side-paint.png","map/dock-worx-side.webp"]) {
        vi.stubEnv("VITE_WEB_BUILD_ID","old-build");const old=artworkUrl(file);
        expect(artworkUrl(file)).toBe(old);
        vi.stubEnv("VITE_WEB_BUILD_ID","new-build");const next=artworkUrl(file);
        expect(next).not.toBe(old);
        const url=new URL(next,"http://localhost");
        expect(url.pathname).toBe("/assets/robots/layered/"+file);
        expect(url.searchParams.get("v")).toBe("new-build");
    }
});
it("encodes the build ID as one query value",()=>{
    vi.stubEnv("VITE_WEB_BUILD_ID","build & variant");
    const url=new URL(artworkUrl("rounded.png"),"http://localhost");
    expect([...url.searchParams]).toEqual([["v","build & variant"]]);
});
