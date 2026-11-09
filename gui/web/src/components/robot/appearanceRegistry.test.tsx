import {render} from "@testing-library/react";
import {expect,it,vi} from "vitest";
vi.mock("./appearanceRegistry",async importOriginal=>{
    const {APPEARANCE_REGISTRY: original}=await importOriginal<typeof import("./appearanceRegistry")>();
    return {APPEARANCE_REGISTRY:{...original,
        shells:{...original.shells,"registry-test":{...original.shells.rounded,dock:"registry-test-dock",wheelsOverShell:{top:true,side:true}}},
        docks:{...original.docks,"registry-test-dock":{...original.docks.standard,appearanceId:"registry-test-dock"}},
    }};
});
import {MOWER_STYLES,type MowerStyle} from "./shellAssets";
import {resolveMowerVisual} from "../../hooks/useMowerVisual";
import {resolveDockAppearance} from "../../constants/mowerAppearances";
import {LayeredMower} from "./LayeredMower";
import {DockGraphic} from "./DockGraphic";
import type {LayeredDockStyle} from "./dockAssets";
import {parseRobotUrdf} from "../../utils/robotModel";
import {ROBOT_URDF} from "../../test/robotUrdf";
it("discovers and renders an unfamiliar shell/dock pair solely from registry metadata",()=>{
    const style="registry-test" as MowerStyle;
    expect(MOWER_STYLES).toContain(style);
    expect(resolveMowerVisual({mower_visual_style:style}).dockAppearance).toBe("registry-test-dock");
    expect(resolveDockAppearance("registry-test-dock").layered).toBe("registry-test-dock");
    const robot=parseRobotUrdf(ROBOT_URDF)!;
    for(const view of ["top","side"] as const){
        const {container,unmount}=render(<svg><LayeredMower robot={robot} style={style} view={view} transparent={false}/>
            <DockGraphic style={"registry-test-dock" as LayeredDockStyle} view={view}/></svg>);
        expect(container.querySelector('[data-mower-style="registry-test"]')).not.toBeNull();
        const shell=container.querySelector('[data-layer="shell"]')!;
        const wheels=container.querySelector('[data-layer="wheels"]')!;
        expect(shell.compareDocumentPosition(wheels)&Node.DOCUMENT_POSITION_FOLLOWING).toBeTruthy();
        expect(container.querySelector('[data-dock-style="registry-test-dock"]')).not.toBeNull();
        expect(container.querySelector('[data-layer="shell-art"] image')?.getAttribute("href")).toContain("rounded.png");
        unmount();
    }
});
