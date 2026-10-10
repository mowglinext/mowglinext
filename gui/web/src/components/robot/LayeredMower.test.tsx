import {render} from "@testing-library/react";
import {describe,it,expect} from "vitest";
import {LayeredMower} from "./LayeredMower";
import {SHELLS} from "./shellAssets";
import {MOWER_STYLES} from "../../hooks/useMowerVisual";
import {parseRobotUrdf,previewRobotGeometry} from "../../utils/robotModel";
import {ROBOT_URDF} from "../../test/robotUrdf";
const robot=parseRobotUrdf(ROBOT_URDF)!;
describe("layered mower",()=>{
    for(const style of MOWER_STYLES) for(const view of ["top","side"] as const) {
        it(style+" "+view+" retains asset, dimensions, sensors and rear stop in transparent mode",()=>{
            const {container,rerender}=render(<svg><LayeredMower robot={robot} style={style} view={view} transparent={false}/></svg>);
            const art=container.querySelector('[data-layer="shell-art"]')!.outerHTML;
            const wheels=container.querySelector('[data-layer="wheels"]')!.outerHTML;
            const sensors=Array.from(container.querySelectorAll("[data-sensor]")).map(s=>s.outerHTML);
            const stop=container.querySelector('[data-layer="stop"]')!;
            expect(Number(stop.getAttribute("data-robot-x"))).toBeLessThan(robot.wheelXOffset);
            expect(SHELLS[style].top[2]).toBeLessThan(SHELLS[style].size[0]);
            rerender(<svg><LayeredMower robot={robot} style={style} view={view} transparent/></svg>);
            expect(container.querySelector('[data-layer="shell"]')).toHaveAttribute("opacity","0.24");
            expect(container.querySelector('[data-layer="shell-art"]')!.outerHTML).toBe(art);
            expect(container.querySelector('[data-layer="wheels"]')!.outerHTML).toBe(wheels);
            expect(Array.from(container.querySelectorAll("[data-sensor]")).map(s=>s.outerHTML)).toEqual(sensors);
        });
    }
    it("scales the visible shell independently of wheels and sensor dimensions",()=>{
        const {container,rerender}=render(<svg><LayeredMower robot={robot} style="yardforce" transparent/></svg>);
        const wheel=container.querySelector('[data-layer="wheels"]')!.outerHTML;
        const next=previewRobotGeometry(robot,{chassis_length:.8,chassis_width:.5});
        rerender(<svg><LayeredMower robot={next} style="yardforce" transparent/></svg>);
        const shell=container.querySelector('[data-layer="shell-art"]')!;
        expect(shell).toHaveAttribute("width","0.5");
        expect(shell).toHaveAttribute("height","0.8");
        expect(container.querySelector('[data-layer="wheels"]')!.outerHTML).toBe(wheel);
    });
});

it("renders a forward rolling caster and independent cutting disc",()=>{
    const {container}=render(<svg><LayeredMower robot={robot} style="yardforce" transparent/></svg>);
    const caster=container.querySelector('[data-part-art="caster"] svg')!;
    expect(Number(caster.getAttribute("height"))).toBeGreaterThan(Number(caster.getAttribute("width")));
    const disc=container.querySelector('[data-layer="blade"] [data-part-art="blade"]');
    expect(disc).not.toBeNull();
});
