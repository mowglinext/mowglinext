import {render} from "@testing-library/react";
import {describe,it,expect,vi} from "vitest";
import {LayeredMower} from "./LayeredMower";
import * as projection from "./sensorArtworkProjection";
import {SHELLS, shellProjectionFrame} from "./shellAssets";
import {MOWER_STYLES} from "../../hooks/useMowerVisual";
import {parseRobotUrdf,previewRobotGeometry} from "../../utils/robotModel";
import {ROBOT_URDF} from "../../test/robotUrdf";
const robot=parseRobotUrdf(ROBOT_URDF)!;
describe("layered mower",()=>{
    for(const style of MOWER_STYLES) for(const view of ["top","side"] as const) {
        it(style+" "+view+" retains asset, dimensions and sensors in transparent mode",()=>{
            const {container,rerender}=render(<svg><LayeredMower robot={robot} style={style} view={view} transparent={false}/></svg>);
            const art=container.querySelector('[data-layer="shell-art"]')!.outerHTML;
            const wheels=container.querySelector('[data-layer="wheels"]')!.outerHTML;
            const sensors=Array.from(container.querySelectorAll("[data-sensor]")).map(s=>s.outerHTML);
            expect(container.querySelector('[data-layer="stop"]')).toBeNull();
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
        expect(shell).toHaveAttribute("data-physical-width","0.5");
        expect(shell).toHaveAttribute("data-physical-height","0.8");
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

it("occludes wheels beneath the shell in side and top views",()=>{
    for (const view of ["top","side"] as const) {
        const {container,unmount}=render(<svg><LayeredMower robot={robot} style="yardforce" view={view} transparent={false}/></svg>);
        const wheel=container.querySelector('[data-layer="wheels"]')!;
        const shell=container.querySelector('[data-layer="shell"]')!;
        expect(wheel.compareDocumentPosition(shell) & Node.DOCUMENT_POSITION_FOLLOWING).toBeTruthy();
        unmount();
    }
});

it("pose-only wrapper changes reuse sensor artwork, but geometry edits redraw",()=>{
    const spy=vi.spyOn(projection,"sensorArtworkProjection");
    const view=(pose:number,r=robot)=><svg data-pose={pose}><LayeredMower robot={r} style="yardforce" transparent={false}/></svg>;
    const {rerender}=render(view(0));
    const count=spy.mock.calls.length;
    expect(count).toBeGreaterThan(0);
    for(let i=1;i<=20;i++)rerender(view(i));
    expect(spy).toHaveBeenCalledTimes(count);
    rerender(view(21,{...robot,baseLength:.8}));
    expect(spy.mock.calls.length).toBeGreaterThan(count);
    spy.mockRestore();
});

it("map mode renders only shell, drive wheels, GPS and LiDAR",()=>{
    const {container,rerender}=render(<svg><LayeredMower robot={robot} style="yardforce" transparent={false} layoutDetails={false}/></svg>);
    expect(container.querySelector('[data-part-art="blade"]')).toBeNull();
    expect(container.querySelector('[data-layer="casters"]')).toBeNull();
    expect(container.querySelector('[data-layer="stop"]')).toBeNull();
    expect(container.querySelector('[data-layer="wheels"]')).not.toBeNull();
    expect(container.querySelector('[data-layer="shell"]')).toHaveAttribute("opacity","1");
    expect(Array.from(container.querySelectorAll("[data-sensor]")).map(e=>e.getAttribute("data-sensor")).sort()).toEqual(["gps","lidar"]);
    expect(container.querySelector('[data-sensor="imu"]')).toBeNull();
    rerender(<svg><LayeredMower robot={robot} style="yardforce" transparent layoutDetails/></svg>);
    expect(container.querySelector('[data-part-art="blade"]')).not.toBeNull();
    expect(container.querySelector('[data-layer="casters"]')).not.toBeNull();
    expect(container.querySelector('[data-layer="stop"]')).toBeNull();
    expect(container.querySelector('[data-sensor="imu"]')).not.toBeNull();
});

it("map derivatives retain all physical bounds and axle transforms",()=>{
    const {container,rerender}=render(<svg><LayeredMower robot={robot} style="yardforce" transparent/></svg>);
    const metricAttributes=()=>Array.from(container.querySelectorAll('[data-layer="shell-art"], [data-part-art] svg, [data-layer="wheels"] > g')).map(node=>
        ["data-physical-x","data-physical-y","data-physical-width","data-physical-height","x","y","width","height","viewBox","transform"].map(name=>node.getAttribute(name)));
    const before=metricAttributes();
    rerender(<svg><LayeredMower robot={robot} style="yardforce" transparent artwork="map"/></svg>);
    expect(metricAttributes()).toEqual(before);
    for(const image of container.querySelectorAll("image"))expect(new URL(image.getAttribute("href")!,"http://localhost").pathname).toMatch(/\/map\/.*\.webp$/);
});

it("shows the complete external Worx side wheel without changing its scale or position", () => {
    const {container, rerender} = render(<svg><LayeredMower robot={robot} style="yardforce" view="side" transparent={false}/></svg>);
    const wheel = container.querySelector('[data-layer="wheels"]')!.outerHTML;
    rerender(<svg><LayeredMower robot={robot} style="worx" view="side" transparent={false}/></svg>);
    const shell = container.querySelector('[data-layer="shell"]')!;
    const currentWheel = container.querySelector('[data-layer="wheels"]')!;
    expect(currentWheel.outerHTML).toBe(wheel);
    expect(shell.compareDocumentPosition(currentWheel) & Node.DOCUMENT_POSITION_FOLLOWING).toBeTruthy();
});

it("registers the Worx side panel against the full chassis including its hidden rear", () => {
    const visible = SHELLS.worx.side;
    const frame = shellProjectionFrame("worx", "side");
    expect(frame[0]).toBe(visible[0]);
    expect(frame[2]).toBeGreaterThan(visible[2] * 1.25);
    const {container, rerender} = render(<svg><LayeredMower robot={robot} style="worx" view="side" transparent={false}/></svg>);
    const shell = container.querySelector('[data-layer="shell-art"]')!;
    expect(shell).toHaveAttribute("viewBox", frame.join(" "));
    expect(Number(shell.getAttribute("data-physical-width"))).toBe(robot.baseLength);
    expect(Number(shell.getAttribute("data-physical-x"))).toBe(-robot.chassisCenterX - robot.baseLength / 2);
    rerender(<svg><LayeredMower robot={robot} style="worx" view="side" transparent artwork="map"/></svg>);
    const compact = container.querySelector('[data-layer="shell-art"]')!;
    expect(compact).toHaveAttribute("viewBox", frame.join(" "));
    expect(compact.querySelector("image")).toHaveAttribute("width", String(frame[2]));
    expect(compact.querySelector("image")).toHaveAttribute("x", String(frame[0]));
});
