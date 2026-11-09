import {render} from "@testing-library/react";
import {describe, expect, it, vi} from "vitest";
import {MowerView} from "./MowerPreview";
import {dockVisualGeometry} from "./dockGeometry";
import {parseRobotUrdf} from "../../utils/robotModel";
import {ROBOT_URDF} from "../../test/robotUrdf";
vi.mock("../../hooks/useMowerVisual", () => ({
    MOWER_STYLES: ["worx"],
    useMowerVisual: () => [{style: "worx", dockAppearance: "worx", transparent: false}, vi.fn()],
}));
vi.mock("react-i18next", () => ({useTranslation: () => ({t: (key: string) => key})}));
const robot = parseRobotUrdf(ROBOT_URDF)!;
describe("docked setup preview", () => {
    for (const view of ["top", "side"] as const) it(`updates ${view} dimensions with a clearer solid dock and faint transparent dock`, () => {
        const {container, rerender} = render(<MowerView robot={robot} view={view} showDock/>);
        const geometry = dockVisualGeometry({dock_visual_length: .8, dock_visual_width: .55, dock_visual_x_offset: .18, dock_visual_y_offset: .12});
        rerender(<MowerView robot={robot} view={view} showDock dockGeometry={geometry}/>);
        const dock = container.querySelector('[data-layer="styled-dock"]')!;
        expect(dock.parentElement).toHaveAttribute("opacity", "0.7");
        expect(dock.querySelector("svg")).toHaveAttribute("data-physical-width", view === "top" ? "0.55" : "0.8");
        const transform = dock.getAttribute("transform");
        rerender(<MowerView robot={robot} view={view} showDock dockGeometry={geometry} transparent/>);
        expect(container.querySelector('[data-layer="styled-dock"]')?.getAttribute("transform")).toBe(transform);
        expect(container.querySelector('[data-layer="styled-dock"]')?.parentElement).toHaveAttribute("opacity", "0.18");
    });
});
