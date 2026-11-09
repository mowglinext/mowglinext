import {render, screen} from "@testing-library/react";
import {describe, it, expect, vi} from "vitest";
import {parseRobotUrdf} from "../../utils/robotModel";
import {ROBOT_URDF} from "../../test/robotUrdf";
import {StatusMowerIllustration} from "./StatusMowerIllustration";
const state=vi.hoisted(()=>({connection:"open", valid:true}));
vi.mock("../../hooks/useWS",()=>({useMultiplexStatus:()=>state.connection}));
vi.mock("../../hooks/useRobotDescription",()=>({useRobotDescription:()=>({...parseRobotUrdf(ROBOT_URDF)!,fromUrdf:state.valid})}));

describe("status card mower",()=>{
    it("toggles its registered side dock without rescaling the mower, and hides contact on disconnect",()=>{
        const props={settings:{mower_visual_style:"worx",lidar_enabled:false},docked:true};
        const {container,rerender}=render(<StatusMowerIllustration {...props}/>);
        const frame=screen.getByTestId('status-mower-top').getAttribute('viewBox');
        expect(container.querySelector('[data-dock-style]')).toHaveAttribute('data-dock-style','worx');
        rerender(<StatusMowerIllustration {...props} docked={false}/>);
        expect(container.querySelector('[data-dock-style]')).toBeNull();
        expect(screen.getByTestId('status-mower-top')).toHaveAttribute('viewBox',frame);
        state.connection="closed";
        rerender(<StatusMowerIllustration {...props}/>);
        expect(container.querySelector('[data-dock-style]')).toBeNull();
        state.connection="open";
    });
    it("does not render guessed geometry when the running URDF is unavailable",()=>{
        state.valid=false;
        const {container}=render(<StatusMowerIllustration settings={{}} docked/>);
        expect(container.querySelector('svg')).toBeNull();
        state.valid=true;
    });
});
