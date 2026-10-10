import {PartSprite} from "./PartSprite";
import type {RobotView} from "../../utils/robotModel";

import {dockBounds} from "./dockGeometry";

export function DockGraphic({view="top",artwork="full"}:{view?:RobotView;artwork?:"full"|"map"}) {
    const b=dockBounds(view);
    return <g data-layer="styled-dock" transform={`translate(${b.x+b.width/2} ${b.y+b.height/2})`}>
        <PartSprite artwork={artwork} part="dock" view={view} width={b.width} height={b.height}
            fallback={<rect x={-b.width/2} y={-b.height/2} width={b.width} height={b.height} rx={.03} fill="#344740" stroke="#86bfaa" strokeWidth={.002}/>}/>
    </g>;
}
