import {memo, useMemo} from "react";
import {useTranslation} from "react-i18next";
import {useRobotDescription} from "../../hooks/useRobotDescription";
import {useMultiplexStatus} from "../../hooks/useWS";
import {resolveMowerVisual} from "../../hooks/useMowerVisual";
import {visibleRobotGeometry} from "../../utils/robotModel";
import {resolveDockAppearance} from "../../constants/mowerAppearances";
import {DEFAULT_DOCK_STYLE} from "../robot/dockAssets";
import {dockVisualGeometry, dockedAssemblyBounds} from "../robot/dockGeometry";
import {normalizeShellColour} from "../robot/shellPaint";
import {LayeredMower} from "../robot/LayeredMower";
import {DockGraphic} from "../robot/DockGraphic";

/** A live-model illustration, not a map pose or a hardware editor. Telemetry
 * only toggles the paired dock; it never changes robot geometry or settings. */
export const StatusMowerIllustration = memo(function StatusMowerIllustration({settings, docked}: {
    settings: Record<string, unknown>; docked: boolean | undefined;
}) {
    const {t} = useTranslation();
    const live = useRobotDescription();
    const connection = useMultiplexStatus();
    const robot = useMemo(() => visibleRobotGeometry(live, settings), [live, settings]);
    const visual = resolveMowerVisual(settings);
    const dockStyle = resolveDockAppearance(visual.dockAppearance).layered ?? DEFAULT_DOCK_STYLE;
    const dock = dockVisualGeometry(settings, robot.wheelXOffset, dockStyle, robot.chassisCenterX);
    // Keep framing identical across contact changes: no jumping/scaling when
    // undocking, and no full-size assets or editor-only underbody components.
    const bounds = dockedAssemblyBounds(robot, "top", .025, dockStyle, dock);
    const showDock = docked === true && connection === "open";
    if (!live.fromUrdf) return null;
    return <div className="health-card-mower" data-testid="status-mower-illustration">
        <svg role="img" aria-label={t(showDock ? "mowgliNextPage.mowerOnDockTop" : "mowgliNextPage.mowerTop")}
            data-testid="status-mower-top" viewBox={[bounds.x,bounds.y,bounds.width,bounds.height].join(" ")}
            width="100%" height="220" style={{display:"block"}}>
            {showDock && <DockGraphic view="top" artwork="map" style={dockStyle} geometry={dock}/>}
            <LayeredMower robot={robot} style={visual.style} colour={normalizeShellColour(settings.mower_visual_colour)}
                view="top" transparent={false} layoutDetails={false} artwork="map"/>
        </svg>
    </div>;
});
