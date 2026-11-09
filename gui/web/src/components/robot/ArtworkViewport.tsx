import type {ReactNode, SVGProps} from "react";

/** Keep nested SVG viewports in source-pixel units. Firefox quantises tiny
 * metre-sized viewport dimensions before the enclosing robot scale is applied,
 * shifting the raster even though getScreenCTM and the position markers agree.
 * Physical dimensions belong in the transform, not the nested viewport.
 * Keep that transform on a parent group: WebKit otherwise clips the image
 * against the untransformed nested viewport. */
export function ArtworkViewport({x, y, width, height, crop, children, ...props}: {
    x: number; y: number; width: number; height: number;
    crop: readonly [number, number, number, number]; children: ReactNode;
} & Omit<SVGProps<SVGSVGElement>, "x" | "y" | "width" | "height" | "viewBox" | "children">) {
    const [sx, sy, sw, sh] = crop;
    return <g transform={`translate(${x} ${y}) scale(${width/sw} ${height/sh})`}>
        <svg {...props} x={0} y={0} width={sw} height={sh}
            data-physical-x={x} data-physical-y={y} data-physical-width={width} data-physical-height={height}
            viewBox={[sx, sy, sw, sh].join(" ")} preserveAspectRatio="none" overflow="hidden">
            {children}
        </svg>
    </g>;
}
