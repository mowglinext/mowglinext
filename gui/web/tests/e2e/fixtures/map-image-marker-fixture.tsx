import {useState} from "react";
import {createRoot} from "react-dom/client";
import Map, {useMap} from "react-map-gl/mapbox";
import mapboxgl from "mapbox-gl";
import {DOCK_APPEARANCES, DOCK_FOREGROUND_CLIP_PATH, MOWER_APPEARANCES} from "../../../src/constants/mowerAppearances.ts";
import {MapImageMarker} from "../../../src/pages/map/components/MapImageMarker.tsx";
import {PALETTE} from "../../../src/theme/colors.ts";
import "mapbox-gl/dist/mapbox-gl.css";

const CENTER: [number, number] = [-122.4194, 37.7749];
const STYLE = {
    version: 8 as const,
    sources: {},
    layers: [{id: "background", type: "background" as const, paint: {"background-color": PALETTE.bgCanvas}}],
};
mapboxgl.accessToken = "pk.test-no-network-token";
// The same renderer fixture can preview either bundled mower without a robot.
const generic = new URLSearchParams(window.location.search).get("mower") === "generic";
const IMAGE = MOWER_APPEARANCES[generic ? "generic" : "biltema-rm1000"].mowerImage!;
const MOWER_ALT = generic ? "Generic mower test image" : "RM1000 mower test image";

declare global {
    interface Window {
        mapImageMarkerTest?: {
            map: mapboxgl.Map;
            setHeading: (headingRad: number) => void;
            setDockAppearance: (appearanceId: "generic" | "biltema-rm1000") => void;
        };
    }
}

function MarkerFixture() {
    const {current: map} = useMap();
    const [heading, setHeading] = useState(0);
    const [dockAppearanceId, setDockAppearance] = useState<"generic" | "biltema-rm1000">(generic ? "generic" : "biltema-rm1000");
    const dockImage = DOCK_APPEARANCES[dockAppearanceId].image!;
    if (map && !window.mapImageMarkerTest) {
        window.mapImageMarkerTest = {map, setHeading, setDockAppearance};
    } else if (map && (window.mapImageMarkerTest?.setHeading !== setHeading ||
        window.mapImageMarkerTest?.setDockAppearance !== setDockAppearance)) {
        window.mapImageMarkerTest = {map, setHeading, setDockAppearance};
    }

    return (
        <>
            <MapImageMarker
                image={dockImage}
                alt="RM1000 dock test image"
                longitude={CENTER[0]}
                latitude={CENTER[1]}
                headingRad={heading}
                zIndex={998}
                onLoad={() => {}}
                onError={() => {}}
            />
            <MapImageMarker
                image={IMAGE}
                alt={MOWER_ALT}
                longitude={CENTER[0]}
                latitude={CENTER[1]}
                headingRad={heading}
                zIndex={999}
                onLoad={() => {}}
                onError={() => {}}
            />
            <MapImageMarker
                image={dockImage}
                alt=""
                longitude={CENTER[0]}
                latitude={CENTER[1]}
                headingRad={heading}
                clipPath={DOCK_FOREGROUND_CLIP_PATH}
                zIndex={1000}
                onLoad={() => {}}
                onError={() => {}}
            />
        </>
    );
}

function App() {
    return (
        <Map
            mapStyle={STYLE}
            initialViewState={{longitude: CENTER[0], latitude: CENTER[1], zoom: 19, bearing: 0, pitch: 0}}
            style={{width: "100vw", height: "100vh"}}
            attributionControl={false}
            maxZoom={25}
        >
            <MarkerFixture />
        </Map>
    );
}

createRoot(document.getElementById("root")!).render(<App />);
