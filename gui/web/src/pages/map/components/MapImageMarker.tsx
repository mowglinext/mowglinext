import {useEffect, useRef, useState} from "react";
import {Marker, useMap} from "react-map-gl/mapbox";
import type {MapImageAppearance} from "../../../constants/mowerAppearances.ts";
import {
    calculateMapImageDimensionsPx,
    getMapImageAnchorOffsetPx,
    offsetMapCoordinatesForward,
    rosHeadingToMapboxRotation,
} from "./mapImageMarkerMath.ts";

interface MapImageMarkerProps {
    image: MapImageAppearance;
    alt: string;
    longitude: number;
    latitude: number;
    headingRad: number;
    clipPath?: string;
    zIndex?: number;
    onLoad: () => void;
    onError: () => void;
}

/** A map-aligned, physically sized image pinned at its pose. The image stays
 * absent until decoded so the map's existing marker can remain as fallback.
 */
export function MapImageMarker(props: MapImageMarkerProps) {
    // A source change gets fresh decode/error/size state. Late decode callbacks
    // from an old source are ignored by the keyed inner component and ref guard.
    return <MapImageMarkerForSource key={props.image.src} {...props} />;
}

function MapImageMarkerForSource({
    image, alt, longitude, latitude, headingRad, clipPath, zIndex, onLoad, onError,
}: MapImageMarkerProps) {
    const {current: map} = useMap();
    const markerCoordinates = offsetMapCoordinatesForward(
        longitude,
        latitude,
        headingRad,
        image.forwardOffsetM ?? 0,
    );
    const markerLongitude = markerCoordinates?.[0];
    const markerLatitude = markerCoordinates?.[1];
    const [assetSizePx, setAssetSizePx] = useState<{width: number; height: number}>();
    const [decoded, setDecoded] = useState(false);
    const [loadFailed, setLoadFailed] = useState(false);
    const imageRef = useRef<HTMLImageElement>(null);
    const decodeSequence = useRef(0);
    const active = useRef(true);

    useEffect(() => {
        active.current = true;
        return () => {
            active.current = false;
            decodeSequence.current += 1;
        };
    }, []);

    const handleLoadError = (sourceImage: HTMLImageElement, sequence: number) => {
        if (!active.current || imageRef.current !== sourceImage || decodeSequence.current !== sequence) return;
        setLoadFailed(true);
        onError();
    };

    useEffect(() => {
        if (!map || markerLongitude === undefined || markerLatitude === undefined) return;

        const updateSize = () => {
            const size = calculateMapImageDimensionsPx(
                (coordinate) => map.project(coordinate),
                markerLongitude,
                markerLatitude,
                image.visibleLengthM,
                image.visibleLengthFraction,
                image.visibleWidthM,
                image.visibleWidthFraction,
            );
            setAssetSizePx(size);
        };

        updateSize();
        map.on("move", updateSize);
        map.on("resize", updateSize);
        return () => {
            map.off("move", updateSize);
            map.off("resize", updateSize);
        };
    }, [map, markerLongitude, markerLatitude, image.visibleLengthM, image.visibleLengthFraction, image.visibleWidthM, image.visibleWidthFraction]);

    if (markerLongitude === undefined || markerLatitude === undefined || !assetSizePx || loadFailed) return null;
    const imageOffset = getMapImageAnchorOffsetPx(assetSizePx.width, assetSizePx.height, image.poseAnchor);

    return (
        <Marker
            longitude={markerLongitude}
            latitude={markerLatitude}
            anchor="center"
            rotation={rosHeadingToMapboxRotation(headingRad + (image.headingOffsetRad ?? 0))}
            rotationAlignment="map"
            pitchAlignment="map"
            style={{
                width: assetSizePx.width,
                height: assetSizePx.height,
                pointerEvents: "none",
                ...(zIndex === undefined ? {} : {zIndex}),
            }}
        >
            <img
                ref={imageRef}
                src={image.src}
                alt={alt}
                draggable={false}
                onLoad={(event) => {
                    const sourceImage = event.currentTarget;
                    const sequence = ++decodeSequence.current;
                    void sourceImage.decode().then(() => {
                        if (!active.current || imageRef.current !== sourceImage || decodeSequence.current !== sequence) return;
                        setDecoded(true);
                        onLoad();
                    }, () => handleLoadError(sourceImage, sequence));
                }}
                onError={(event) => {
                    const sourceImage = event.currentTarget;
                    const sequence = ++decodeSequence.current;
                    handleLoadError(sourceImage, sequence);
                }}
                style={{
                    position: "absolute",
                    left: imageOffset.left,
                    top: imageOffset.top,
                    width: "100%",
                    height: "100%",
                    maxWidth: "none",
                    pointerEvents: "none",
                    userSelect: "none",
                    ...(clipPath ? {clipPath} : {}),
                    visibility: decoded ? "visible" : "hidden",
                }}
            />
        </Marker>
    );
}
