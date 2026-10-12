export interface ProjectedPoint {
    x: number;
    y: number;
}

export type ProjectLngLat = (coordinate: [number, number]) => ProjectedPoint;
export interface MapImageDimensionsPx {
    width: number;
    height: number;
}

/** Convert the existing mower-heading LineString into ROS ENU yaw. */
export function getMowerHeadingRad(
    coordinates: ReadonlyArray<readonly number[]>,
    toRosXY: (longitude: number, latitude: number) => readonly [number, number],
): number | undefined {
    if (coordinates.length < 2) return undefined;
    const [startLongitude, startLatitude] = coordinates[0];
    const [endLongitude, endLatitude] = coordinates[1];
    if (![startLongitude, startLatitude, endLongitude, endLatitude].every(Number.isFinite)) return undefined;
    const [startX, startY] = toRosXY(startLongitude, startLatitude);
    const [endX, endY] = toRosXY(endLongitude, endLatitude);
    const deltaX = endX - startX;
    const deltaY = endY - startY;
    if (![deltaX, deltaY].every(Number.isFinite) || (deltaX === 0 && deltaY === 0)) return undefined;
    return Math.atan2(deltaY, deltaX);
}

export function hasValidMapPosition(coordinates: readonly number[]): boolean {
    const [longitude, latitude] = coordinates;
    return Number.isFinite(longitude) && longitude >= -180 && longitude <= 180 &&
        Number.isFinite(latitude) && latitude >= -90 && latitude <= 90;
}

/** Shift a display marker a short distance along ROS ENU yaw without changing
 * the tracked pose data. Distances are intended for image alignment only.
 */
export function offsetMapCoordinatesForward(
    longitude: number,
    latitude: number,
    headingRad: number,
    offsetM: number,
): [number, number] | undefined {
    if (
        !hasValidMapPosition([longitude, latitude]) ||
        !Number.isFinite(headingRad) ||
        !Number.isFinite(offsetM)
    ) return undefined;

    const cosLatitude = Math.cos(latitude * Math.PI / 180);
    if (Math.abs(cosLatitude) < MIN_COS_LATITUDE) return undefined;

    const eastM = offsetM * Math.cos(headingRad);
    const northM = offsetM * Math.sin(headingRad);
    const shiftedLongitude = longitude + eastM / (EARTH_RADIUS_M * cosLatitude) * (180 / Math.PI);
    const shiftedLatitude = latitude + northM / EARTH_RADIUS_M * (180 / Math.PI);
    return hasValidMapPosition([shiftedLongitude, shiftedLatitude])
        ? [shiftedLongitude, shiftedLatitude]
        : undefined;
}

const EARTH_RADIUS_M = 6_378_137;
const SCALE_PROBE_M = 0.5;
const MIN_COS_LATITUDE = 1e-6;

/**
 * Return the square image size that makes its calibrated long axis represent
 * `visibleLengthM` at this map pose. Mapbox's map-aligned marker transform
 * supplies bearing and pitch foreshortening after this local Mercator scale.
 */
export function calculateMapImageDimensionsPx(
    project: ProjectLngLat,
    longitude: number,
    latitude: number,
    visibleLengthM: number,
    visibleLengthFraction: number,
    visibleWidthM?: number,
    visibleWidthFraction?: number,
): MapImageDimensionsPx | undefined {
    if (
        !Number.isFinite(longitude) || longitude < -180 || longitude > 180 ||
        !Number.isFinite(latitude) || latitude < -90 || latitude > 90 ||
        !Number.isFinite(visibleLengthM) || visibleLengthM <= 0 ||
        !Number.isFinite(visibleLengthFraction) || visibleLengthFraction <= 0 || visibleLengthFraction > 1
    ) return undefined;

    const cosLatitude = Math.cos(latitude * Math.PI / 180);
    if (Math.abs(cosLatitude) < MIN_COS_LATITUDE) return undefined;

    const probeLongitude = longitude +
        (SCALE_PROBE_M / (EARTH_RADIUS_M * cosLatitude)) * (180 / Math.PI);
    const origin = project([longitude, latitude]);
    const eastProbe = project([probeLongitude, latitude]);
    const pixelsPerMeter = Math.hypot(eastProbe.x - origin.x, eastProbe.y - origin.y) / SCALE_PROBE_M;
    const height = pixelsPerMeter * visibleLengthM / visibleLengthFraction;
    const hasWidthCalibration = visibleWidthM !== undefined || visibleWidthFraction !== undefined;
    if (hasWidthCalibration && (
        !Number.isFinite(visibleWidthM) || visibleWidthM! <= 0 ||
        !Number.isFinite(visibleWidthFraction) || visibleWidthFraction! <= 0 || visibleWidthFraction! > 1
    )) return undefined;
    const width = hasWidthCalibration
        ? pixelsPerMeter * visibleWidthM! / visibleWidthFraction!
        : height;
    return Number.isFinite(width) && width > 0 && Number.isFinite(height) && height > 0
        ? {width, height}
        : undefined;
}

/** Compatibility helper for callers interested in the long-axis image size. */
export function calculateMapImageSizePx(
    project: ProjectLngLat,
    longitude: number,
    latitude: number,
    visibleLengthM: number,
    visibleLengthFraction: number,
): number | undefined {
    return calculateMapImageDimensionsPx(project, longitude, latitude, visibleLengthM, visibleLengthFraction)?.height;
}

/** Offset an image of any aspect ratio so its normalized pose anchor sits at
 * the marker's center. Mapbox rotates the marker about that center, keeping
 * the pose fixed.
 */
export function getMapImageAnchorOffsetPx(
    widthPx: number,
    heightPx: number,
    poseAnchor: {x: number; y: number},
): {left: number; top: number} {
    return {
        left: widthPx * (0.5 - poseAnchor.x),
        top: heightPx * (0.5 - poseAnchor.y),
    };
}

/** ROS yaw is CCW from east; Mapbox marker rotation is clockwise from north. */
export function rosHeadingToMapboxRotation(headingRad: number): number {
    return 90 - headingRad * (180 / Math.PI);
}
