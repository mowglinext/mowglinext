
import type {BBox, Feature, Polygon, Point, Position, LineString} from 'geojson';
import {MapArea, Point32} from "../types/ros.ts";

import {transpose} from "../utils/map.tsx";

export class MowingFeature implements Feature {
    id: string;
    type: 'Feature';
    geometry: Polygon | Point | LineString;
    properties: Record<string, unknown>;

    constructor(id: string) {
        this.type = 'Feature';
        this.id = id;
        this.geometry = {type: 'Point', coordinates: [0, 0]};
        this.properties = {};
    }
}

export class PointFeatureBase extends MowingFeature implements Feature<Point>  {

    geometry: Point;
    properties: {
        color: string,
        feature_type: string
    }

    constructor(id: string, coordinate: Position, feature_type:string) {
        super(id);
        
        this.properties = { 
            color : 'black',
            feature_type: feature_type
        };
        this.geometry = {type:'Point', coordinates: coordinate} as Point;
    }

    setColor(color:string) {
        this.properties.color = color;
    }
}

export class LineFeatureBase extends MowingFeature implements Feature<LineString>  {

    geometry: LineString;
    properties: {
        color: string,
        width: number,
        feature_type: string
    }

    constructor(id: string, coordinates: Position[], color: string, feature_type:string) {
        super(id);
        
        this.properties = { 
            color : color,
            width : 1,
            feature_type: feature_type
        };
        this.geometry = {type:'LineString', coordinates: coordinates} as LineString;
    }
}

export class PathFeature extends LineFeatureBase {
    constructor(id: string, coordinates: Position[], color: string, lineWidth = 1) {
        super(id, coordinates,color, 'path');
        this.properties.width = lineWidth;
    }
}

export class ActivePathFeature extends LineFeatureBase {
    constructor(id: string, coordinates: Position[]) {
        super(id, coordinates, 'orange', 'active_path');
        this.properties.width = 3;
    }
}

export class MowerFeatureBase extends PointFeatureBase  {
    constructor(coordinate: Position) {
        super('mower', coordinate,'mower');
        this.setColor('#00a6ff');
    }
}

// One piece of the URDF-derived robot silhouette (chassis / wheel / blade).
// All parts share feature_type 'mower-footprint' so they reuse the existing
// fill + outline map layers; `color` is per-part (data-driven fill) so the
// wheels and blade read distinctly from the chassis.
export class RobotPartFeature extends MowingFeature implements Feature<Polygon> {
    declare geometry: Polygon;
    constructor(id: string, ring: Position[], color: string) {
        super(id);
        this.geometry = { type: 'Polygon', coordinates: [ring] };
        this.properties = { color, feature_type: 'mower-footprint' };
    }
}

export class DockFeatureBase extends PointFeatureBase  {
    declare properties: {
        color: string;
        feature_type: string;
        heading: number;
    };

    constructor(coordinate: Position, heading = 0) {
        super('dock', coordinate,'dock');
        this.properties.heading = heading;
        this.setColor('#ff00f2');
    }

    getHeading(): number {
        return this.properties.heading ?? 0;
    }

    setHeading(heading: number) {
        this.properties.heading = heading;
    }

    getCoordinates(): Position {
        return this.geometry.coordinates;
    }

    setCoordinates(coordinate: Position) {
        this.geometry.coordinates = coordinate;
    }
}


export class MowingFeatureBase extends MowingFeature implements Feature<Polygon> {
    geometry: Polygon;

    properties: {
        color: string
        , name? :string
        , index: number
        , mowing_order: number
        , feature_type: string
    }
    bbox?: BBox | undefined;

    
    constructor(id: string, feature_type: string) {
        super(id)
        this.type = 'Feature';
        this.properties = { 
            color : 'black'
            , index : 0
            , mowing_order:9999
            , feature_type: feature_type
        };
        this.geometry = {type:'Polygon', coordinates:[]} as Polygon;
    }

    setGeometry(geometry: Polygon) {
        this.geometry = geometry;
    }

    transpose( points: Point32[], offsetX: number, offsetY: number, datum: [number,number,number]) {
        this.geometry.coordinates = [points.map((point) => {
            return transpose(offsetX, offsetY, datum, point.y||0, point.x||0)
        })];
    }


    
    setColor(color: string) : MowingFeatureBase {
        this.properties.color = color;
        return this;
    }
}


export class ObstacleFeature extends MowingFeatureBase {
    mowing_area: MowingAreaFeature;

    constructor(id: string, mowing_area: MowingAreaFeature) {
        super(id, 'obstacle');
        this.setColor("#bf0000");
        this.mowing_area = mowing_area;
    }

    getMowingArea() : MowingAreaFeature {
        return this.mowing_area;
    }

}

export class MapAreaFeature extends MowingFeatureBase {
    area?: MapArea;

    constructor(id: string, feature_type: string) {
        super(id, feature_type);
    }

    setArea( area: MapArea, offsetX: number, offsetY: number, datum: [number,number,number]) {
        this.area = area;
        this.transpose(area.area?.points??[], offsetX, offsetY, datum);
    }


    getArea(): MapArea | undefined {
        return this.area;
    }
}


export class NavigationFeature extends MapAreaFeature {
    constructor(id: string) {
        super(id, 'navigation');
        this.setColor("white");
    }
}

export class MowingAreaFeature extends MapAreaFeature {

    //mowing_order: number;
  
    
    constructor(id: string, mowing_order: number ) {
        super(id, 'workarea');
        this.properties.mowing_order = mowing_order;
    
        this.setName('');
        this.setColor("#01d30d");

    }
    
    setArea( area: MapArea, offsetX: number , offsetY: number, datum: [number,number,number]  ) {
        super.setArea(area, offsetX, offsetY, datum);
        this.setName(area.name ?? '')
    }


    setName(name: string) : MowingAreaFeature {
        this.properties['name'] = name;
        if (this.area)
            this.area.name = name;
        return this;
    }

    getName() : string {
        return this.properties?.name ?  this.properties?.name : '';
    }


    getMowingOrder() : number {
        return this.properties.mowing_order;
    }

    setMowingOrder(val: number) : MowingAreaFeature{
        this.properties.mowing_order = val;
        return this;
    }

    getIndex() : number {
        return this.properties.mowing_order-1;
    }

    /**
     * Human-readable label. Callers should pass `unnamedLabel` (already
     * translated, e.g. t('mapAreasList.unnamedArea', {order})) so the
     * fallback for unnamed areas is localised; without it the English
     * "Area N" is used.
     */
    getLabel(unnamedLabel?: string) : string {
        const name = this.getName();
        if (name) return name + " (" + this.getMowingOrder().toString() + ")";
        return unnamedLabel ?? "Area " + this.getMowingOrder().toString();
    }


}

// ---------------------------------------------------------------------------
// Serialization / rehydration
//
// History snapshots (undo/redo) and GeoJSON import must NOT store the class
// instances directly: structuredClone strips prototypes, so a restored
// snapshot fails every `instanceof` check and the editor silently drops all
// polygons (Save would then persist an EMPTY map). Instead we snapshot plain
// serializable data and rebuild real class instances through this factory.
// ---------------------------------------------------------------------------

/** Plain, structured-clone-safe snapshot of a MowingFeature. */
export interface SerializedMapFeature {
    id: string;
    type: 'Feature';
    geometry: Polygon | Point | LineString;
    properties: Record<string, unknown>;
    /** ObstacleFeature only: id of the parent MowingAreaFeature. */
    parent_id?: string;
    /** MapAreaFeature only: original ROS MapArea payload. */
    area?: MapArea;
}

/** Snapshot a single feature into plain serializable data. */
export function serializeFeature(feature: MowingFeature): SerializedMapFeature {
    const json: SerializedMapFeature = {
        id: feature.id,
        type: 'Feature',
        geometry: structuredClone(feature.geometry),
        properties: structuredClone(feature.properties),
    };
    if (feature instanceof ObstacleFeature) {
        json.parent_id = feature.getMowingArea()?.id;
    }
    if (feature instanceof MapAreaFeature && feature.area) {
        json.area = structuredClone(feature.area);
    }
    return json;
}

/** Snapshot a whole feature map into plain serializable data. */
export function serializeFeatures(
    features: Record<string, MowingFeature>
): Record<string, SerializedMapFeature> {
    const result: Record<string, SerializedMapFeature> = {};
    for (const [id, feature] of Object.entries(features)) {
        result[id] = serializeFeature(feature);
    }
    return result;
}

/**
 * Rebuild a proper class instance from a plain snapshot. Obstacles need
 * their (already rehydrated) `parent` MowingAreaFeature; returns null when
 * an obstacle has no parent to attach to. Unknown feature types come back
 * as a generic MowingFeature carrying geometry + properties (enough for the
 * display-only layers, which never use instanceof on them).
 */
export function featureFromJSON(
    json: SerializedMapFeature,
    parent?: MowingAreaFeature
): MowingFeature | null {
    const props = structuredClone(json.properties ?? {});
    const featureType = props.feature_type as string | undefined;

    switch (featureType) {
        case 'workarea': {
            const area = new MowingAreaFeature(json.id, (props.mowing_order as number) ?? 9999);
            area.setGeometry(json.geometry as Polygon);
            area.properties = {...area.properties, ...props} as MowingAreaFeature['properties'];
            if (json.area) area.area = structuredClone(json.area);
            return area;
        }
        case 'navigation': {
            const nav = new NavigationFeature(json.id);
            nav.setGeometry(json.geometry as Polygon);
            nav.properties = {...nav.properties, ...props} as NavigationFeature['properties'];
            if (json.area) nav.area = structuredClone(json.area);
            return nav;
        }
        case 'obstacle': {
            if (!parent) return null;
            const obstacle = new ObstacleFeature(json.id, parent);
            obstacle.setGeometry(json.geometry as Polygon);
            obstacle.properties = {...obstacle.properties, ...props} as ObstacleFeature['properties'];
            return obstacle;
        }
        case 'dock': {
            const coords = (json.geometry as Point).coordinates;
            return new DockFeatureBase(coords, (props.heading as number) ?? 0);
        }
        default: {
            const feature = new MowingFeature(json.id);
            feature.geometry = structuredClone(json.geometry);
            feature.properties = props;
            return feature;
        }
    }
}

/**
 * Rebuild a whole feature map: areas first, then obstacles wired to their
 * rehydrated parents (by parent_id, falling back to the first area so a
 * snapshot with a broken link still restores the polygon).
 */
export function featuresFromJSON(
    snapshot: Record<string, SerializedMapFeature>
): Record<string, MowingFeature> {
    const result: Record<string, MowingFeature> = {};
    const parents: Record<string, MowingAreaFeature> = {};

    for (const [id, json] of Object.entries(snapshot)) {
        if (json.properties?.feature_type === 'obstacle') continue;
        const feature = featureFromJSON(json);
        if (!feature) continue;
        result[id] = feature;
        if (feature instanceof MowingAreaFeature) parents[id] = feature;
    }

    for (const [id, json] of Object.entries(snapshot)) {
        if (json.properties?.feature_type !== 'obstacle') continue;
        const parent =
            (json.parent_id ? parents[json.parent_id] : undefined) ??
            Object.values(parents)[0];
        const feature = featureFromJSON(json, parent);
        if (feature) result[id] = feature;
    }

    return result;
}

/**
 * Prototype-preserving shallow clone with fresh properties + geometry.
 * Use inside setState updaters instead of mutating an existing instance in
 * place (in-place mutation breaks under React StrictMode double-invoke and
 * retroactively corrupts anything still holding the old reference).
 */
export function cloneFeature<T extends MowingFeature>(feature: T): T {
    const copy = Object.create(Object.getPrototypeOf(feature)) as T;
    Object.assign(copy, feature);
    copy.properties = {...feature.properties};
    copy.geometry = structuredClone(feature.geometry);
    return copy;
}

