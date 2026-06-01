
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

export class MowerFootprintFeature extends MowingFeature implements Feature<Polygon> {
    declare geometry: Polygon;
    constructor(ring: Position[]) {
        super('mower-footprint');
        this.geometry = { type: 'Polygon', coordinates: [ring] };
        this.properties = { color: '#00a6ff', feature_type: 'mower-footprint' };
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

    getLabel() : string {
        const name = this.getName();
        return name ? name  + " (" + this.getMowingOrder().toString() +")" : "Area " + this.getMowingOrder().toString();
    }


}

