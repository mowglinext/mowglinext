import {describe, it, expect} from 'vitest';
import {
    MowingFeature,
    PointFeatureBase,
    LineFeatureBase,
    PathFeature,
    ActivePathFeature,
    MowerFeatureBase,
    DockFeatureBase,
    MowingFeatureBase,
    ObstacleFeature,
    NavigationFeature,
    MowingAreaFeature,
    closeRing,
    serializeFeature,
    featuresFromJSON,
} from './map.ts';

describe('MowingFeature (base class)', () => {
    it('creates with correct id and type', () => {
        const f = new MowingFeature('test-1');
        expect(f.id).toBe('test-1');
        expect(f.type).toBe('Feature');
    });

    it('implements Feature interface with default geometry', () => {
        const f = new MowingFeature('f1');
        expect(f.geometry.type).toBe('Point');
        expect(f.properties).toEqual({});
    });
});

describe('PointFeatureBase', () => {
    it('creates point with coordinates and feature type', () => {
        const f = new PointFeatureBase('p1', [10, 20], 'mower');
        expect(f.geometry.type).toBe('Point');
        expect(f.geometry.coordinates).toEqual([10, 20]);
        expect(f.properties.feature_type).toBe('mower');
        expect(f.properties.color).toBe('black');
    });

    it('setColor updates color', () => {
        const f = new PointFeatureBase('p1', [0, 0], 'test');
        f.setColor('red');
        expect(f.properties.color).toBe('red');
    });
});

describe('LineFeatureBase', () => {
    it('creates line with coordinates and color', () => {
        const coords = [[0, 0], [1, 1], [2, 2]] as [number, number][];
        const f = new LineFeatureBase('l1', coords, 'blue', 'path');
        expect(f.geometry.type).toBe('LineString');
        expect(f.geometry.coordinates).toEqual(coords);
        expect(f.properties.color).toBe('blue');
        expect(f.properties.width).toBe(1);
        expect(f.properties.feature_type).toBe('path');
    });
});

describe('PathFeature', () => {
    it('creates with path feature type', () => {
        const f = new PathFeature('path1', [[0, 0]], 'green');
        expect(f.properties.feature_type).toBe('path');
        expect(f.properties.color).toBe('green');
        expect(f.properties.width).toBe(1);
    });

    it('accepts custom line width', () => {
        const f = new PathFeature('path1', [[0, 0]], 'green', 3);
        expect(f.properties.width).toBe(3);
    });
});

describe('ActivePathFeature', () => {
    it('creates with orange color and width 3', () => {
        const f = new ActivePathFeature('ap1', [[0, 0], [1, 1]]);
        expect(f.properties.color).toBe('orange');
        expect(f.properties.width).toBe(3);
        expect(f.properties.feature_type).toBe('active_path');
    });
});

describe('MowerFeatureBase', () => {
    it('creates at given position with mower type', () => {
        const f = new MowerFeatureBase([10, 20]);
        expect(f.id).toBe('mower');
        expect(f.geometry.coordinates).toEqual([10, 20]);
        expect(f.properties.feature_type).toBe('mower');
        expect(f.properties.color).toBe('#00a6ff');
    });
});

describe('DockFeatureBase', () => {
    it('creates at given position with dock type', () => {
        const f = new DockFeatureBase([5, 10]);
        expect(f.id).toBe('dock');
        expect(f.geometry.coordinates).toEqual([5, 10]);
        expect(f.properties.feature_type).toBe('dock');
        expect(f.properties.color).toBe('#ff00f2');
    });

    it('tracks whether the dock heading is actually available', () => {
        const missing = new DockFeatureBase([5, 10]);
        expect(missing.getHeading()).toBe(0);
        expect(missing.hasValidHeading()).toBe(false);
        const restored = featuresFromJSON({dock: serializeFeature(missing)}).dock;
        expect(restored).toBeInstanceOf(DockFeatureBase);
        expect((restored as DockFeatureBase).hasValidHeading()).toBe(false);

        const withHeading = new DockFeatureBase([5, 10], Math.PI / 2);
        expect(withHeading.hasValidHeading()).toBe(true);
        withHeading.setHeading(Number.NaN);
        expect(withHeading.hasValidHeading()).toBe(false);
    });
});

describe('MowingFeatureBase', () => {
    it('creates polygon with default properties', () => {
        const f = new MowingFeatureBase('area-0-area-0', 'workarea');
        expect(f.geometry.type).toBe('Polygon');
        expect(f.geometry.coordinates).toEqual([]);
        expect(f.properties.feature_type).toBe('workarea');
        expect(f.properties.mowing_order).toBe(9999);
        expect(f.properties.index).toBe(0);
    });

    it('setGeometry replaces geometry', () => {
        const f = new MowingFeatureBase('test', 'workarea');
        const poly = {type: 'Polygon' as const, coordinates: [[[0, 0], [1, 0], [1, 1], [0, 0]]]};
        f.setGeometry(poly);
        expect(f.geometry).toEqual(poly);
    });

    it('setColor returns self for chaining', () => {
        const f = new MowingFeatureBase('test', 'workarea');
        const result = f.setColor('red');
        expect(result).toBe(f);
        expect(f.properties.color).toBe('red');
    });
});

describe('MowingAreaFeature', () => {
    it('creates workarea with mowing order and green color', () => {
        const f = new MowingAreaFeature('area-0-area-0', 1);
        expect(f.properties.feature_type).toBe('workarea');
        expect(f.properties.mowing_order).toBe(1);
        expect(f.properties.color).toBe('#01d30d');
    });

    it('getName / setName', () => {
        const f = new MowingAreaFeature('area-0-area-0', 1);
        expect(f.getName()).toBe('');
        f.setName('Front Yard');
        expect(f.getName()).toBe('Front Yard');
    });

    it('getMowingOrder / setMowingOrder', () => {
        const f = new MowingAreaFeature('area-0-area-0', 3);
        expect(f.getMowingOrder()).toBe(3);
        f.setMowingOrder(5);
        expect(f.getMowingOrder()).toBe(5);
    });

    it('getIndex returns mowing_order - 1', () => {
        const f = new MowingAreaFeature('area-0-area-0', 3);
        expect(f.getIndex()).toBe(2);
    });

    it('getLabel returns name with order when named', () => {
        const f = new MowingAreaFeature('area-0-area-0', 2);
        f.setName('Garden');
        expect(f.getLabel()).toBe('Garden (2)');
    });

    it('getLabel returns "Area N" when unnamed', () => {
        const f = new MowingAreaFeature('area-0-area-0', 2);
        expect(f.getLabel()).toBe('Area 2');
    });

    it('setMowingOrder returns self for chaining', () => {
        const f = new MowingAreaFeature('area-0-area-0', 1);
        const result = f.setMowingOrder(5);
        expect(result).toBe(f);
    });

    it('setName returns self for chaining', () => {
        const f = new MowingAreaFeature('area-0-area-0', 1);
        const result = f.setName('test');
        expect(result).toBe(f);
    });
});

describe('NavigationFeature', () => {
    it('creates with white color and navigation type', () => {
        const f = new NavigationFeature('nav-0-area-0');
        expect(f.properties.feature_type).toBe('navigation');
        expect(f.properties.color).toBe('white');
    });
});

describe('ObstacleFeature', () => {
    it('creates linked to parent mowing area', () => {
        const parent = new MowingAreaFeature('area-0-area-0', 1);
        const obstacle = new ObstacleFeature('area-0-obstacle-0', parent);
        expect(obstacle.properties.feature_type).toBe('obstacle');
        expect(obstacle.properties.color).toBe('#bf0000');
        expect(obstacle.getMowingArea()).toBe(parent);
    });

    it('getMowingArea returns parent reference', () => {
        const parent = new MowingAreaFeature('area-0-area-0', 1);
        parent.setName('Garden');
        const obstacle = new ObstacleFeature('area-0-obstacle-0', parent);
        expect(obstacle.getMowingArea().getName()).toBe('Garden');
    });

    // Field report: obstacles generated by map_server (dig proposals, promoted
    // tracker obstacles) showed up as TRIANGLES. ROS polygons are unclosed;
    // mapbox-gl-draw drops the last position of every ring, assuming GeoJSON's
    // closing vertex — so a 4-vertex rectangle lost a corner.
    it('closes an unclosed ROS ring so no vertex is lost when it is drawn', () => {
        const parent = new MowingAreaFeature('area-0-area-0', 1);
        const obstacle = new ObstacleFeature('area-0-obstacle-0', parent);
        const rosRectangle = [{x: 0, y: 0, z: 0}, {x: 0.6, y: 0, z: 0}, {x: 0.6, y: 0.4, z: 0}, {x: 0, y: 0.4, z: 0}];

        obstacle.transpose(rosRectangle, 0, 0, [48.0, 2.0, 0]);

        const ring = obstacle.geometry.coordinates[0];
        expect(ring).toHaveLength(5);
        expect(ring[4]).toEqual(ring[0]);
        expect(new Set(ring.slice(0, -1).map(p => p.join(','))).size).toBe(4);
    });
});

describe('closeRing', () => {
    it('leaves an already closed ring (a polygon drawn in the GUI) untouched', () => {
        const closed = [[0, 0], [1, 0], [1, 1], [0, 0]];
        expect(closeRing(closed)).toBe(closed);
    });

    it('does not invent a ring out of fewer than three positions', () => {
        expect(closeRing([[0, 0], [1, 1]])).toEqual([[0, 0], [1, 1]]);
    });
});

describe('Feature ID conventions', () => {
    it('area features follow type-index-component-number pattern', () => {
        const area = new MowingAreaFeature('area-0-area-0', 1);
        expect(area.id).toBe('area-0-area-0');
    });

    it('navigation features follow pattern', () => {
        const nav = new NavigationFeature('navigation-0-area-0');
        expect(nav.id).toBe('navigation-0-area-0');
    });

    it('obstacle features follow pattern with parent index', () => {
        const parent = new MowingAreaFeature('area-0-area-0', 1);
        const obstacle = new ObstacleFeature('area-0-obstacle-0', parent);
        expect(obstacle.id).toBe('area-0-obstacle-0');
    });
});
