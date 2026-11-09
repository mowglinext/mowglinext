import {describe, expect, it} from 'vitest';
import {assertSupportedMapPolygon, projectMapPolygon} from './mapGeometryContract';
import {transpose} from '../../../utils/map';

const polygon = (coordinates: unknown) => ({type: 'Polygon', coordinates});
const square = [[0, 0], [1, 0], [1, 1], [0, 1], [0, 0]];

describe('lossless simple polygon boundary', () => {
    it.each([square, square.slice(0, -1), [...square].reverse()].map(ring => ({ring})))('accepts existing open/closed rings and either winding without modifying them', ({ring}) => {
        const source = polygon([ring]), before = structuredClone(source);
        expect(() => assertSupportedMapPolygon(source)).not.toThrow();
        expect(source).toEqual(before);
    });
    it.each([
        [], [[]], [[[0, 0], [1, 0]]], [[[0, 0], [0, 0], [0, 0]]],
        [[[0, 0], [1, 0], [2, 0]]], [[[0, 0], [1, 1], [0, 1], [1, 0], [0, 0]]],
        [[[0, 0], [1, 0], [1, 1], [1, 0], [0, 1], [0, 0]]],
        [[[0, 0], [NaN, 0], [0, 1]]], [[[0, 0], [Infinity, 0], [0, 1]]],
        [[[0, 0], ['1', 0], [0, 1]]], [[[181, 0], [182, 0], [181, 1]]],
        [[[0, 91], [1, 91], [0, 92]]],
    ].map(coordinates => ({coordinates})))('rejects invalid outer rings %j', ({coordinates}) => {
        expect(() => assertSupportedMapPolygon(polygon(coordinates))).toThrow('invalid');
    });
    it.each([
        [square, square], [square, []], [square, [[.2, .2], [.3, .2]]],
        [square, [[2, 2], [3, 2], [3, 3], [2, 2]]],
        [square, [[.5, .5], [2, .5], [2, 2], [.5, .5]]],
        [square, [[.2, .2], [.8, .8], [.2, .8], [.8, .2], [.2, .2]]],
    ].map(coordinates => ({coordinates})))('refuses all interior rings, including invalid/crossing/outside holes, rather than rewriting them', ({coordinates}) => {
        expect(() => assertSupportedMapPolygon(polygon(coordinates))).toThrow('unsupported');
    });
    it('refuses MultiPolygon without flattening it', () => {
        expect(() => assertSupportedMapPolygon({type: 'MultiPolygon', coordinates: [[square]]})).toThrow('unsupported');
    });
    it('preserves the current projection, origin and winding for a valid map polygon', () => {
        const ring = [[0, 0], [4, 0], [4, 3], [0, 3], [0, 0]].map(([x, y]) => transpose(0, 0, [0, 0, 0], y, x));
        const points = projectMapPolygon(polygon([ring]), 0, 0, [0, 0, 0]);
        expect(points).toHaveLength(5);
        expect(points[0]).toEqual({x: 0, y: 0, z: 0});
        expect(points[1].x).toBeCloseTo(4, 12); expect(points[2].y).toBeCloseTo(3, 12);
        expect(points[4]).toEqual(points[0]);
    });
    it('rejects collapse after the existing millimetre deduplication', () => {
        const ring = [[0, 0], [.0001, 0], [0, .0001], [0, 0]].map(([x, y]) => transpose(0, 0, [0, 0, 0], y, x));
        expect(() => projectMapPolygon(polygon([ring]), 0, 0, [0, 0, 0])).toThrow('invalid');
    });
    it.each([
        {offset: NaN, datum: [0, 0, 0]}, {offset: 0, datum: [Infinity, 0, 0]},
        {offset: 0, datum: [90, 0, 0]}, {offset: 0, datum: [0, 181, 0]},
    ])('rejects an invalid projection frame before preparing a write: %j', ({offset, datum}) => {
        expect(() => projectMapPolygon(polygon([square]), offset, 0, [datum[0], datum[1], datum[2]])).toThrow('invalid');
    });
    it('rejects a local frame outside the established 100 km bound', () => {
        expect(() => projectMapPolygon(polygon([square]), 0, 0, [0, 0, 0])).toThrow('invalid');
    });
    it('rejects an edge that collapses only in ROS float32 precision', () => {
        const ring = [[90000, 90000], [90000.002, 90000], [90000, 90001], [90000, 90000]]
            .map(([x, y]) => transpose(0, 0, [0, 0, 0], y, x));
        expect(() => projectMapPolygon(polygon([ring]), 0, 0, [0, 0, 0])).toThrow('invalid');
    });
});
