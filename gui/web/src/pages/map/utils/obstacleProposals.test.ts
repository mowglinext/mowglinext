import {describe, expect, it} from 'vitest';
import type {Map} from '../../../types/ros.ts';
import {MapObstacleInfoConstants} from '../../../types/ros.ts';
import {extractObstacleProposals, isDigProposal} from './obstacleProposals.ts';

const square = (x: number, y: number) => ({
    points: [{x, y, z: 0}, {x: x + 0.6, y, z: 0}, {x: x + 0.6, y: y + 0.6, z: 0}, {x, y: y + 0.6, z: 0}],
});

describe('extractObstacleProposals', () => {
    it('returns nothing for a map without proposals', () => {
        const map: Map = {working_area: [{name: 'lawn', obstacles: [square(0, 0)]}], working_area_indices: [0]};

        expect(extractObstacleProposals(map)).toEqual([]);
        expect(extractObstacleProposals(undefined)).toEqual([]);
    });

    it('lists a dig proposal with the ROS area index, never the list position', () => {
        // Arrange: area 0 of map_server is a navigation area, so the first
        // WORKING area is ROS area 1.
        const map: Map = {
            working_area: [{
                name: 'back lawn',
                proposed_obstacles: [square(-3.2, 11.0)],
                proposed_obstacle_info: [{
                    id: 7, name: 'Dig at (-3.23, 11.01): wheels 0.45 m vs pose 0.02 m',
                    source: MapObstacleInfoConstants.SOURCE_DIG, pending: true,
                }],
            }],
            working_area_indices: [1],
        };

        // Act
        const proposals = extractObstacleProposals(map);

        // Assert
        expect(proposals).toHaveLength(1);
        expect(proposals[0]).toMatchObject({id: 7, areaIndex: 1, areaName: 'back lawn'});
        expect(proposals[0].name).toContain('Dig at');
        expect(isDigProposal(proposals[0])).toBe(true);
    });

    it('drops entries that could not be accepted or rejected', () => {
        const map: Map = {
            working_area: [{
                name: 'lawn',
                proposed_obstacles: [square(0, 0), {points: [{x: 0, y: 0, z: 0}]}, square(2, 2)],
                proposed_obstacle_info: [
                    {id: 0, name: 'no handle', source: MapObstacleInfoConstants.SOURCE_DIG},
                    {id: 3, name: 'degenerate polygon', source: MapObstacleInfoConstants.SOURCE_DIG},
                    {id: 4, name: 'ok', source: MapObstacleInfoConstants.SOURCE_DIG},
                ],
            }],
            working_area_indices: [0],
        };

        expect(extractObstacleProposals(map).map(p => p.id)).toEqual([4]);
    });

    it('skips an area whose ROS index is unknown', () => {
        const map: Map = {
            working_area: [{
                name: 'restored, not saved yet',
                proposed_obstacles: [square(0, 0)],
                proposed_obstacle_info: [{id: 9, source: MapObstacleInfoConstants.SOURCE_DIG}],
            }],
            working_area_indices: [],
        };

        expect(extractObstacleProposals(map)).toEqual([]);
    });
});
