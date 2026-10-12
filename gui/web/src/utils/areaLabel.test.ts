import {describe, expect, it} from 'vitest';
import i18n from '../i18n';
import {areaLabel, liveAreaLabel, formatArea} from './areaLabel.ts';

describe('area display labels', () => {
    const t = i18n.getFixedT('fr');
    it('uses a name verbatim or a one-based localized fallback', () => {
        expect(areaLabel(t, 1)).toBe('Zone 1');
        expect(areaLabel(t, 1, ' Potager ')).toBe('Potager');
    });
    it('resolves live ROS indices independently of stable identity and navigation areas', () => {
        expect(liveAreaLabel(t, {working_area: [{id: 91, name: 'Potager'}, {id: 7}], working_area_indices: [2, 4]}, 4)).toBe('Zone 2');
        expect(liveAreaLabel(t, {working_area: [{id: 7}, {id: 91, name: 'Potager'}], working_area_indices: [0, 3]}, 3)).toBe('Potager');
    });
    it('keeps small obstacles visible and formats using the selected locale', () => {
        expect(formatArea(0.03, 'fr')).toBe('< 1 m²');
        expect(formatArea(12345, 'fr')).toBe('1,23 ha');
    });
});
