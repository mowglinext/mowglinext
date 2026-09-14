import {describe, expect, it} from 'vitest';
import {deriveIsMoving} from './mowerMotion.ts';

describe('deriveIsMoving', () => {
    it('reports motion for every self-propelled state when not charging', () => {
        expect(deriveIsMoving(2, false)).toBe(true);
        expect(deriveIsMoving(3, false)).toBe(true);
        expect(deriveIsMoving(4, false)).toBe(true);
    });

    it('reports no motion for docked, idle and unknown states', () => {
        expect(deriveIsMoving(0, false)).toBe(false);
        expect(deriveIsMoving(1, false)).toBe(false);
        expect(deriveIsMoving(-1, false)).toBe(false);
        expect(deriveIsMoving(undefined, false)).toBe(false);
        expect(deriveIsMoving(null, false)).toBe(false);
    });

    // Precedence regression. The inline expression this helper replaced was
    //   stateNum === 2 || stateNum === 3 || stateNum === 4 && !isCharging
    // and && binds tighter than ||, so !isCharging only ever qualified state
    // 4. States 2 and 3 therefore read as "moving" on the charger.
    it('reports no motion while charging, for state 2 as well as 4', () => {
        expect(deriveIsMoving(2, true)).toBe(false);
        expect(deriveIsMoving(3, true)).toBe(false);
        expect(deriveIsMoving(4, true)).toBe(false);
    });

    it('treats a missing charging flag as not charging', () => {
        expect(deriveIsMoving(2, undefined)).toBe(true);
        expect(deriveIsMoving(2, null)).toBe(true);
    });
});
