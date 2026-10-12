import {describe, expect, it} from 'vitest';
import i18n from '../i18n';
import {matchesSettingSearch, settingSearchText} from './settingsSearch.ts';

describe('localized settings search', () => {
    it.each([['fr', 'largeur de coupe'], ['en', 'tool width'], ['fr', 'TOOL_WIDTH']])('%s: %s finds the cutting width', (lang, query) => {
        expect(matchesSettingSearch(query, 'tool_width', ...settingSearchText('tool_width', i18n.getFixedT(lang)))).toBe(true);
    });
    it('normalizes accents, separators, case and word order', () => {
        expect(matchesSettingSearch('REGLAGE batterie', 'Batterie — réglage')).toBe(true);
        expect(matchesSettingSearch('compass', 'wheel radius')).toBe(false);
    });
});
