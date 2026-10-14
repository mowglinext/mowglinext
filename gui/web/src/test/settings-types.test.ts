import {describe, it, expect} from 'vitest'
import schemaSource from '../../../asserts/mower_config.schema.json?raw'
import generated from '../types/settings.generated.ts?raw'
import {renderSettingsTypes} from '../../scripts/gen-settings-types.mjs'

// Anti-drift guard for the generated settings types: src/types/settings.generated.ts is rendered from
// the settings JSON schema, so a new or retyped schema key must be followed by
//   cd gui/web && npm run generate:settings-types
//
// The schema is read with `?raw` rather than imported as JSON: the GUI image builds with only gui/web
// in its context, where gui/asserts does not exist, and `tsc` has to compile this file there too. A
// `?raw` import is typed by vite/client and never resolved by tsc; vitest (run from the repo) reads it.
type Schema = {properties: Record<string, {properties: Record<string, unknown>}>}
const schema = JSON.parse(schemaSource) as Schema

describe('settings.generated.ts', () => {
    it('matches the settings JSON schema', () => {
        expect(generated.replace(/\r\n/g, '\n')).toBe(renderSettingsTypes(schema))
    })

    it('types the NTRIP keys the schema does not describe', () => {
        for (const key of ['ntrip_host', 'ntrip_port', 'ntrip_user', 'ntrip_password', 'ntrip_mountpoint']) {
            expect(generated).toContain(`    ${key}?:`)
        }
    })

    it('types every schema key', () => {
        const keys = Object.values(schema.properties).flatMap((group) => Object.keys(group.properties))
        expect(keys.length).toBeGreaterThan(100)
        for (const key of keys) {
            expect(generated).toContain(`    ${key}?:`)
        }
    })
})
