import {describe, it, expect} from 'vitest'
import {ESLint} from 'eslint'

// The guard for the failure mode #442 names explicitly: "a config that passes
// only because every real rule was disabled is a failure". `yarn lint` exits 0
// today, and these cases pin WHY — the rules are still on, the scoped
// exemptions are still scoped, and the plugin that once failed to resolve
// still resolves.
//
// It also catches the original breakage directly: before the flat-config
// migration ESLint 9 found no config at all and analysed zero files, which is
// exactly what `calculateConfigForFile` throwing/returning nothing looks like.

// Vitest's root is `gui/web` (vitest.config.ts lives there), so the process cwd
// is the directory holding eslint.config.js. Deliberately NOT derived from
// `import.meta.url`: Vite rewrites that to a `/@fs/...` URL inside the test
// runner, which is not a filesystem path ESLint can load a config from.
// Locally declared rather than pulling in @types/node, which this project does
// not install and which would change type resolution for the whole `src` tree.
declare const process: {cwd(): string}

const GUI_WEB_ROOT = process.cwd()

/** A representative first-party product file — not vendored, not generated. */
const FIRST_PARTY_FILE = 'src/pages/MapPage.tsx'

/** A hand-port of @mapbox/mapbox-gl-draw's untyped internal JS modes. */
const VENDORED_MODE_FILE = 'src/modes/DirectSelectWithBoxMode.tsx'

/** Machine-generated, DO-NOT-EDIT, and drift-gated in CI. */
const GENERATED_FILE = 'src/types/ros.generated.ts'

/**
 * Rules that must keep BLOCKING. Each one is a live-bug class that was actually
 * fixed while migrating the config, so a regression here means real defects
 * would ship silently.
 */
const BLOCKING_RULES = [
    '@typescript-eslint/no-base-to-string',
    '@typescript-eslint/no-non-null-asserted-optional-chain',
    '@typescript-eslint/no-unsafe-enum-comparison',
    '@typescript-eslint/restrict-plus-operands',
    '@typescript-eslint/prefer-promise-reject-errors',
    '@typescript-eslint/no-unnecessary-type-assertion',
    'react-hooks/rules-of-hooks',
    'no-prototype-builtins',
    'no-control-regex',
]

/**
 * The debt ratchet (overlay C). These are DEMOTED, never disabled — a future
 * "quick fix" that switches one of them off fails this test.
 */
const RATCHETED_RULES = [
    '@typescript-eslint/no-explicit-any',
    '@typescript-eslint/no-unsafe-assignment',
    '@typescript-eslint/no-unsafe-member-access',
    '@typescript-eslint/no-unsafe-call',
    '@typescript-eslint/no-unsafe-argument',
    '@typescript-eslint/no-unsafe-return',
    '@typescript-eslint/no-floating-promises',
    '@typescript-eslint/no-misused-promises',
]

/** ESLint reports severities as 0 (off) / 1 (warn) / 2 (error). */
const OFF = 0
const WARN = 1
const ERROR = 2

type RuleEntry = number | string | unknown[]

type ResolvedConfig = {rules?: Record<string, RuleEntry>}

/**
 * The `eslint` package's own type declarations depend on @types/node, which
 * this project does not install, so `ESLint` resolves to an error type here.
 * Construct it in exactly one place so the untyped boundary stays a single
 * documented line rather than being sprinkled through the cases.
 */
function createLinter(): ESLint {
    return new ESLint({cwd: GUI_WEB_ROOT})
}

function severityOf(entry: RuleEntry | undefined): number | undefined {
    if (entry === undefined) return undefined
    const raw = Array.isArray(entry) ? entry[0] : entry
    if (typeof raw === 'number') return raw
    if (raw === 'off') return OFF
    if (raw === 'warn') return WARN
    if (raw === 'error') return ERROR
    return undefined
}

async function rulesFor(relativePath: string): Promise<Record<string, RuleEntry>> {
    const config = (await createLinter().calculateConfigForFile(relativePath)) as ResolvedConfig
    return config.rules ?? {}
}

describe('eslint flat config', () => {
    it('resolves a config for first-party source instead of failing to find one', async () => {
        // Arrange / Act
        const rules = await rulesFor(FIRST_PARTY_FILE)

        // Assert
        expect(Object.keys(rules).length).toBeGreaterThan(0)
    })

    it('keeps the high-signal bug-class rules blocking for first-party code', async () => {
        // Arrange / Act
        const rules = await rulesFor(FIRST_PARTY_FILE)

        // Assert
        for (const rule of BLOCKING_RULES) {
            expect(severityOf(rules[rule]), `${rule} must stay an error`).toBe(ERROR)
        }
    })

    it('keeps react-hooks/exhaustive-deps reported as a warning', async () => {
        // Arrange / Act
        const rules = await rulesFor(FIRST_PARTY_FILE)

        // Assert — driving these to zero changes effect timing in map/settings
        // hooks, so they stay reported-but-not-blocking on purpose.
        expect(severityOf(rules['react-hooks/exhaustive-deps'])).toBe(WARN)
    })

    it('demotes the debt-ratchet rules to warnings without ever disabling them', async () => {
        // Arrange / Act
        const rules = await rulesFor(FIRST_PARTY_FILE)

        // Assert
        for (const rule of RATCHETED_RULES) {
            expect(severityOf(rules[rule]), `${rule} must be warn, not off`).toBe(WARN)
        }
    })

    it('resolves the ESM-only react-refresh plugin', async () => {
        // Arrange / Act
        const rules = await rulesFor(FIRST_PARTY_FILE)

        // Assert — under the old .eslintrc CJS loader this rule failed to
        // resolve, producing one phantom "Definition for rule ... was not
        // found" error in every single linted file.
        expect(rules['react-refresh/only-export-components']).toBeDefined()
        expect(severityOf(rules['react-refresh/only-export-components'])).toBe(WARN)
    })

    it('applies the untyped-boundary exemption only to the vendored mapbox-gl-draw ports', async () => {
        // Arrange
        const unsafeFamily = RATCHETED_RULES.filter((rule) => rule.includes('unsafe') || rule.includes('explicit-any'))

        // Act
        const vendored = await rulesFor(VENDORED_MODE_FILE)
        const firstParty = await rulesFor(FIRST_PARTY_FILE)

        // Assert — off for the vendored ports, still reported everywhere else.
        for (const rule of unsafeFamily) {
            expect(severityOf(vendored[rule]), `${rule} is exempt in src/modes`).toBe(OFF)
            expect(severityOf(firstParty[rule]), `${rule} stays on outside src/modes`).toBe(WARN)
        }
    })

    it('keeps rules-of-hooks blocking even inside the vendored mapbox-gl-draw ports', async () => {
        // Arrange / Act
        const vendored = await rulesFor(VENDORED_MODE_FILE)

        // Assert — the exemption covers the untyped API surface only.
        expect(severityOf(vendored['react-hooks/rules-of-hooks'])).toBe(ERROR)
    })

    it('ignores the generated ROS type bindings rather than linting them', async () => {
        // Arrange / Act
        const ignored = await createLinter().isPathIgnored(GENERATED_FILE)

        // Assert — hand-edits here are erased by generate_ts_types.sh AND fail
        // the .github/workflows/msg-codegen-drift.yml diff gate.
        expect(ignored).toBe(true)
    })
})
