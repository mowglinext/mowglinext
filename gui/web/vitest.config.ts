import {defineConfig} from 'vitest/config'
import react from '@vitejs/plugin-react'

export default defineConfig({
    plugins: [react()],
    test: {
        environment: 'jsdom',
        globals: true,
        setupFiles: ['./src/test/setup.ts'],
        css: true,
        // Vitest's 5s default is too tight for the antd-portal tests. A
        // Select/Dropdown test mounts into a portal and drives rc-virtual-list
        // through userEvent, which costs ~800ms on an idle machine — but the
        // budget is WALL-CLOCK, so it also absorbs however long this worker
        // spends contending for CPU with the others. Under a full parallel run
        // GnssSerialDeviceConfigField.test.tsx intermittently crossed 5s (~2
        // runs in 8) and failed with "Test timed out", a false negative about
        // the machine rather than the component.
        //
        // Raise the ceiling rather than chase per-call timeouts: note that
        // Testing Library's own asyncUtilTimeout must stay BELOW this, so a
        // findBy* that genuinely never matches still reports "unable to find
        // element" instead of being cut off by this outer timeout.
        testTimeout: 20000,
        // Playwright e2e specs live here and import @playwright/test — keep them
        // out of the vitest (unit) run. The appearance registry suite imports
        // node:test and runs separately via test:appearances in `npm test`.
        exclude: ['node_modules', 'dist', 'tests/e2e/**', 'scripts/appearance-registry.test.mjs'],
    },
})
