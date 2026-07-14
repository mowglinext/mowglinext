import {defineConfig} from 'vitest/config'
import react from '@vitejs/plugin-react'

export default defineConfig({
    plugins: [react()],
    test: {
        environment: 'jsdom',
        globals: true,
        setupFiles: ['./src/test/setup.ts'],
        css: true,
        // Playwright e2e specs live here and import @playwright/test — keep them
        // out of the vitest (unit) run.
        exclude: ['node_modules', 'dist', 'tests/e2e/**'],
    },
})
