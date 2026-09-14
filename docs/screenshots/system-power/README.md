# Diagnostics: Host power controls

Desktop (1440 × 1000) and mobile (390 × 844) screenshots from
`gui/web/tests/e2e/system-power.spec.ts`. All telemetry and power endpoints are
mocked; no robot is rebooted or shut down by the tests.

Run `yarn test:e2e system-power.spec.ts` from `gui/web`. The captures are written
to `tests/e2e/.artifacts/system-power-*.png`; copy those four PNG files here after
visually checking them. Waits in the screenshot helper only settle entrance
animations; behavioural assertions wait on the actual UI/request state.
