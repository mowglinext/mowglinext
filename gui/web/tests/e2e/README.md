# GUI end-to-end tests (Playwright)

Every page is driven by **fully mocked data** — no Go backend and no robot are
required. The only thing they need is the vite dev server, and Playwright
starts it for you (`webServer` in `playwright.config.ts`, port 5173,
`reuseExistingServer: true` — a `yarn dev` you already have running is reused).

```bash
yarn test:e2e         # headless, the whole suite (5 spec files)
yarn test:e2e:ui      # interactive UI mode
npx playwright test pages.spec.ts             # all scenarios × all pages
npx playwright test -g "emergency-latched"    # one scenario
npx playwright show-report                    # open the HTML report
```

## How the mocking works

- **REST** is intercepted with `page.route` (see `mock/mockBackend.ts`). The
  matcher is anchored to the `/api/` path root so it never swallows the app's
  own vite modules (`/src/api/*`). Every endpoint returns a neutral default;
  a scenario overrides specific pathnames via its `rest` map.
- **The multiplex WebSocket** (`/api/mowglinext/multiplex`) is intercepted with
  `page.routeWebSocket`. When the page sends `{op:"subscribe",topic}` the mock
  replies with a MessagePack frame `pack({topic, data})` — the exact wire
  format the real Go backend uses. Topic payloads come from the scenario's
  `topics` map. `silentSocket: true` accepts the connection but sends nothing
  (the "offline / stale data" state).
- **Container logs** (`/api/containers/<id>/logs`) get their own
  `routeWebSocket`: each string in the scenario's `containerLogs` array is sent
  base64-encoded, the text wire format that route really uses. The dedicated
  `/api/mowglinext/subscribe/*` sockets are accepted and left silent.

## Scenarios = robot-state permutations

`mock/scenarios.ts` enumerates the situations we want to be able to view and
regression-test: idle-docked, mowing (non-zero area), low-battery returning,
latched emergency, charging, rain-detected docking, no-GPS (float), and a
fully offline socket. **Add a scenario there and every `pages.spec.ts` case
picks it up automatically** — the focused specs below pull one scenario by name
(`SCENARIOS.find(...)`) or declare an inline one.

## What the specs check

`pages.spec.ts` loads each page under each scenario and asserts:
1. the shell mounts,
2. the page title renders (the route actually mounted, no crash / error
   boundary),
3. **zero uncaught page errors** in any state,

then writes `tests/e2e/.artifacts/<page>__<scenario>.png` so every situation
is reviewable. The artifacts dir is git-ignored (`gui/web/.gitignore`) and is
also Playwright's `outputDir`.

The other four specs are targeted regressions rather than a matrix:

| Spec | Pins |
|------|------|
| `log-stream.spec.ts` | 1 000 high-rate container log lines all reach the live tail; a docker RFC3339Nano stamp is shown in the timestamp column and stripped from the body. |
| `map-console.spec.ts` | map layers mount with **no** `missing required property "source"` / `React.Fragment` console errors. |
| `reset-mowing-progress.spec.ts` | "Reset mowing progress" confirms first, then calls **only** `coverage_clear_resume`; the menu item is disabled while mowing. |
| `visual-effects.spec.ts` | Balanced/Efficient display modes run zero backdrop-blur and zero infinite animations; Visual restores them; emergency emphasis is kept but yields to `prefers-reduced-motion`. |

## Where this fits

No CI workflow runs this suite — it is a by-hand gate (see
[`docs/claude/testing-ci.md`](../../../../docs/claude/testing-ci.md) for what CI
does gate, and [`docs/claude/codemaps/gui_frontend.md`](../../../../docs/claude/codemaps/gui_frontend.md)
for the pages, hooks and topic keys these specs drive).
