# GUI end-to-end tests (Playwright)

Every page is driven by **fully mocked data** — no Go backend and no robot are
required. Run them against nothing but the vite dev server.

```bash
yarn test:e2e         # headless, all scenarios × all pages
yarn test:e2e:ui      # interactive UI mode
npx playwright test -g "emergency-latched"   # one scenario
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

## Scenarios = robot-state permutations

`mock/scenarios.ts` enumerates the situations we want to be able to view and
regression-test: idle-docked, mowing (non-zero area), low-battery returning,
latched emergency, charging, rain-detected docking, no-GPS (float), and a
fully offline socket. **Add a scenario there and every page spec picks it up
automatically.**

## What the spec checks

`pages.spec.ts` loads each page under each scenario and asserts:
1. the shell mounts,
2. the page title renders (the route actually mounted, no crash / error
   boundary),
3. **zero uncaught page errors** in any state,

then writes `tests/e2e/.artifacts/<page>__<scenario>.png` so every situation
is reviewable. The artifacts dir is git-ignored.
