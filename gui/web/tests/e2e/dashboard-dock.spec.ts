import {expect, test} from "@playwright/test";
import {installMockBackend} from "./mock/mockBackend.ts";

const area = {area: {points: [
  {x: 10, y: 10}, {x: 20, y: 10}, {x: 20, y: 20}, {x: 10, y: 20},
]}};

for (const width of [375, 1440]) {
  for (const sample of [
    {name: "saved", coords: {dock_x: 15, dock_y: 15}, unit: [0.5, 0.5]},
    {name: "outside", coords: {dock_x: 30, dock_y: 15}, unit: [0.9, 0.5]},
    {name: "origin", coords: {dock_x: 0, dock_y: 0}, unit: [0.1, 0.9]},
    {name: "missing", coords: {}, unit: null},
    {name: "partial", coords: {dock_x: 15}, unit: null},
    {name: "non-finite", coords: {dock_x: Infinity, dock_y: 15}, unit: null},
  ]) {
    test(`dashboard dock ${sample.name} at ${width}px`, async ({page}) => {
      await page.setViewportSize({width, height: 900});
      await installMockBackend(page, {
        name: "dashboard-dock",
        topics: {
          map: {working_area: [area], ...sample.coords},
          fusionRaw: {pose: {pose: {position: {x: 15, y: 15, z: 0}}}},
        },
      });
      await page.goto("/#/mowglinext");
      const map = page.getByTestId("live-map-mini");
      // This SVG is only mounted after the page has received recorded areas.
      await expect(map).toBeVisible();
      const marker = page.getByTestId("mini-map-dock");
      if (!sample.unit) {
        await expect(marker).toHaveCount(0);
        return;
      }
      const height = width < 768 ? 220 : 300;
      const transform = await marker.getAttribute("transform");
      const coords = transform!.match(/[-\d.]+/g)!.map(Number);
      expect(coords[0]).toBeCloseTo(sample.unit[0] * 600);
      expect(coords[1]).toBeCloseTo(sample.unit[1] * height);
      // A dock outside the lawn must remain inside the visible SVG on narrow
      // screens too (slice used to crop the ends of the normalised view).
      await marker.scrollIntoViewIfNeeded();
      const bounds = await map.boundingBox();
      const dot = await marker.boundingBox();
      expect(dot!.x).toBeGreaterThanOrEqual(bounds!.x);
      expect(dot!.x + dot!.width).toBeLessThanOrEqual(bounds!.x + bounds!.width);
      expect(dot!.y).toBeGreaterThanOrEqual(bounds!.y);
      expect(dot!.y + dot!.height).toBeLessThanOrEqual(bounds!.y + bounds!.height);
      if (sample.name === "saved") {
        const robot = map.locator('circle[filter="url(#robotGlow)"]').locator("..");
        await expect(robot).toHaveAttribute("transform", transform!);
      }
      await page.screenshot({path: `tests/e2e/.artifacts/dashboard-dock-${sample.name}-${width}.png`});
    });
  }
}
