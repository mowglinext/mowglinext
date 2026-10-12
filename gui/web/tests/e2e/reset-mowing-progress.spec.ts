import { expect, test } from "@playwright/test";
import { installMockBackend } from "./mock/mockBackend.ts";
import { SCENARIOS } from "./mock/scenarios.ts";

test("reset mowing progress confirms, clears only resume data, and reports success", async ({
  page,
}) => {
  const idleScenario = SCENARIOS.find(
    (scenario) => scenario.name === "idle-docked-full",
  );
  if (!idleScenario) throw new Error("idle-docked-full scenario is missing");

  await installMockBackend(page, {
    ...idleScenario,
    rest: {
      ...idleScenario.rest,
      "/api/settings/yaml": { datum_lat: 48.1, datum_lon: 11.5 },
    },
  });

  const mowerCommands: string[] = [];
  await page.route(/\/api\/mowglinext\/call\//, async (route) => {
    const path = new URL(route.request().url()).pathname;
    // The map also polls this read-only service. Keep every other service in
    // the assertion so unexpected start/blade/mutation commands still fail.
    if (!path.endsWith('/get_lidar_ignore_corridors')) mowerCommands.push(path);
    await route.fulfill({
      status: 200,
      contentType: "application/json",
      body: JSON.stringify({}),
    });
  });

  await page.goto("/#/map");
  await page.getByRole("button", { name: "More" }).click();
  await page.getByText("Reset mowing progress").click();

  const confirmation = page.getByRole("dialog");
  await expect(confirmation.locator(".ant-modal-confirm-title")).toHaveText(
    "Reset saved mowing progress?",
  );
  await expect(
    confirmation.getByText(
      "The mower will not start, and your mapped areas will not be deleted.",
    ),
  ).toBeVisible();
  expect(mowerCommands).toEqual([]);
  await page.waitForTimeout(300);
  await page.screenshot({
    path: "tests/e2e/.artifacts/reset-mowing-progress-confirmation.png",
    fullPage: true,
  });

  await confirmation.getByRole("button", { name: "Reset progress" }).click();

  await expect(page.getByText("Mowing progress was reset")).toBeVisible();
  expect(mowerCommands).toEqual(["/api/mowglinext/call/coverage_clear_resume"]);
});

test("reset mowing progress is disabled during an active mowing mission", async ({
  page,
}) => {
  const mowingScenario = SCENARIOS.find(
    (scenario) => scenario.name === "mowing-area2-rtk-fixed",
  );
  if (!mowingScenario)
    throw new Error("mowing-area2-rtk-fixed scenario is missing");

  await installMockBackend(page, {
    ...mowingScenario,
    rest: {
      ...mowingScenario.rest,
      "/api/settings/yaml": { datum_lat: 48.1, datum_lon: 11.5 },
    },
  });

  await page.goto("/#/map");
  await page.getByRole("button", { name: "More" }).click();

  const resetItem = page.getByRole("menuitem", {
    name: "Reset mowing progress",
  });
  await expect(resetItem).toHaveAttribute("aria-disabled", "true");
});
