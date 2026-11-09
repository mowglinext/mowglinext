import {expect, test, type Page} from "@playwright/test";
import {installMockBackend} from "./mock/mockBackend.ts";
import {SCENARIOS, type Scenario} from "./mock/scenarios.ts";
import en from "../../src/i18n/locales/en.json" with {type: "json"};
import fr from "../../src/i18n/locales/fr.json" with {type: "json"};

const docked = SCENARIOS.find(s => s.name === "charging-on-dock")!;
const chargeHold = (stateName = "CHARGING", batteryPercent = 98, sessionActive = true): Scenario => ({
  ...docked,
  name: stateName,
  rest: {...docked.rest, "/api/settings/yaml": {battery_manual_resume_percent: 35}},
  topics: {
    ...docked.topics,
    highLevelStatus: {
      state: 1, state_name: stateName, battery_percent: batteryPercent,
      is_charging: true, emergency: false,
    },
    coverageSession: {session_active: sessionActive},
  },
});

// The shell subscribes before the lazy Home page mounts. Keep publishing like
// the live telemetry stream so both consumers receive the scenario's values.
async function installChargeHold(page: Page, scenario: Scenario) {
  await installMockBackend(page, {
    ...scenario,
    topicSequences: Object.fromEntries(Object.entries(scenario.topics ?? {}).map(([topic, value]) => [topic, [value, value]])),
  }, {liveStatusIntervalMs: 250});
}

for (const [language, copy] of [["en", en], ["fr", fr]] as const) {
  for (const width of [1440, 390]) {
    test.describe(`${language} ${width}px`, () => {
      test.use({viewport: {width, height: 900}});
      test.beforeEach(async ({page}) => {
        await page.addInitScript(lang => localStorage.setItem("mowglinext.lang", lang), language);
      });

      test("charge hold has one explanation and one caption per control", async ({page}) => {
        await installChargeHold(page, chargeHold());
        await page.goto("/#/");
        await expect(page.getByRole("heading", {level: 1})).toContainText(copy.mowgliNextPage.headlineMowingPausedPrefix);
        await expect(page.getByText(copy.mowgliNextPage.sublineChargeHold, {exact: true})).toHaveCount(1);
        await expect(page.getByText(copy.actionCluster.cancelMowing, {exact: true})).toHaveCount(1);
        await expect(page.getByText(copy.actionCluster.resumeNow, {exact: true})).toHaveCount(1);
        await expect(page.getByText(copy.actionCluster.resumeAvailable.replace("{{percent}}", "35"), {exact: true})).toBeVisible();
        await expect(page.getByRole("button", {name: copy.actionCluster.resumeNow, exact: true})).toBeEnabled();
        await expect(page.getByRole("button", {name: copy.actionCluster.emergencyStop, exact: true})).toBeVisible();
        await expect(page.getByText(/95%|0\.08/)).toHaveCount(0);
        await page.screenshot({path: `tests/e2e/.artifacts/charge-hold-${language}-${width}.png`, animations: "disabled"});
      });
    });
  }
}

test.beforeEach(async ({page}) => {
  await page.addInitScript(() => localStorage.setItem("mowglinext.lang", "en"));
});

test("critical charge hold preserves resume and cancel commands", async ({page}) => {
  const commands: {command: string; args: unknown}[] = [];
  await installChargeHold(page, chargeHold("CRITICAL_BATTERY_CHARGING"));
  await page.route("**/api/mowglinext/call/*", route => {
    commands.push({command: route.request().url().split("/").pop()!, args: route.request().postDataJSON()});
    return route.fulfill({json: {}});
  });
  await page.goto("/#/");
  await expect(page.getByText(en.mowgliNextPage.sublineChargeHold, {exact: true})).toHaveCount(1);
  await page.getByRole("button", {name: "Resume now", exact: true}).click();
  await expect.poll(() => commands.length).toBe(1);
  await page.getByRole("button", {name: "Cancel mowing", exact: true}).click();
  await expect.poll(() => commands.length).toBe(2);
  expect(commands).toEqual([
    {command: "high_level_control", args: {Command: 1}},
    {command: "high_level_control", args: {Command: 8}},
  ]);
});

test("manual resume remains disabled below the configured floor", async ({page}) => {
  await installChargeHold(page, chargeHold("CHARGING", 34));
  await page.goto("/#/");
  await expect(page.getByRole("button", {name: "Resume now", exact: true})).toBeDisabled();
  await expect(page.getByText("Available from 35%", {exact: true})).toBeVisible();
});

test("manually docked hold explains manual departure once", async ({page}) => {
  await installChargeHold(page, chargeHold("MANUAL_CHARGING"));
  await page.goto("/#/");
  await expect(page.getByText(en.mowgliNextPage.sublineManualChargeHold, {exact: true})).toHaveCount(1);
  await expect(page.getByRole("button", {name: "Resume now", exact: true})).toHaveCount(0);
  await expect(page.getByText("Cancel mowing", {exact: true})).toHaveCount(1);
});

test("ordinary dock charging keeps the normal start control", async ({page}) => {
  await installChargeHold(page, chargeHold("CHARGING", 98, false));
  await page.goto("/#/");
  await expect(page.getByRole("heading", {level: 1})).toContainText(en.mowgliNextPage.headlineChargingPrefix);
  await expect(page.getByRole("button", {name: "Start mowing", exact: true})).toBeVisible();
  await expect(page.getByRole("button", {name: "Resume now", exact: true})).toHaveCount(0);
  await expect(page.getByText(en.mowgliNextPage.sublineChargeHold, {exact: true})).toHaveCount(0);
});

for (const charging of [true, false]) {
  test(`resume undock is explicit while charging=${charging}`, async ({page}) => {
    const scenario = chargeHold("RESUMING_UNDOCKING");
    scenario.topics!.highLevelStatus = {
      state: 2, state_name: "RESUMING_UNDOCKING", battery_percent: 98,
      is_charging: charging, emergency: false,
    };
    await installChargeHold(page, scenario);
    await page.goto("/#/");
    await expect(page.getByRole("heading", {level: 1})).toHaveText(en.mowgliNextPage.headlineResumingMowing);
    await expect(page.getByText(en.mowgliNextPage.sublineResumingMowing, {exact: true})).toBeVisible();
    await expect(page.getByText(en.mowgliNextPage.sublineChargeHold, {exact: true})).toHaveCount(0);
    await expect(page.getByRole("button", {name: "Resume now", exact: true})).toHaveCount(0);
  });
}
