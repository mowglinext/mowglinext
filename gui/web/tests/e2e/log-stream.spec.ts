import { expect, test } from "@playwright/test";
import { installMockBackend } from "./mock/mockBackend.ts";

test("high-rate container logs are retained and reach the live tail", async ({
  page,
}) => {
  const lines = Array.from(
    { length: 1_000 },
    (_, index) => `INFO synthetic log line ${index}`,
  );
  await installMockBackend(page, {
    name: "high-rate-container-logs",
    rest: {
      "/api/containers": {
        containers: [
          {
            id: "mock-container",
            names: ["/mock-container"],
            state: "running",
            labels: { app: "mock" },
          },
        ],
      },
    },
    containerLogs: lines,
  });

  await page.goto("/#/logs");
  const renderedLines = page.getByTestId("log-line");
  await expect(renderedLines).toHaveCount(1_000, { timeout: 15_000 });
  await expect(
    page.getByText("INFO synthetic log line 999", { exact: true }),
  ).toBeVisible();
  await page.screenshot({
    path: "tests/e2e/.artifacts/log-stream-1000-lines.png",
    fullPage: true,
  });
});

// The daemon prefixes every line with an RFC3339Nano stamp
// (ContainerLogsOptions.Timestamps in gui/pkg/providers/docker.go). Pin BOTH
// halves of the contract end to end: the prefix is stripped from the body, and
// the rendered column shows the daemon's instant rather than the browser clock.
test.describe("docker-stamped log lines", () => {
  test.use({ timezoneId: "UTC" });

  test("renders the daemon timestamp and strips it from the body", async ({
    page,
  }) => {
    await installMockBackend(page, {
      name: "docker-stamped-container-logs",
      rest: {
        "/api/containers": {
          containers: [
            {
              id: "mock-container",
              names: ["/mock-container"],
              state: "running",
              labels: { app: "mock" },
            },
          ],
        },
      },
      containerLogs: [
        "2026-05-12T22:02:33.123456789Z INFO docker stamped line",
        "2026-05-12T22:02:34.000000000Z [INFO] [1747087353.123456789] [map_server_node]: planning",
      ],
    });

    await page.goto("/#/logs");

    // Body: both the docker prefix and the ROS epoch are gone.
    await expect(
      page.getByText("INFO docker stamped line", { exact: true }),
    ).toBeVisible();
    await expect(
      page.getByText("[INFO] [map_server_node]: planning", { exact: true }),
    ).toBeVisible();

    // Column: the daemon's instant, not Date.now().
    await expect(page.getByText("2026-05-12T22:02:33").first()).toBeVisible();
  });
});
