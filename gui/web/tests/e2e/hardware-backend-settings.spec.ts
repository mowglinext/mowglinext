import {expect, test} from "@playwright/test";
import {installMockBackend} from "./mock/mockBackend.ts";

const calibration = {
    ticks_per_meter: 401.5,
    wheel_track: 0.325,
    wheel_pid_kp: 10,
    wheel_pid_ki: 2000,
    wheel_pid_kd: 0,
    wheel_pid_integral_limit: 45,
    wheel_pid_pwm_per_mps: 282.135,
};

test.beforeEach(async ({page}) => {
    await page.addInitScript(() => localStorage.setItem("mowglinext.lang", "en"));
});

test("MAVROS shows canonical calibration without Mowgli drive controls", async ({page}) => {
    const posts: string[] = [];
    page.on("request", request => {
        if (request.method() === "POST") posts.push(new URL(request.url()).pathname);
    });
    await installMockBackend(page, {
        name: "mavros-settings",
        rest: {
            "/api/settings/yaml": calibration,
            "/api/settings/hardware-backend": {
                backend: "mavros",
                supported: ["mowgli", "mavros"],
                runtime_routing: "pending_image",
                parameter_routes: {
                    ticks_per_meter: {
                        parameter: "mavros/esc_wheel_odometry.ticks_per_meter",
                        runtime: "pending_image",
                    },
                    wheel_track: {
                        parameter: "mavros/esc_wheel_odometry.track_width_m",
                        runtime: "pending_image",
                    },
                },
            },
        },
    });
    await page.goto("/#/settings");

    await expect(page.getByText("Pixhawk via MAVROS")).toBeVisible();
    await expect(page.getByText(/Live routing is pending/)).toBeVisible();
    await expect(page.getByRole("menuitem", {name: "Drive Motor"})).toHaveCount(0);
    await expect(page.getByText("Encoder Ticks/Meter", {exact: true})).toBeVisible();

    const ticksField = page.locator(".ant-form-item")
        .filter({hasText: "Encoder Ticks/Meter"})
        .getByRole("spinbutton");
    await ticksField.fill("402.5");
    await page.getByRole("button", {name: /Save \(1 changes\)/}).click();
    await expect.poll(() => posts.filter(path => path === "/api/settings/yaml").length).toBe(1);
    expect(posts).not.toContain("/api/params");

    await page.getByRole("menuitem", {name: "Safety"}).click();
    await expect(page.getByText("Firmware Parameters", {exact: true})).toHaveCount(0);
    expect(posts).not.toContain("/api/params");
    expect(posts).not.toContain("/api/tools/drive/ff-calibration/start");
    expect(posts).not.toContain("/api/tools/drive/pid-tuning/start");
});

test("Mowgli keeps its drive controls", async ({page}) => {
    await installMockBackend(page, {
        name: "mowgli-settings",
        rest: {
            "/api/settings/yaml": calibration,
            "/api/settings/hardware-backend": {
                backend: "mowgli",
                supported: ["mowgli", "mavros"],
                runtime_routing: "available",
                parameter_routes: {
                    ticks_per_meter: {parameter: "hardware_bridge.ticks_per_meter", runtime: "available"},
                    wheel_pid_kp: {parameter: "hardware_bridge.wheel_pid_kp", runtime: "available"},
                },
            },
        },
    });
    await page.goto("/#/settings");

    await expect(page.getByText("Mowgli STM32")).toBeVisible();
    await page.getByRole("menuitem", {name: "Drive Motor"}).click();
    await expect(page.getByText("Wheel Velocity PID", {exact: true})).toBeVisible();
    await expect(page.getByRole("button", {name: "Start odometry/feed-forward calibration"})).toBeVisible();
});
