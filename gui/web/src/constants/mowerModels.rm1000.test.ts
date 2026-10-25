import {describe, expect, it} from "vitest";
import {MOWER_MODELS} from "./mowerModels";

const rm1000 = MOWER_MODELS.find(model => model.value === "BiltemaRM1000");

describe("Biltema RM1000 measured geometry preset", () => {
    it("keeps the reported blade location and conservatively encloses the measured shell", () => {
        expect(rm1000).toBeDefined();
        const d = rm1000!.defaults;

        // Owner's tape reference: +403 mm front, -145 mm rear from back axle.
        // The published 570 mm overall envelope adds a little safety margin.
        const front = d.chassis_center_x + d.chassis_length / 2;
        const rear = d.chassis_center_x - d.chassis_length / 2;
        expect(front).toBeGreaterThanOrEqual(0.403);
        expect(rear).toBeLessThanOrEqual(-0.145);
        expect(d.chassis_width).toBeGreaterThanOrEqual(0.395);

        // Blade motor measured +190 mm from rear drive axle, near Y=0.
        expect(d.chassis_center_x + d.blade_x_offset).toBeCloseTo(0.190, 5);
        expect(d.blade_y_offset).toBe(0);
        expect(d.tool_width).toBe(2 * d.blade_radius);

        // Body width must enclose the entire rear wheel outside span.
        expect(d.wheel_track + d.wheel_width).toBeLessThanOrEqual(d.chassis_width);
        expect(d.caster_x_offset + d.caster_radius).toBeLessThan(front - 0.05);

        // Ground clearance/height are provisional and should remain internally coherent.
        expect(d.wheel_radius + d.chassis_z_offset).toBeCloseTo(0.040, 5);
        expect(d.wheel_radius + d.chassis_z_offset + d.chassis_height).toBeCloseTo(0.260, 5);
    });

    it("does not substitute someone else's calibrations for missing RM1000 measurements", () => {
        expect(rm1000).toBeDefined();
        const d = rm1000!.defaults;
        for (const key of [
            "ticks_per_meter", "chassis_mass_kg",
            "gps_x", "gps_y", "gps_z",
            "imu_x", "imu_y", "imu_z",
            "lidar_x", "lidar_y", "lidar_z",
            "battery_full_voltage", "battery_empty_voltage", "battery_critical_voltage",
        ]) {
            expect(d).not.toHaveProperty(key);
        }
    });
});
