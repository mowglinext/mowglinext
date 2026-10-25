import type {MowerVisualPreset} from "../hooks/useMowerVisual";

// Per-model physical presets applied by the GUI's "mower model" picker.
//
// PARITY CONTRACT (guarded by gui/pkg/api/schema_template_parity_test.go,
// TestMowerModelPresetMatchesTemplate): the preset whose `value` equals the
// ROS2 template's `mower_model` — "YardForce500" today — is the same machine
// the in-package template ros2/src/mowgli_bringup/config/mowgli_robot.yaml
// describes, so every key it defines MUST equal that template's default.
// Divergences that are known and deliberately NOT fixed are listed with a
// reason in modelPresetDivergesFromTemplate in that test; a NEW divergence
// fails CI. This guard exists because imu_y sat at -0.195 here and 0.0 in the
// template for a long time, silently, and a GUI "reset to default" moved the
// IMU 19.5 cm sideways. The other presets describe DIFFERENT machines and are
// not compared to the template.
//
// Wheel dimensions are radii in metres, not diameters.
// YardForce 500/500B: rear radius 0.1 m, maintainer measurement (2026-09-05).
// SA650: owner-reported diameters on 2026-10-11 are 190 mm rear / 90 mm caster,
// giving radii 0.095 / 0.045 m. Wheel variant and measurement method were not supplied.
// 900ECO / LUV1000RI / Sabo: use those same radii as PROVISIONAL assumptions,
// requested by the contributor to replace the implausible copied 0.04475 m
// rear radius. They are NOT measurements of these three models; verify on each
// robot and override where necessary. See docs/WHEEL_PRESET_DIMENSIONS.md.
// wheel_radius feeds the URDF (base_link height, wheel geometry and derived
// caster/blade positions); it does not compute odometry or ticks_per_meter.

export type MowerModel = {
    value: string;
    label: string;
    description: string;
    tag?: string;
    /** Optional matched chassis/dock appearance; never passed to ROS parameters. */
    appearance?: MowerVisualPreset;
    defaults: Record<string, number>;
};

// Caster X positions are fixed, provisional starting points inferred from the
// preset body envelope (roughly 50 mm tyre clearance from the front edge),
// not measured wheelbases. Measure the actual axle position on each robot.
export const MOWER_MODELS: MowerModel[] = [
    {
        value: "YardForce500",
        label: "mowerModels.YardForce500.label",
        description: "mowerModels.YardForce500.description",
        appearance: {style: "yardforce", dockAppearance: "styled"},
        tag: "mowerModels.YardForce500.tag",
        defaults: {
            wheel_radius: 0.1, wheel_track: 0.325, wheel_x_offset: 0.0,
            wheel_width: 0.04, chassis_height: 0.19, chassis_z_offset: -0.05, chassis_mass_kg: 8.76,
            caster_x_offset: 0.40, caster_radius: 0.03, caster_track: 0.36,
            blade_x_offset: 0.0, blade_y_offset: 0.0, blade_radius: 0.09, tool_width: 0.18, ticks_per_meter: 300,
            battery_full_voltage: 28.5, battery_empty_voltage: 24.0,
            battery_critical_voltage: 23.0,
            // gps_z centres the 0.02 m URDF antenna puck on the shell top:
            // chassis_z_offset + chassis_height + 0.01 = -0.05 + 0.19 + 0.01.
            gps_x: 0.3, gps_y: 0.0, gps_z: 0.15,
            imu_x: 0.187, imu_y: -0.195, imu_z: 0.0, imu_yaw: 0.0,
            lidar_x: 0.0, lidar_y: 0.025, lidar_z: 0.3, lidar_yaw: -3.1416,
            chassis_length: 0.60, chassis_width: 0.45, chassis_center_x: 0.18,
        },
    },
    {
        value: "YardForce500B",
        label: "mowerModels.YardForce500B.label",
        description: "mowerModels.YardForce500B.description",
        appearance: {style: "yardforce", dockAppearance: "styled"},
        defaults: {
            wheel_radius: 0.1, wheel_track: 0.325, wheel_x_offset: 0.0,
            wheel_width: 0.04, chassis_height: 0.19, chassis_z_offset: -0.05, chassis_mass_kg: 8.76,
            caster_x_offset: 0.40, caster_radius: 0.03, caster_track: 0.36,
            blade_x_offset: 0.0, blade_y_offset: 0.0, blade_radius: 0.09, tool_width: 0.18, ticks_per_meter: 300,
            battery_full_voltage: 28.5, battery_empty_voltage: 24.0,
            battery_critical_voltage: 23.0,
            // gps_z centres the 0.02 m URDF antenna puck on the shell top:
            // chassis_z_offset + chassis_height + 0.01 = -0.05 + 0.19 + 0.01.
            gps_x: 0.3, gps_y: 0.0, gps_z: 0.15,
            imu_x: 0.187, imu_y: -0.195, imu_z: 0.0, imu_yaw: 0.0,
            lidar_x: 0.0, lidar_y: 0.025, lidar_z: 0.3, lidar_yaw: -3.1416,
            chassis_length: 0.60, chassis_width: 0.45, chassis_center_x: 0.18,
        },
    },
    {
        value: "YardForceSA650",
        label: "mowerModels.YardForceSA650.label",
        description: "mowerModels.YardForceSA650.description",
        // Owner-reported diameters: rear 190 mm, caster 90 mm (2026-10-11).
        defaults: {
            wheel_radius: 0.095, wheel_track: 0.325, wheel_x_offset: 0.0,
            wheel_width: 0.04, chassis_height: 0.26, chassis_z_offset: 0, chassis_mass_kg: 9.5,
            caster_x_offset: 0.39, caster_radius: 0.045, caster_track: 0.36,
            blade_x_offset: 0.0, blade_y_offset: 0.0, blade_radius: 0.09, tool_width: 0.18, ticks_per_meter: 1050,
            battery_full_voltage: 28.5, battery_empty_voltage: 24.0,
            battery_critical_voltage: 23.0,
            gps_x: 0.1, gps_y: 0.0, gps_z: 0.26,
            lidar_x: 0.39, imu_x: 0.19,
            chassis_length: 0.57, chassis_width: 0.39, chassis_center_x: 0.19,
        },
    },
    {
        value: "BiltemaRM1000",
        label: "mowerModels.BiltemaRM1000.label",
        description: "mowerModels.BiltemaRM1000.description",
        appearance: {style: "rm1000", dockAppearance: "styled"},
        // Tape-measured on one RM1000 (2026-10-25); see docs/RM1000_GEOMETRY.md.
        // The rear wheel measured 180 mm over tread base / 200 mm over lugs:
        // use the 100 mm outer radius for geometry, NOT encoder calibration.
        // Conservatively use 570 mm external length (published dimensions) rather
        // than the 548 mm tape span, keeping the measured chassis centre near +129 mm.
        // Caster placement and body height/clearance remain provisional.
        // Deliberately leave mass, motor/encoder, battery and sensor calibration alone.
        defaults: {
            wheel_radius: 0.1, wheel_track: 0.325, wheel_width: 0.036,
            wheel_x_offset: 0.0,
            caster_radius: 0.045, caster_track: 0.215, caster_x_offset: 0.30,
            chassis_length: 0.57, chassis_width: 0.40, chassis_height: 0.22,
            chassis_center_x: 0.13, chassis_z_offset: -0.06,
            blade_x_offset: 0.06, blade_y_offset: 0.0,
            blade_radius: 0.09, tool_width: 0.18,
            dock_visual_length: 0.63, dock_visual_width: 0.46,
        },
    },
    {
        value: "YardForce900ECO",
        label: "mowerModels.YardForce900ECO.label",
        description: "mowerModels.YardForce900ECO.description",
        // Provisional SA650-based radii; this model has not been measured.
        defaults: {
            wheel_radius: 0.095, wheel_track: 0.325, wheel_x_offset: 0.0,
            wheel_width: 0.04, chassis_height: 0.26, chassis_z_offset: 0, chassis_mass_kg: 10.0,
            caster_x_offset: 0.39, caster_radius: 0.045, caster_track: 0.36,
            blade_x_offset: 0.0, blade_y_offset: 0.0, blade_radius: 0.09, tool_width: 0.18, ticks_per_meter: 1050,
            battery_full_voltage: 28.5, battery_empty_voltage: 24.0,
            battery_critical_voltage: 23.0,
            gps_x: 0.3, gps_y: 0.0, gps_z: 0.26,
            lidar_x: 0.39, imu_x: 0.19,
            chassis_length: 0.57, chassis_width: 0.39, chassis_center_x: 0.19,
        },
    },
    {
        value: "LUV1000RI",
        label: "mowerModels.LUV1000RI.label",
        description: "mowerModels.LUV1000RI.description",
        // Provisional SA650-based radii; this model has not been measured.
        defaults: {
            wheel_radius: 0.095, wheel_track: 0.285, wheel_x_offset: 0.0,
            wheel_width: 0.04, chassis_height: 0.282, chassis_z_offset: 0, chassis_mass_kg: 9.0,
            caster_x_offset: 0.40, caster_radius: 0.045, caster_track: 0.36,
            blade_x_offset: 0.0, blade_y_offset: 0.0, blade_radius: 0.09, tool_width: 0.18, ticks_per_meter: 1050,
            battery_full_voltage: 28.5, battery_empty_voltage: 24.0,
            battery_critical_voltage: 23.0,
            gps_x: 0.3, gps_y: 0.0, gps_z: 0.28,
            lidar_x: 0.39, imu_x: 0.19,
            chassis_length: 0.574, chassis_width: 0.40, chassis_center_x: 0.19,
        },
    },
    {
        value: "Sabo",
        label: "mowerModels.Sabo.label",
        description: "mowerModels.Sabo.description",
        // Provisional SA650-based radii; this model has not been measured.
        defaults: {
            wheel_radius: 0.095, wheel_track: 0.45, wheel_x_offset: 0.0,
            wheel_width: 0.05, chassis_height: 0.36, chassis_z_offset: 0, chassis_mass_kg: 14.0,
            caster_x_offset: 0.59, caster_radius: 0.045, caster_track: 0.45,
            blade_x_offset: 0.0, blade_y_offset: 0.0, blade_radius: 0.16, tool_width: 0.32, ticks_per_meter: 1050,
            battery_full_voltage: 28.5, battery_empty_voltage: 21.0,
            battery_critical_voltage: 20.0,
            gps_x: 0.18, gps_y: 0.0, gps_z: 0.36,
            lidar_x: 0.49, imu_x: 0.29,
            chassis_length: 0.775, chassis_width: 0.535, chassis_center_x: 0.29,
        },
    },
    {
        value: "Worx500MPlus",
        label: "mowerModels.Worx500MPlus.label",
        description: "mowerModels.Worx500MPlus.description",
        appearance: {style: "worx", dockAppearance: "worx"},
        // Contributor configuration screenshot (2026-10-11), not factory specifications.
        // Blade centre confirmed at (0.195, -0.0575) m from base_link, NOT the axle.
        // No battery, sensor or firmware calibration is inferred from this body model.
        defaults: {
            wheel_radius: 0.1, wheel_track: 0.36, wheel_x_offset: -0.005,
            // Provisional shallow shell envelope from the reference photos/approved preview.
            // The screenshot's 230 mm value was not a measured shell height.
            wheel_width: 0.04, chassis_height: 0.14, chassis_z_offset: -0.07, chassis_mass_kg: 8.76,
            // Provisional front caster position: 50 mm tyre clearance from the front edge.
            caster_x_offset: 0.405, caster_radius: 0.03, caster_track: 0.0,
            blade_radius: 0.1, tool_width: 0.20, ticks_per_meter: 343,
            chassis_length: 0.58, chassis_width: 0.395, chassis_center_x: 0.195,
            blade_x_offset: 0.0, blade_y_offset: -0.0575,
        },
    },
    {
        value: "CUSTOM",
        label: "mowerModels.CUSTOM.label",
        description: "mowerModels.CUSTOM.description",
        defaults: {},
    },
];
