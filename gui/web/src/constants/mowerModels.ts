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
// wheel_radius: 0.1 m is the maintainer's measurement on the YardForce 500
// (2026-09-05). Every preset here carried 0.04475 m — an 8.95 cm DIAMETER
// drive wheel — copied identically across six different machines, while
// mowgli.urdf.xacro and all three Webots surfaces independently used 0.093 for
// the same wheel. Only the two YardForce 500 variants are corrected here,
// because that is the only machine anyone has actually measured; SA650 /
// 900ECO / LUV1000RI / Sabo still carry the legacy 0.04475 and each needs its
// own measurement. wheel_radius feeds only the URDF (base_link's height above
// ground + wheel visuals) — nothing computes odometry from it — so a wrong
// value silently offsets every sensor z rather than corrupting distance.

export type MowerModel = {
    value: string;
    label: string;
    description: string;
    tag?: string;
    defaults: Record<string, number>;
};

export const MOWER_MODELS: MowerModel[] = [
    {
        value: "YardForce500",
        label: "mowerModels.YardForce500.label",
        description: "mowerModels.YardForce500.description",
        tag: "mowerModels.YardForce500.tag",
        defaults: {
            wheel_radius: 0.1, wheel_track: 0.325, wheel_x_offset: 0.0,
            wheel_width: 0.04, chassis_height: 0.19, chassis_mass_kg: 8.76,
            caster_radius: 0.03, caster_track: 0.36,
            blade_radius: 0.09, tool_width: 0.18, ticks_per_meter: 300,
            battery_full_voltage: 28.5, battery_empty_voltage: 24.0,
            battery_critical_voltage: 23.0,
            gps_x: 0.3, gps_y: 0.0, gps_z: 0.2,
            imu_x: 0.187, imu_y: -0.195, imu_z: 0.0, imu_yaw: 0.0,
            lidar_x: 0.0, lidar_y: 0.025, lidar_z: 0.3, lidar_yaw: -3.1416,
            chassis_length: 0.60, chassis_width: 0.45, chassis_center_x: 0.18,
        },
    },
    {
        value: "YardForce500B",
        label: "mowerModels.YardForce500B.label",
        description: "mowerModels.YardForce500B.description",
        defaults: {
            wheel_radius: 0.1, wheel_track: 0.325, wheel_x_offset: 0.0,
            wheel_width: 0.04, chassis_height: 0.19, chassis_mass_kg: 8.76,
            caster_radius: 0.03, caster_track: 0.36,
            blade_radius: 0.09, tool_width: 0.18, ticks_per_meter: 300,
            battery_full_voltage: 28.5, battery_empty_voltage: 24.0,
            battery_critical_voltage: 23.0,
            gps_x: 0.3, gps_y: 0.0, gps_z: 0.2,
            imu_x: 0.187, imu_y: -0.195, imu_z: 0.0, imu_yaw: 0.0,
            lidar_x: 0.0, lidar_y: 0.025, lidar_z: 0.3, lidar_yaw: -3.1416,
            chassis_length: 0.60, chassis_width: 0.45, chassis_center_x: 0.18,
        },
    },
    {
        value: "YardForceSA650",
        label: "mowerModels.YardForceSA650.label",
        description: "mowerModels.YardForceSA650.description",
        defaults: {
            wheel_radius: 0.04475, wheel_track: 0.325, wheel_x_offset: 0.0,
            wheel_width: 0.04, chassis_height: 0.26, chassis_mass_kg: 9.5,
            caster_radius: 0.03, caster_track: 0.36,
            blade_radius: 0.09, tool_width: 0.18, ticks_per_meter: 1050,
            battery_full_voltage: 28.5, battery_empty_voltage: 24.0,
            battery_critical_voltage: 23.0,
            gps_x: 0.1, gps_y: 0.0, gps_z: 0.26,
            lidar_x: 0.39, imu_x: 0.19,
            chassis_length: 0.57, chassis_width: 0.39, chassis_center_x: 0.19,
        },
    },
    {
        value: "YardForce900ECO",
        label: "mowerModels.YardForce900ECO.label",
        description: "mowerModels.YardForce900ECO.description",
        defaults: {
            wheel_radius: 0.04475, wheel_track: 0.325, wheel_x_offset: 0.0,
            wheel_width: 0.04, chassis_height: 0.26, chassis_mass_kg: 10.0,
            caster_radius: 0.03, caster_track: 0.36,
            blade_radius: 0.09, tool_width: 0.18, ticks_per_meter: 1050,
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
        defaults: {
            wheel_radius: 0.04475, wheel_track: 0.285, wheel_x_offset: 0.0,
            wheel_width: 0.04, chassis_height: 0.282, chassis_mass_kg: 9.0,
            caster_radius: 0.03, caster_track: 0.36,
            blade_radius: 0.09, tool_width: 0.18, ticks_per_meter: 1050,
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
        defaults: {
            wheel_radius: 0.04475, wheel_track: 0.45, wheel_x_offset: 0.0,
            wheel_width: 0.05, chassis_height: 0.36, chassis_mass_kg: 14.0,
            caster_radius: 0.035, caster_track: 0.45,
            blade_radius: 0.16, tool_width: 0.32, ticks_per_meter: 1050,
            battery_full_voltage: 28.5, battery_empty_voltage: 21.0,
            battery_critical_voltage: 20.0,
            gps_x: 0.18, gps_y: 0.0, gps_z: 0.36,
            lidar_x: 0.49, imu_x: 0.29,
            chassis_length: 0.775, chassis_width: 0.535, chassis_center_x: 0.29,
        },
    },
    {
        value: "CUSTOM",
        label: "mowerModels.CUSTOM.label",
        description: "mowerModels.CUSTOM.description",
        defaults: {},
    },
];
