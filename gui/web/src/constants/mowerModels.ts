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
        label: "YardForce Classic 500",
        description: "Most common model. 28V battery, 18cm blade, rear-wheel drive.",
        tag: "Popular",
        defaults: {
            wheel_radius: 0.04475, wheel_track: 0.325, wheel_x_offset: 0.0,
            wheel_width: 0.04, chassis_height: 0.19, chassis_mass_kg: 8.76,
            caster_radius: 0.03, caster_track: 0.36,
            blade_radius: 0.09, tool_width: 0.18, ticks_per_revolution: 84,
            battery_full_voltage: 28.5, battery_empty_voltage: 24.0,
            battery_critical_voltage: 23.0,
            gps_x: 0.173, gps_y: -0.004, gps_z: 0.2,
            imu_x: 0.187, imu_y: -0.195, imu_z: 0.0, imu_yaw: 0.0,
            lidar_x: 0.0, lidar_y: 0.025, lidar_z: 0.1, lidar_yaw: -3.1416,
            chassis_length: 0.54, chassis_width: 0.40, chassis_center_x: 0.18,
        },
    },
    {
        value: "YardForce500B",
        label: "YardForce 500B",
        description: "500 B variant with different blade motor UART and panel layout.",
        defaults: {
            wheel_radius: 0.04475, wheel_track: 0.325, wheel_x_offset: 0.0,
            wheel_width: 0.04, chassis_height: 0.19, chassis_mass_kg: 8.76,
            caster_radius: 0.03, caster_track: 0.36,
            blade_radius: 0.09, tool_width: 0.18, ticks_per_revolution: 84,
            battery_full_voltage: 28.5, battery_empty_voltage: 24.0,
            battery_critical_voltage: 23.0,
            gps_x: 0.173, gps_y: -0.004, gps_z: 0.2,
            imu_x: 0.187, imu_y: -0.195, imu_z: 0.0, imu_yaw: 0.0,
            lidar_x: 0.0, lidar_y: 0.025, lidar_z: 0.1, lidar_yaw: -3.1416,
            chassis_length: 0.54, chassis_width: 0.40, chassis_center_x: 0.18,
        },
    },
    {
        value: "YardForceSA650",
        label: "YardForce SA650",
        description: "Larger model (570x390mm) with higher encoder resolution.",
        defaults: {
            wheel_radius: 0.04475, wheel_track: 0.325, wheel_x_offset: 0.0,
            wheel_width: 0.04, chassis_height: 0.26, chassis_mass_kg: 9.5,
            caster_radius: 0.03, caster_track: 0.36,
            blade_radius: 0.09, tool_width: 0.18, ticks_per_revolution: 1050,
            battery_full_voltage: 28.5, battery_empty_voltage: 24.0,
            battery_critical_voltage: 23.0,
            gps_x: 0.1, gps_y: 0.0, gps_z: 0.26,
            lidar_x: 0.39, imu_x: 0.19,
            chassis_length: 0.57, chassis_width: 0.39, chassis_center_x: 0.19,
        },
    },
    {
        value: "YardForce900ECO",
        label: "YardForce 900 ECO",
        description: "Same chassis as SA650 (570x390mm), larger battery.",
        defaults: {
            wheel_radius: 0.04475, wheel_track: 0.325, wheel_x_offset: 0.0,
            wheel_width: 0.04, chassis_height: 0.26, chassis_mass_kg: 10.0,
            caster_radius: 0.03, caster_track: 0.36,
            blade_radius: 0.09, tool_width: 0.18, ticks_per_revolution: 1050,
            battery_full_voltage: 28.5, battery_empty_voltage: 24.0,
            battery_critical_voltage: 23.0,
            gps_x: 0.3, gps_y: 0.0, gps_z: 0.26,
            lidar_x: 0.39, imu_x: 0.19,
            chassis_length: 0.57, chassis_width: 0.39, chassis_center_x: 0.19,
        },
    },
    {
        value: "LUV1000RI",
        label: "YardForce LUV1000RI",
        description: "574x400mm chassis, narrower wheelbase (0.285m), ultrasonic sensor.",
        defaults: {
            wheel_radius: 0.04475, wheel_track: 0.285, wheel_x_offset: 0.0,
            wheel_width: 0.04, chassis_height: 0.282, chassis_mass_kg: 9.0,
            caster_radius: 0.03, caster_track: 0.36,
            blade_radius: 0.09, tool_width: 0.18, ticks_per_revolution: 1050,
            battery_full_voltage: 28.5, battery_empty_voltage: 24.0,
            battery_critical_voltage: 23.0,
            gps_x: 0.3, gps_y: 0.0, gps_z: 0.28,
            lidar_x: 0.39, imu_x: 0.19,
            chassis_length: 0.574, chassis_width: 0.40, chassis_center_x: 0.19,
        },
    },
    {
        value: "Sabo",
        label: "Sabo MOWiT 500F",
        description: "Large professional mower (775x535mm), 32cm cutting width.",
        defaults: {
            wheel_radius: 0.04475, wheel_track: 0.45, wheel_x_offset: 0.0,
            wheel_width: 0.05, chassis_height: 0.36, chassis_mass_kg: 14.0,
            caster_radius: 0.035, caster_track: 0.45,
            blade_radius: 0.16, tool_width: 0.32, ticks_per_revolution: 1050,
            battery_full_voltage: 28.5, battery_empty_voltage: 21.0,
            battery_critical_voltage: 20.0,
            gps_x: 0.18, gps_y: 0.0, gps_z: 0.36,
            lidar_x: 0.49, imu_x: 0.29,
            chassis_length: 0.775, chassis_width: 0.535, chassis_center_x: 0.29,
        },
    },
    {
        value: "CUSTOM",
        label: "Custom Robot",
        description: "Manually configure all hardware parameters for a custom build.",
        defaults: {},
    },
];
