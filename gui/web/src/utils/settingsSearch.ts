import type {TFunction} from 'i18next';

// Legacy form labels. Declarative fields use settingsFields.<key> directly.
// Keep aliases here so natural-language search and technical keys share one index.
const FIELD_TEXT: Record<string, string[]> = {
    "imu_cal_samples": [
        "settingsSensors.calibrationSamples",
        "settingsSensors.calibrationSamplesTooltip"
    ],
    "imu_cal_auto_rest_sec": [
        "settingsSensors.restWindowBeforeCal",
        "settingsSensors.restWindowBeforeCalTooltip"
    ],
    "imu_cal_periodic_recal_sec": [
        "settingsSensors.periodicRecalInterval",
        "settingsSensors.periodicRecalIntervalTooltip"
    ],
    "battery_full_voltage": [
        "settingsBattery.fullVoltage",
        "settingsBattery.fullVoltageTooltip"
    ],
    "battery_empty_voltage": [
        "settingsBattery.emptyVoltage",
        "settingsBattery.emptyVoltageTooltip"
    ],
    "battery_critical_voltage": [
        "settingsBattery.criticalVoltage",
        "settingsBattery.criticalVoltageTooltip"
    ],
    "battery_full_percent": [
        "settingsBattery.resumeAbove",
        "settingsBattery.resumeAboveTooltip"
    ],
    "battery_low_percent": [
        "settingsBattery.lowDock",
        "settingsBattery.lowDockTooltip"
    ],
    "battery_critical_percent": [
        "settingsBattery.critical",
        "settingsBattery.criticalTooltip"
    ],
    "battery_critical_recovery_percent": [
        "settingsBattery.criticalRecovery",
        "settingsBattery.criticalRecoveryTooltip"
    ],
    "battery_manual_resume_percent": [
        "settingsBattery.manualResume",
        "settingsBattery.manualResumeTooltip"
    ],
    "battery_charge_tail_current_a": [
        "settingsBattery.chargeTail",
        "settingsBattery.chargeTailTooltip"
    ],
    "mowing_speed": [
        "settingsMowing.mowingSpeed",
        "settingsMowing.mowingSpeedTooltip"
    ],
    "transit_speed": [
        "settingsMowing.transitSpeed",
        "settingsMowing.transitSpeedTooltip"
    ],
    "blade_load_min_speed_ratio": [
        "settingsMowing.bladeLoadMinSpeedRatio",
        "settingsMowing.bladeLoadMinSpeedRatioTooltip"
    ],
    "headland_width": [
        "settingsMowing.headlandWidth",
        "settingsMowing.headlandWidthTooltip"
    ],
    "connector_max_headland_passes": [
        "settingsMowing.connectorMaxHeadlandPasses",
        "settingsMowing.connectorMaxHeadlandPassesTooltip",
        "settingsMowing.connectorMaxHeadlandPassesUnlimited"
    ],
    "chassis_safety_inset": [
        "settingsMowing.chassisSafetyInset",
        "settingsMowing.chassisSafetyInsetTooltip"
    ],
    "min_turning_radius": [
        "settingsMowing.minTurningRadius",
        "settingsMowing.minTurningRadiusTooltip"
    ],
    "swath_overlap": [
        "settingsMowing.swathOverlap",
        "settingsMowing.swathOverlapTooltip"
    ],
    "mow_direction": [
        "settingsMowing.mowDirection",
        "settingsMowing.mowDirectionTooltip",
        "settingsMowing.mowDirectionAuto",
        "settingsMowing.mowDirectionCw",
        "settingsMowing.mowDirectionCcw"
    ],
    "mow_cross_hatch": [
        "settingsMowing.crossHatch",
        "settingsMowing.crossHatchTooltip",
        "settingsMowing.crossHatch"
    ],
    "undock_distance": [
        "dockingSection.undockDistance",
        "dockingSection.undockDistanceTooltip"
    ],
    "undock_speed": [
        "dockingSection.undockSpeed",
        "dockingSection.undockSpeedTooltip"
    ],
    "dock_approach_distance": [
        "dockingSection.approachDistance",
        "dockingSection.approachDistanceTooltip"
    ],
    "dock_use_charger_detection": [
        "dockingSection.chargerDetection",
        "dockingSection.chargerDetectionTooltip"
    ],
    "dock_max_retries": [
        "dockingSection.maxRetries",
        "dockingSection.maxRetriesTooltip"
    ],
    "dock_charging_threshold": [
        "dockingSection.chargingThreshold",
        "dockingSection.chargingThresholdTooltip"
    ],
    "dock_approach_overshoot": [
        "dockingSection.approachOvershoot",
        "dockingSection.approachOvershootTooltip"
    ],
    "dock_pose_yaw_sigma_rad": [
        "dockingSection.baseHeadingUncertainty",
        "dockingSection.baseHeadingUncertaintyTooltip"
    ],
    "declination_deg": [
        "settingsLocalization.magneticDeclinationLabel",
        "settingsLocalization.magneticDeclinationTooltip"
    ],
    "min_horizontal_u": [
        "settingsLocalization.minHorizontalFieldLabel",
        "settingsLocalization.minHorizontalFieldTooltip"
    ],
    "mag_yaw_variance": [
        "settingsLocalization.magYawVarianceLabel",
        "settingsLocalization.magYawVarianceTooltip"
    ],
    "mqtt_host": [
        "settingsMqtt.host",
        "settingsMqtt.hostTooltip"
    ],
    "mqtt_port": [
        "settingsMqtt.port"
    ],
    "mqtt_use_ssl": [
        "settingsMqtt.useSsl",
        "settingsMqtt.useSslTooltip"
    ],
    "mqtt_username": [
        "settingsMqtt.username",
        "settingsMqtt.usernameTooltip"
    ],
    "mqtt_password": [
        "settingsMqtt.password"
    ],
    "mqtt_topic_prefix": [
        "settingsMqtt.topicPrefix",
        "settingsMqtt.topicPrefixTooltip"
    ],
    "xy_goal_tolerance": [
        "settingsNavigation.transitXyTolerance",
        "settingsNavigation.transitXyToleranceTooltip"
    ],
    "yaw_goal_tolerance": [
        "settingsNavigation.yawTolerance",
        "settingsNavigation.yawToleranceTooltip"
    ],
    "coverage_xy_tolerance": [
        "settingsNavigation.coverageXyTolerance",
        "settingsNavigation.coverageXyToleranceTooltip"
    ],
    "progress_timeout_sec": [
        "settingsNavigation.progressTimeout",
        "settingsNavigation.progressTimeoutTooltip"
    ],
    "boundary_inner_margin_m": [
        "settingsNavigation.boundaryInnerMargin",
        "settingsNavigation.boundaryInnerMarginTooltip"
    ],
    "dock_inner_margin_exempt_radius_m": [
        "settingsNavigation.dockExemptRadius",
        "settingsNavigation.dockExemptRadiusTooltip"
    ],
    "datum_lat": [
        "settingsPositioning.latitude"
    ],
    "datum_lon": [
        "settingsPositioning.longitude"
    ],
    "gnss_config_baud": [
        "settingsPositioning.baudLabel",
        "settingsPositioning.baudTooltip"
    ],
    "gnss_profile_rate_hz": [
        "settingsPositioning.positionRateLabel"
    ],
    "gnss_receiver_family": [
        "settingsPositioning.receiverFamilyLabel"
    ],
    "gps_wait_after_undock_sec": [
        "settingsPositioning.rtkWaitAfterUndockLabel"
    ],
    "gps_timeout_sec": [
        "settingsPositioning.gpsTimeoutLabel"
    ],
    "robot_name": [
        "settingsHardware.robotName",
        "settingsHardware.robotNameTooltip"
    ],
    "wheel_radius": [
        "settingsHardware.wheelRadius",
        "settingsHardware.wheelRadiusTooltip"
    ],
    "wheel_track": [
        "settingsHardware.wheelTrack",
        "settingsHardware.wheelTrackTooltip"
    ],
    "blade_radius": [
        "settingsHardware.bladeRadius",
        "settingsHardware.bladeRadiusTooltip"
    ],
    "tool_width": [
        "settingsHardware.toolWidth",
        "settingsHardware.toolWidthTooltip"
    ],
    "ticks_per_meter": [
        "settingsHardware.encoderTicksPerMeter",
        "settingsHardware.encoderTicksPerMeterTooltip"
    ],
    "chassis_length": [
        "settingsHardware.chassisLength"
    ],
    "chassis_width": [
        "settingsHardware.chassisWidth"
    ],
    "chassis_height": [
        "settingsHardware.chassisHeight"
    ],
    "chassis_center_x": [
        "settingsHardware.chassisCenterX",
        "settingsHardware.chassisCenterXTooltip"
    ],
    "chassis_mass_kg": [
        "settingsHardware.mass"
    ],
    "wheel_width": [
        "settingsHardware.wheelWidth"
    ],
    "wheel_x_offset": [
        "settingsHardware.wheelXOffset",
        "settingsHardware.wheelXOffsetTooltip"
    ],
    "caster_radius": [
        "settingsHardware.casterRadius"
    ],
    "caster_x_offset": ["settingsHardware.casterXOffset", "settingsHardware.casterXOffsetTooltip"],
    "caster_track": [
        "settingsHardware.casterTrack"
    ],
    "obstacle_inflation_radius": [
        "settingsObstacles.inflationRadius",
        "settingsObstacles.inflationRadiusTooltip"
    ],
    "max_obstacle_avoidance_distance": [
        "settingsObstacles.maxDetourDistance",
        "settingsObstacles.maxDetourDistanceTooltip"
    ],
    "obstacle_margin": [
        "settingsObstacles.drawnObstacleMargin",
        "settingsObstacles.drawnObstacleMarginTooltip",
        "settingsObstacles.drawnObstacleMarginBesideBody"
    ],
    "obstacle_clearance_margin": [
        "settingsObstacles.clearanceMargin",
        "settingsObstacles.clearanceMarginTooltip"
    ],
    "obstacle_detection_range_m": [
        "settingsObstacles.detectionRange",
        "settingsObstacles.detectionRangeTooltip"
    ],
    "obstacle_wait_timeout_s": [
        "settingsObstacles.waitTimeout",
        "settingsObstacles.waitTimeoutTooltip"
    ],
    "obstacle_slowdown_ratio": [
        "settingsObstacles.slowdownRatio",
        "settingsObstacles.slowdownRatioTooltip"
    ],
    "rain_delay_minutes": [
        "settingsRain.resumeDelay",
        "settingsRain.resumeDelayTooltip"
    ],
    "rain_debounce_sec": [
        "settingsRain.debounce",
        "settingsRain.debounceTooltip"
    ],
    "led_count": [
        "settingsLeds.ledCount",
        "settingsLeds.ledCountTooltip"
    ],
    "led_spi_device": [
        "settingsLeds.spiDevice",
        "settingsLeds.spiDeviceTooltip"
    ],
    "led_spi_speed_hz": [
        "settingsLeds.spiClock",
        "settingsLeds.spiClockTooltip"
    ],
    "led_brightness": [
        "settingsLeds.brightness",
        "settingsLeds.brightnessTooltip"
    ],
    "led_idle_scale": [
        "settingsLeds.idleBrightness",
        "settingsLeds.idleBrightnessTooltip"
    ],
    "led_refresh_hz": [
        "settingsLeds.refreshRate",
        "settingsLeds.refreshRateTooltip"
    ],
    "led_low_battery_percent": [
        "settingsLeds.lowBattery",
        "settingsLeds.lowBatteryTooltip"
    ],
    "led_charge_full_percent": [
        "settingsLeds.chargeFull",
        "settingsLeds.chargeFullTooltip"
    ],
    "led_status_timeout_s": [
        "settingsLeds.statusTimeout",
        "settingsLeds.statusTimeoutTooltip"
    ],
    "led_keepalive_s": [
        "settingsLeds.keepalive",
        "settingsLeds.keepaliveTooltip"
    ],
    "led_device_retry_s": [
        "settingsLeds.deviceRetry",
        "settingsLeds.deviceRetryTooltip"
    ],
    "led_charge_complete_timeout_s": [
        "settingsLeds.chargeCompleteTimeout",
        "settingsLeds.chargeCompleteTimeoutTooltip"
    ],
    "led_charge_complete_dim_scale": [
        "settingsLeds.chargeCompleteDim",
        "settingsLeds.chargeCompleteDimTooltip"
    ],
    "led_charge_complete_indicator_count": [
        "settingsLeds.chargeCompleteIndicatorCount",
        "settingsLeds.chargeCompleteIndicatorCountTooltip"
    ],
    "led_charge_complete_indicator_scale": [
        "settingsLeds.chargeCompleteIndicatorScale",
        "settingsLeds.chargeCompleteIndicatorScaleTooltip"
    ],
    "led_charge_complete_indicator_ids": [
        "settingsLeds.chargeCompleteIndicatorIds",
        "settingsLeds.chargeCompleteIndicatorIdsTooltip"
    ],
    "wheel_pid_kp": [
        "settingsDriveMotor.params.kpLabel",
        "settingsDriveMotor.kpTooltip"
    ],
    "wheel_pid_ki": [
        "settingsDriveMotor.params.kiLabel",
        "settingsDriveMotor.kiTooltip"
    ],
    "wheel_pid_kd": [
        "settingsDriveMotor.params.kdLabel",
        "settingsDriveMotor.kdTooltip"
    ],
    "wheel_pid_integral_limit": [
        "settingsDriveMotor.integralLimit",
        "settingsDriveMotor.integralLimitTooltip"
    ],
    "wheel_pid_pwm_per_mps": [
        "settingsDriveMotor.pwmPerMps",
        "settingsDriveMotor.pwmPerMpsTooltip"
    ],
    "lidar_enabled": [
        "settingsSensors.lidarSensor",
        "settingsSensors.lidarDescription"
    ],
    "mower_model": [
        "settingsHardware.robotModel",
        "settingsHardware.robotModelDescription"
    ]
};

export function normalizeSearch(value: string): string {
    return value.normalize('NFD').replace(/[\u0300-\u036f]/g, '').toLocaleLowerCase().replace(/[_-]+/g, ' ').trim();
}

export function settingSearchText(key: string, t: TFunction): string[] {
    const paths = FIELD_TEXT[key] ?? [`settingsFields.${key}.label`, `settingsFields.${key}.tooltip`];
    return paths.map(path => t(path, {defaultValue: ''})).filter(Boolean);
}

export function matchesSettingSearch(query: string, ...texts: string[]): boolean {
    const normalized = normalizeSearch(texts.join(' '));
    return normalizeSearch(query).split(/\s+/).every(word => normalized.includes(word));
}
