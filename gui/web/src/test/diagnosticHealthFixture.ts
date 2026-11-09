import type {DiagnosticHealthInput} from '../utils/diagnosticHealth';
export const healthy = (): DiagnosticHealthInput => ({
    emergency: {active_emergency: false, latched_emergency: false}, highLevel: {state: 1},
    emergencyFresh: true, operationalFresh: true, batteryFresh: true, connected: true,
    gnss: {fix_type: 3, fix_valid: true, horizontal_accuracy_m: 0}, batteryPercent: 80,
    snapshotFresh: true, snapshot: {timestamp: '2026-10-15T00:00:00Z', containers_available: true,
        containers: ['mowgli-ros2','mowglinext-gui'].map(component => ({name: component === 'mowglinext-gui' ? 'mowgli-gui' : component, component, state: 'running', status: '', started_at: ''})),
        system: {cpu_temperature: 40, cpu_temperature_available: true}, coverage: [], cross_checks: {overall_status: 'ok', warnings: [],
            dock_pose: {has_config: false, configured_x: 0, configured_y: 0, configured_yaw: 0, datum_lat: 0, datum_lon: 0}}},
    diagnostics: [{name: 'Hardware Bridge', hardware_id: 'mowgli/hardware_bridge', message: 'OK', level: 0, values: [], receivedAt: 1000, receivedMonotonic: 1000}],
    nowMs: 1000, nowMonotonic: 1000, settings: {lidar_enabled: false},
});
