export const validStatus = () => ({feed_forward: {status: 'not_validated'}, pid: {status: 'not_validated'}});
export const validReport = () => ({...validStatus(), latest_report: {mode: 'ff', report_path: '/report.yaml'}, parsed: {
    generated_at: '', mode: 'ff', profile: '', backup_file: '', cmd_topic: '/cmd_vel', turn_direction: 'right',
    applied_live: false, requested_apply: false, auto_turn: false, distance_m: 0, max_speed_mps: 0,
    segment_duration_s: 0, passes: 0, current_params: {}, starting_params: {}, proposed_params: {ticks_per_meter: 343.596123},
    reasons: null, trials: null,
}});
export const validStart = () => ({job_id: 'one', mode: 'ff', state: 'running', started_at: '', apply: false, report_path: '/report.yaml'});

