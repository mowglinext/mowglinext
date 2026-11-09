/** Exact subset supplied from the Mowgli bridge observation by the operator.
 * Peuchmower2 latest sample. No receiver identity, header time or other
 * measurement was supplied. */
export const capturedBridgeFixed = {
    backend: 'universal',
    fix_valid: true,
    fix_type: 6,
    rtk_mode: 3,
    capability_flags: 260271615,
    value_flags: 260074911,
    horizontal_accuracy_m: 0.014000000432133675,
    position_observation_sequence: 326347,
};

/** Synthetic scaffolding ONLY for additional fields the captured value mask
 * requires at the decoder boundary. These zero/false values are not hardware
 * observations and must not be cited as receiver/correction measurements. */
export const bridgeFixedWireFixture = {
    ...capturedBridgeFixed,
    vertical_accuracy_m: 0,
    satellites_used: 0,
    satellites_visible: 0,
    mean_cn0_db_hz: 0,
    max_cn0_db_hz: 0,
    correction_stream_status: 0,
    msm_summary_seen: false,
    msm_summary_decoded: false,
    msm_summary_valid: false,
    msm_summary_age_s: 0,
};
