// Single source for the onboarding wizard's step indices. Shared by the wizard
// (navigation + save gating) and the readiness step (CTA deep-links), so the
// two never drift out of lockstep.
export const STEP_WELCOME = 0;
export const STEP_ROBOT_MODEL = 1;
export const STEP_FIRMWARE = 2;
export const STEP_NTRIP = 3;
export const STEP_GPS = 4;
export const STEP_DATUM = 5;
export const STEP_SENSORS = 6;
export const STEP_CALIBRATION = 7;
export const STEP_COMPLETE = 8;

export const STEP_COUNT = 9;
