import {describe, expect, it} from "vitest";
import {deriveChargeHold, effectiveManualResumePercent} from "./chargeHold.ts";

describe("deriveChargeHold", () => {
  it("keeps ordinary idle/docked charging out of the mowing-session controls", () => {
    expect(deriveChargeHold("IDLE_DOCKED", 55, 30, 20, 95, true)).toBeUndefined();
    expect(deriveChargeHold("CHARGING", 55, 30, 20, 95, false)).toBeUndefined();
  });

  it.each(["CHARGING", "CRITICAL_BATTERY_CHARGING"] as const)(
    "recognises %s as a paused mowing session",
    (stateName) => {
      expect(deriveChargeHold(stateName, 55, 30, 20, 95, true)).toEqual({
        stateName, batteryPercent: 55, batteryFullPercent: 95, manualResumePercent: 30, autoResume: true,
      });
    },
  );

  it("uses the behavior node's low-battery safety band for displayed eligibility", () => {
    expect(effectiveManualResumePercent(22, 25)).toBe(30);
  });

  it("marks a manually docked session as non-resumable from the dashboard", () => {
    expect(deriveChargeHold("MANUAL_CHARGING", 55, 30, 20, 95, true)?.autoResume).toBe(false);
  });
});
