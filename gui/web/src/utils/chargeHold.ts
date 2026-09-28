export type ChargeHold = {
  stateName: "CHARGING" | "CRITICAL_BATTERY_CHARGING" | "MANUAL_CHARGING";
  batteryPercent: number;
  batteryFullPercent: number;
  manualResumePercent: number;
  autoResume: boolean;
};

// Mirror the behavior node's safety band for display only. The BT remains the
// authority that accepts or refuses COMMAND_START when it receives the request.
export function effectiveManualResumePercent(configured: number, batteryLowPercent: number): number {
  return Math.max(configured, batteryLowPercent + 5);
}

/** Distinguish an idle dock charge from a battery-initiated mowing-session hold. */
export function deriveChargeHold(
  stateName: string,
  batteryPercent: number,
  configuredManualResumePercent: number,
  batteryLowPercent: number,
  batteryFullPercent: number,
  sessionActive: boolean,
): ChargeHold | undefined {
  if (!sessionActive || (stateName !== "CHARGING" && stateName !== "CRITICAL_BATTERY_CHARGING" && stateName !== "MANUAL_CHARGING")) {
    return undefined;
  }
  return {
    stateName,
    batteryPercent,
    batteryFullPercent,
    manualResumePercent: effectiveManualResumePercent(configuredManualResumePercent, batteryLowPercent),
    autoResume: stateName !== "MANUAL_CHARGING",
  };
}
