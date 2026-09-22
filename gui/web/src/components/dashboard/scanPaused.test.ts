import {describe, expect, it} from "vitest";
import {isCoverageScanPaused} from "./scanPaused.ts";

describe("isCoverageScanPaused", () => {
  it("recognises only the live autonomous MOWING overlay", () => {
    expect(isCoverageScanPaused({state: 2, state_name: "MOWING", sub_state_name: "SCAN_PAUSED"}))
      .toBe(true);
    expect(isCoverageScanPaused({state: 1, state_name: "MOWING", sub_state_name: "SCAN_PAUSED"}))
      .toBe(false);
    expect(isCoverageScanPaused({state: 2, state_name: "RETURNING_HOME", sub_state_name: "SCAN_PAUSED"}))
      .toBe(false);
    expect(isCoverageScanPaused({state: 2, state_name: "MOWING", sub_state_name: "FOLLOWING"}))
      .toBe(false);
  });
});
