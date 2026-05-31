import {useEffect, useRef, useState} from "react";
import {useWheelTicks} from "./useWheelTicks.ts";

/**
 * Differentiate raw wheel-tick counters into RPM per wheel.
 *
 * Firmware emits `WheelTick` at ~50 Hz with cumulative `wheel_ticks_*` and
 * direction (0/1) plus `wheel_tick_factor` (ticks per metre at the wheel
 * radius). We don't have ticks-per-revolution directly, so RPM is derived
 * from the radius supplied via `useSettings()` -- or, if we don't have it,
 * we fall back to a configurable default (0.105 m radius -> ~0.66 m
 * circumference -> ticks * factor / circumference revolutions).
 *
 * Returns NaN per wheel until two samples have arrived.
 */

export interface WheelRpm {
  fl: number;
  fr: number;
  rl: number;
  rr: number;
  /** rad/s of the body, derived from L/R difference (useful for sanity) */
  bodyOmega: number;
}

interface UseWheelRpmOptions {
  /** Wheel radius in metres -- supplied so we can convert m/s -> rpm */
  wheelRadiusM?: number;
}

export function useWheelRpm({wheelRadiusM = 0.105}: UseWheelRpmOptions = {}): WheelRpm {
  const ticks = useWheelTicks();
  const [rpm, setRpm] = useState<WheelRpm>({fl: 0, fr: 0, rl: 0, rr: 0, bodyOmega: 0});
  const lastSample = useRef<{
    ts: number;
    fl: number; fr: number; rl: number; rr: number;
  } | null>(null);

  useEffect(() => {
    if (ticks.wheel_tick_factor == null || ticks.wheel_tick_factor === 0) return;
    if (ticks.stamp == null) return;

    const now = ticks.stamp.sec + ticks.stamp.nanosec * 1e-9;
    const fl = ticks.wheel_ticks_fl ?? 0;
    const fr = ticks.wheel_ticks_fr ?? 0;
    const rl = ticks.wheel_ticks_rl ?? 0;
    const rr = ticks.wheel_ticks_rr ?? 0;

    const prev = lastSample.current;
    if (prev != null) {
      const dt = now - prev.ts;
      if (dt > 0 && dt < 5) {
        // ticks-per-metre at the wheel surface
        const factor = ticks.wheel_tick_factor;
        const circumference = 2 * Math.PI * wheelRadiusM;
        // direction-aware delta -- direction byte tells us if ticks count
        // up or down; firmware sends the *signed* delta encoded as raw
        // counter, so we use raw delta and apply the direction sign.
        const sign = (v: number | undefined) => (v ?? 1) === 1 ? 1 : -1;
        const dFL = (fl - prev.fl) * sign(ticks.wheel_direction_fl);
        const dFR = (fr - prev.fr) * sign(ticks.wheel_direction_fr);
        const dRL = (rl - prev.rl) * sign(ticks.wheel_direction_rl);
        const dRR = (rr - prev.rr) * sign(ticks.wheel_direction_rr);

        const toRpm = (d: number) => (d / factor) / circumference * 60 / dt;
        const newRpm = {
          fl: toRpm(dFL),
          fr: toRpm(dFR),
          rl: toRpm(dRL),
          rr: toRpm(dRR),
          bodyOmega: 0,
        };
        // approx body omega from L/R wheel delta (assume track ~0.32 m)
        const track = 0.32;
        const leftAvg  = (newRpm.fl + newRpm.rl) / 2 * circumference / 60;
        const rightAvg = (newRpm.fr + newRpm.rr) / 2 * circumference / 60;
        newRpm.bodyOmega = (rightAvg - leftAvg) / track;
        setRpm(newRpm);
      }
    }
    lastSample.current = {ts: now, fl, fr, rl, rr};
  }, [ticks, wheelRadiusM]);

  return rpm;
}
