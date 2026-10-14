import {useEffect, useRef, useState} from 'react';
import type {WheelTick} from '../types/ros';
import {useTopic} from './useTopic';
import {MOTOR_DISPLAY_STALE_MS, unknownWheelRpm, wheelRpmBetween} from '../utils/motorTelemetry';

export const DEFAULT_WHEEL_RADIUS_M = 0.04475;
export type WheelRpm = ReturnType<typeof unknownWheelRpm>;
interface UseWheelRpmOptions {wheelRadiusM?: number}
interface Sample {ticks: WheelTick; radius: number; rpm: WheelRpm; advancedAt: number | null}
const seconds = (t: WheelTick) => t.stamp ? t.stamp.sec + t.stamp.nanosec * 1e-9 : NaN;

/** Shared calibrated wheel RPM display: native and MAVROS use the same WheelTick contract. */
export function useWheelRpm({wheelRadiusM = DEFAULT_WHEEL_RADIUS_M}: UseWheelRpmOptions = {}): WheelRpm {
    const previous = useRef<Sample | null>(null);
    const {data, lastMessageAt} = useTopic<Sample>('ticks', {
        ticks: {}, radius: wheelRadiusM, rpm: unknownWheelRpm(), advancedAt: null,
    }, {withTimestamp: true, select: raw => {
        const ticks = (raw !== null && typeof raw === "object" ? raw : {}) as WheelTick;
        const old = previous.current;
        const sample: Sample = {ticks, radius: wheelRadiusM, rpm: unknownWheelRpm(), advancedAt: Date.now()};
        if (!((ticks.wheel_tick_factor ?? 0) > 0) || !((ticks.valid_wheels ?? 0) > 0)) sample.advancedAt = null;
        else if (old && old.radius === wheelRadiusM) {
            if (seconds(ticks) === seconds(old.ticks)) {
                // Cached delivery cannot renew or resurrect a measurement.
                const mask = (ticks.valid_wheels ?? 0) & (old.ticks.valid_wheels ?? 0);
                sample.ticks = {...ticks, valid_wheels: mask};
                sample.rpm = {
                    fl: mask & 1 ? old.rpm.fl : NaN, fr: mask & 2 ? old.rpm.fr : NaN,
                    rl: mask & 4 ? old.rpm.rl : NaN, rr: mask & 8 ? old.rpm.rr : NaN,
                    bodyOmega: (mask & 12) === 12 ? old.rpm.bodyOmega : NaN,
                };
                sample.advancedAt = old.advancedAt;
            } else sample.rpm = wheelRpmBetween(old.ticks, ticks, wheelRadiusM);
        }
        previous.current = sample;
        return sample;
    }});
    const [now, setNow] = useState(Date.now);
    useEffect(() => {
        const timer = window.setInterval(() => setNow(Date.now()), 1000);
        return () => window.clearInterval(timer);
    }, []);
    const displayNow = Math.max(now, lastMessageAt ?? now);
    return data.radius === wheelRadiusM && data.advancedAt !== null && displayNow - data.advancedAt >= 0
        && displayNow - data.advancedAt < MOTOR_DISPLAY_STALE_MS ? data.rpm : unknownWheelRpm();
}
