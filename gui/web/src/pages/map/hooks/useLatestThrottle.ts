import { useEffect, useLayoutEffect, useState } from "react";

export interface LatestThrottle<T> {
  push: (value: T) => void;
  cancel: () => void;
}

/**
 * Emit immediately, then coalesce values that arrive inside the interval into
 * one trailing update. The trailing update always receives the latest value.
 */
export function createLatestThrottle<T>(
  callback: (value: T) => void,
  intervalMs: number | (() => number),
): LatestThrottle<T> {
  let lastEmitAt = Number.NEGATIVE_INFINITY;
  let latest: T | undefined;
  let hasLatest = false;
  let timer: ReturnType<typeof setTimeout> | null = null;
  const interval = () => typeof intervalMs === 'function' ? intervalMs() : intervalMs;

  const emit = () => {
    timer = null;
    if (!hasLatest) return;
    const value = latest as T;
    latest = undefined;
    hasLatest = false;
    lastEmitAt = Date.now();
    callback(value);
  };

  const cancel = () => {
    if (timer !== null) clearTimeout(timer);
    timer = null;
    latest = undefined;
    hasLatest = false;
    lastEmitAt = Number.NEGATIVE_INFINITY;
  };

  return {
    push: (value: T) => {
      latest = value;
      hasLatest = true;
      const remainingMs = interval() - (Date.now() - lastEmitAt);
      if (remainingMs <= 0) {
        if (timer !== null) clearTimeout(timer);
        emit();
      } else if (timer === null) {
        timer = setTimeout(emit, remainingMs);
      }
    },
    cancel,
  };
}

/** Keep the throttle stable across renders while invoking the latest callback. */
export function useLatestThrottle<T>(
  callback: (value: T) => void,
  intervalMs: number,
): LatestThrottle<T> {
  // The mutable callback/budget belong to the throttle resource, rather than
  // exposing render-written refs. Update them only after React commits.
  const [resource] = useState(() => {
    let committedCallback = callback;
    let committedInterval = intervalMs;
    return {
      throttle: createLatestThrottle<T>(value => committedCallback(value), () => committedInterval),
      update: (nextCallback: (value: T) => void, nextInterval: number) => {
        committedCallback = nextCallback;
        committedInterval = nextInterval;
      },
    };
  });
  const throttle = resource.throttle;
  useLayoutEffect(() => resource.update(callback, intervalMs), [resource, callback, intervalMs]);
  useEffect(() => () => throttle.cancel(), [throttle]);

  // A display-mode change should not leave an old trailing timer running at
  // the previous rate. The next incoming render frame starts the new budget.
  useEffect(() => {
    throttle.cancel();
  }, [intervalMs, throttle]);

  return throttle;
}
