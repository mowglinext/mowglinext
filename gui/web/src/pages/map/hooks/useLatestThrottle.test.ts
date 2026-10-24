import {StrictMode} from "react";
import {act, renderHook} from "@testing-library/react";
import { afterEach, beforeEach, describe, expect, it, vi } from "vitest";
import { createLatestThrottle, useLatestThrottle } from "./useLatestThrottle.ts";

describe("createLatestThrottle", () => {
  beforeEach(() => {
    vi.useFakeTimers();
    vi.setSystemTime(0);
  });

  afterEach(() => {
    vi.useRealTimers();
  });

  it("emits the leading value and the latest trailing value", () => {
    const emit = vi.fn();
    const throttle = createLatestThrottle<number>(emit, 100);

    for (let value = 0; value < 100; value += 1) {
      throttle.push(value);
    }

    expect(emit).toHaveBeenCalledTimes(1);
    expect(emit).toHaveBeenLastCalledWith(0);

    vi.advanceTimersByTime(100);
    expect(emit).toHaveBeenCalledTimes(2);
    expect(emit).toHaveBeenLastCalledWith(99);
  });

  it("caps a sustained 100 Hz stream at about 10 Hz", () => {
    const emit = vi.fn();
    const throttle = createLatestThrottle<number>(emit, 100);

    for (let value = 0; value < 100; value += 1) {
      throttle.push(value);
      vi.advanceTimersByTime(10);
    }

    expect(emit.mock.calls.length).toBeGreaterThanOrEqual(10);
    expect(emit.mock.calls.length).toBeLessThanOrEqual(11);
    expect(emit).toHaveBeenLastCalledWith(99);
  });

  it("caps a sustained 100 Hz stream at about 5 Hz", () => {
    const emit = vi.fn();
    const throttle = createLatestThrottle<number>(emit, 200);

    for (let value = 0; value < 100; value += 1) {
      throttle.push(value);
      vi.advanceTimersByTime(10);
    }

    expect(emit.mock.calls.length).toBeGreaterThanOrEqual(5);
    expect(emit.mock.calls.length).toBeLessThanOrEqual(6);
    expect(emit).toHaveBeenLastCalledWith(99);
  });

  it("uses the latest budget when the display mode changes", () => {
    const emit = vi.fn();
    let intervalMs = 100;
    const throttle = createLatestThrottle<number>(emit, () => intervalMs);

    throttle.push(1);
    intervalMs = 50;
    vi.advanceTimersByTime(50);
    throttle.push(2);

    expect(emit).toHaveBeenCalledTimes(2);
    expect(emit).toHaveBeenLastCalledWith(2);
  });

  it("drops a pending trailing value when cancelled", () => {
    const emit = vi.fn();
    const throttle = createLatestThrottle<number>(emit, 100);

    throttle.push(1);
    throttle.push(2);
    throttle.cancel();
    vi.advanceTimersByTime(100);

    expect(emit).toHaveBeenCalledTimes(1);
    expect(emit).toHaveBeenLastCalledWith(1);
  });
});


describe('useLatestThrottle committed callbacks and cleanup', () => {
  beforeEach(() => { vi.useFakeTimers(); vi.setSystemTime(0); });
  afterEach(() => vi.useRealTimers());

  it('delivers the trailing frame to the latest callback without resetting the throttle', () => {
    const first = vi.fn();
    const next = vi.fn();
    const {result, rerender, unmount} = renderHook(({callback, interval}) => useLatestThrottle(callback, interval),
      {initialProps: {callback: first, interval: 100}, wrapper: StrictMode});
    const throttle = result.current;
    act(() => { throttle.push(1); throttle.push(2); });
    rerender({callback: next, interval: 100});
    expect(result.current).toBe(throttle);
    act(() => { vi.advanceTimersByTime(100); });
    expect(first).toHaveBeenCalledExactlyOnceWith(1);
    expect(next).toHaveBeenCalledExactlyOnceWith(2);
    act(() => { throttle.push(3); });
    unmount();
    act(() => { vi.advanceTimersByTime(100); });
    expect(next).toHaveBeenCalledOnce();
  });

  it('cancels the old trailing frame on a display budget transition', () => {
    const callback = vi.fn();
    const {result, rerender} = renderHook(({interval}) => useLatestThrottle(callback, interval), {initialProps: {interval: 100}});
    act(() => { result.current.push(1); result.current.push(2); });
    rerender({interval: 50});
    act(() => { vi.advanceTimersByTime(100); });
    expect(callback).toHaveBeenCalledExactlyOnceWith(1);
    act(() => { result.current.push(3); });
    expect(callback).toHaveBeenLastCalledWith(3);
  });
});
