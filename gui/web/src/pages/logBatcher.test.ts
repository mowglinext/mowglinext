import { afterEach, beforeEach, describe, expect, it, vi } from "vitest";
import { appendCappedBatch, createLogBatcher } from "./logBatcher.ts";

describe("appendCappedBatch", () => {
  it("appends a batch without changing the previous array", () => {
    const previous = [1, 2];
    const result = appendCappedBatch(previous, [3, 4], 10);

    expect(result).toEqual([1, 2, 3, 4]);
    expect(previous).toEqual([1, 2]);
  });

  it("keeps only the newest values at the cap", () => {
    expect(appendCappedBatch([1, 2, 3], [4, 5, 6], 4)).toEqual([3, 4, 5, 6]);
    expect(appendCappedBatch([1, 2], [3, 4, 5, 6, 7], 3)).toEqual([5, 6, 7]);
  });
});

describe("createLogBatcher", () => {
  beforeEach(() => {
    vi.useFakeTimers();
  });

  afterEach(() => {
    vi.useRealTimers();
  });

  it("flushes a burst in one batch", () => {
    const onBatch = vi.fn<Parameters<typeof createLogBatcher<number>>[0]>();
    const batcher = createLogBatcher<number>(onBatch, 100);

    for (let value = 0; value < 100; value += 1) batcher.push(value);
    expect(onBatch).not.toHaveBeenCalled();

    vi.advanceTimersByTime(100);
    expect(onBatch).toHaveBeenCalledTimes(1);
    expect(onBatch).toHaveBeenCalledWith(
      Array.from({ length: 100 }, (_, index) => index),
    );
  });

  it("reduces a sustained 100 Hz stream to 10 batches per second", () => {
    const onBatch = vi.fn<Parameters<typeof createLogBatcher<number>>[0]>();
    const batcher = createLogBatcher<number>(onBatch, 100);

    for (let value = 0; value < 100; value += 1) {
      batcher.push(value);
      vi.advanceTimersByTime(10);
    }

    expect(onBatch).toHaveBeenCalledTimes(10);
    expect(onBatch.mock.calls.flatMap(([batch]) => batch)).toEqual(
      Array.from({ length: 100 }, (_, index) => index),
    );
  });

  it("drops pending values when reset", () => {
    const onBatch = vi.fn<Parameters<typeof createLogBatcher<number>>[0]>();
    const batcher = createLogBatcher<number>(onBatch, 100);

    batcher.push(1);
    batcher.reset();
    vi.advanceTimersByTime(100);
    expect(onBatch).not.toHaveBeenCalled();

    batcher.push(2);
    vi.advanceTimersByTime(100);
    expect(onBatch).toHaveBeenCalledWith([2]);
  });
});
