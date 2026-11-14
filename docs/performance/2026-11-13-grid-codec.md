# Shared occupancy-grid decoding

Canonical int8 cell values are decoded once for a delta subscription and reused for full frames and resync. Full-only subscriptions use the same scanner; other JSON retains the generic decoder. Topic rates, patch decisions, sequence/base IDs, recovery and compression settings are unchanged.

## Paired component measurements

Untouched dev `687260fbd66cf64f689e472fd172fe16ede735b5` versus the grid-only implementation `a3fae9a59200d03e647b0d30bd57b1f03769b024` (the production code measured here). No CDR/JSON optimization is included in either arm. Intel i7-10700K, Go 1.25.10, GOMAXPROCS=2, process affinity CPUs 2,3, nice 19, 1 GiB Go soft memory limit. Five pairs ×32 operations; rotated/reversed arm order with the same input bytes/seeds in each pair. The two relevant arms are included in [the raw table](2026-11-13-grid-codec.csv); an additional historical implementation was interleaved as a control.

| Component / fixture | Median before → after | Paired time reduction, range | Allocated bytes/op before → after |
|---|---:|---:|---:|
| delta_300k | 28.821 → 6.365 ms | 77.6–78.6% | 1,539,380 → 304,507 |
| delta_1m | 98.281 → 21.513 ms | 77.4–78.8% | 5,242,166 → 1,009,039 |
| full_stripes_300k | 40.456 → 5.614 ms | 85.2–86.6% | 29,623,816 → 914,913 |
| full_lidar_300k | 50.123 → 6.745 ms | 86.0–87.7% | 33,445,843 → 914,909 |
| full_random_300k | 52.786 → 7.815 ms | 85.1–86.2% | 33,985,612 → 914,893 |
| full_late_signed_zero_300k | 40.843 → 45.354 ms | -14.4–-5.9% | 29,623,844 → 29,927,651 |
| full_late_128_300k | 40.585 → 45.593 ms | -19.3–-12.0% | 29,623,833 → 29,927,637 |

Delta measurements include JSON decoding and the unchanged-cell diff, before MessagePack encoding or I/O. Each 32-operation arm produced 2 full-frame decisions and 30 patches. Full-frame measurements include JSON→MessagePack encoding before compression/I/O. Fixtures are synthetic: LiDAR-like unchanged grids use seed 971; full-frame distributions use seed 42. These are not recorded robot map sizes.

At the producer default of one mow-progress message per 2 seconds, the measured delta savings amortize to about 11.2 ms of component elapsed time per second and subscription for 300k cells, or 38.4 ms/s for 1M. This elapsed-time calculation is not a CPU-utilization measurement; the benchmark can include GC and scheduling effects with GOMAXPROCS=2. Subscription count matters because each subscription has its own delta state.

## Costs and compatibility

A late `-0` or 128 cell uses the original generic fallback. The extra scan costs about 4.5–5 ms per full frame in these cases (roughly 11–12% by median); those representations are not emitted by the CDR int8 producer. This is not a faster decoder for arbitrary JSON. Exact-value tests cover fallback, metadata, numeric bits, aliases, duplicates and resync. Existing CDR→JSON precision/sanitizer semantics are unchanged; no new outlier filtering is introduced.

No network-byte, TCP-packet, reconnect, WiFi or physical-robot improvement is claimed. No Pi/ARM or field measurement was taken.

## Reproduce the component comparisons

From `gui/`:

```sh
GOMAXPROCS=2 go test ./pkg/api -run "^$" \
  -bench "^BenchmarkGrid(DeltaDecode|FullFrame)$" -benchtime=32x -count=5
```

These portable benchmarks use the retained frozen original state machine and original generic MessagePack reference in the same binary. They allow reproducing the component comparison from the repository alone. The raw CSV above came from separate baseline/candidate test binaries with the same fixture functions; identical absolute timings are not expected across those layouts or hosts. The private Linux packet-loss/capture harness is deliberately excluded from the repository.
