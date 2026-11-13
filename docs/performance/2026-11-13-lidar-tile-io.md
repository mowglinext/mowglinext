# Packed LiDAR tile I/O — 2026-11-13

Baseline: unchanged dev `6445b2c014720124e68cd96395e44d1d78491522`. Host: Kuffe, Intel i7-10700K, Linux x86_64, g++ 15.2. Production change: one packed row per stream operation instead of two operations per cell, with identical on-disk format and evidence validation.

## Operation and method

The format remains native little-endian IEEE 754 double + one touched byte per cell, no struct padding. Standard row buffer 900 bytes; constructor bounds keep it under 18 KiB. Header, length/datum, per-cell evidence validation, flush/close/rename, async ownership and retry semantics remain unchanged. Default 100×100 tiles go from 20,000 payload stream calls/tile to 100.

Complete public operations are timed: `RequestSave` to ready, new instance + first `UpdateWindow` to ready, and an overlapping one-tile window shift to ready. Worker launch, allocations, I/O, validation and 50 µs sleeping polling are included. Same local SSD/warm page cache; new instance does not mean cold device cache. No system cache flush.

Both sides use identical harness/dependencies and `-O3 -DNDEBUG -std=c++17 -pthread`, CPU 2/nice 19, serialized compute. Independent verification rebuilt in project C++20. Seeds 101/202 train, 733/991 held out; two balanced A/B orders per seed, two warmups and four measured repetitions give 32 samples/version/operation/scenario. Paired block medians, not correlated within-process samples, are the comparison unit.

`dense`: all 25 default 100² tiles in 500² window at 0.1 m. `insert`: twenty 360-beam scans through actual mapper Insert, 3–11.99 m ranges and moving poses. `sparse`: only central tile populated. Heldout geometries: `fine` 0.05 m/1000² window and `radius3` 700²/49 tiles.

[All 120 paired operation block medians](2026-11-13-lidar-tile-io-paired.csv).

## Results

The baseline/final entries below are medians of the paired block medians in the CSV. Reduction is one minus their ratio; the last column is the range of individual paired-block reductions.

| Scenario | Operation | Baseline ms | Final ms | Reduction | Paired reduction range |
|---|---|---:|---:|---:|---:|
| dense | save | 7.7250 | 3.0266 | 60.8% | 58.3–62.4% |
| dense | load | 6.3093 | 1.0479 | 83.4% | 83.0–84.1% |
| dense | shift | 12.7859 | 3.8360 | 70.0% | 69.6–71.0% |
| insert | save | 3.9888 | 1.6858 | 57.7% | 55.9–59.2% |
| insert | load | 3.1920 | 0.6290 | 80.3% | 79.9–81.1% |
| insert | shift | 6.6714 | 2.1888 | 67.2% | 65.9–68.0% |
| sparse | save | 0.6576 | 0.4555 | 30.7% | 23.5–38.3% |
| sparse | load | 0.4243 | 0.2185 | 48.5% | 32.0–49.3% |
| sparse | shift | 1.0181 | 0.6154 | 39.6% | 35.5–47.2% |
| fine | save | 26.1994 | 7.7369 | 70.5% | 68.1–71.4% |
| fine | load | 24.3562 | 3.7468 | 84.6% | 83.5–85.2% |
| fine | shift | 46.0099 | 10.8745 | 76.4% | 75.7–76.9% |
| radius3 | save | 14.9335 | 5.8977 | 60.5% | 58.9–64.1% |
| radius3 | load | 12.4099 | 1.9813 | 84.0% | 83.2–84.5% |
| radius3 | shift | 25.7274 | 7.6515 | 70.3% | 68.8–71.7% |

Sparse tiles remain below 50%. The store hides its mapper while async work runs; shorter operations may reduce that gap, but production polling can still expose the result at the same next timer tick. No fewer missed updates or improved localization was measured.

## Exactness and independent verification

All 20 store tests pass on unchanged baseline and candidate, including independent scalar-format golden encoding across nine tile offsets/rows/signed zero and malformed later-row evidence. Candidate also passes 20 tests under ASan+UBSan. Cross-binary base→candidate and candidate→base each verify 250,000 exact evidence cells and 26 identical manifest/tile SHA256 hashes after rewrite.

Independent C++20 builds repeated 20 tests/version and the cross-load checks. Fresh seeds 1709/2609 across all five scenarios give 20 additional A/B run pairs. Dense save/load/shift medians were 7.692→2.881 ms, 6.545→1.042 ms, 13.046→3.720 ms (62.6/84.1/71.5%). Insert reductions 59.1/80.5/68.0%. All non-sparse paired medians exceeded 50%; sparse wall reductions 31.5/48.0/45.6% did not.

A second independent publication review rebuilt both C++20 versions: all 20 store tests pass on each. Six new adversarial geometry/evidence cases (1/7/17/31/100/682 cells per tile, radii 1/2/3/8, negative/near-limit coordinates, signed zero/subnormal values) preserve exact evidence bits in both directions and all 412 compared files. Both versions reject the same 42 corruption variants and an injected mid-read truncation. UBSan passes all 20 tests plus six cases.

## Reproduction and limits

Build and run the existing `test_lidar_submap_store` target in a configured Release ROS workspace; its two added tests validate the legacy packed representation independently. The timing harness wraps only the public API operations described above, keeping map generation/population outside timing. The paired CSV records all seeds, orders and medians for auditing the table.

Focused actual-source builds/tests were run; full workspace CI remains required. Changed C++ lines pass clang-format 18. No ARM, robot, field, power-loss/cold-storage, whole-robot CPU or mission-time validation was performed. Existing persistence behavior is preserved; no new durability guarantee is asserted. Exact tested bytes/evidence do not constitute universal bitwise proof across architectures.
