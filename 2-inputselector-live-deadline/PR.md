## Title

fix(inputselector): let live sources aggregate on a deadline

**Type: Bug fix** — one silent live source no longer stalls every other source. Behaviour change for live pipelines: strict cross-pad PTS order is relaxed for the round in which a pad is skipped (non-live pipelines unchanged).

## Summary

With live sources, **one silent input stops every stream behind the same
`dxinputselector`.** It merges streams in PTS order and needs a candidate buffer
on every sink pad; when a pad has none, `aggregate()` returns `NEED_DATA` and
waits indefinitely. One camera dropping (cable, encoder hang, RTSP server
restart) therefore stops detection on all cameras sharing the inference path.

`GstAggregator` already defines the answer for live pipelines: wait on the clock
for the pipeline latency, then call `aggregate()` with `timeout=TRUE` so the
subclass can leave out pads that missed the deadline (*"if a pad does not have a
buffer in time we ignore that pad"*, gstaggregator.c). Two things kept that from
working:

1. No `get_next_time`, so the base class had no deadline and never set `timeout`.
   The inherited `latency` property was a no-op.
2. `aggregate()` ignored its `timeout` argument.

## Changes

- `get_next_time` returns the running time of the earliest buffer currently held
  (the shape of `gst_base_ts_mux_get_next_time()`), and `GST_CLOCK_TIME_NONE`
  while no pad holds one, so the srcpad task sleeps until data arrives.
- `aggregate()` skips pads without a buffer when `timeout` is set.

`gst_aggregator_simple_get_next_time()` is **not** usable here: it reads the
srcpad segment position, which this element never advances. The deadline then
lies permanently in the past and the srcpad task spins at 100% of a core. The
regression test below catches that.

## Behaviour change

- **Live only.** Once the deadline passes, strict cross-pad PTS order is relaxed
  for that round. A skipped pad's buffer is not dropped; it goes out in a later
  round.
- **Non-live pipelines are unchanged** — `timeout` is never TRUE without a clock.
- The deadline is the pipeline latency: upstream latency + this element's
  `latency` property (default 0). With `rtspsrc` upstream that is the
  jitterbuffer latency. Applications wanting a tighter bound set `latency`
  (e.g. 33 ms at 30 fps).

## Verification

Base: `dev` @ `a7a95c9`. I checked on that tree that `aggregate()` still ignores
`timeout` and there is no `get_next_time`.

New `test_dxinputselector_live_deadline` (3 cases, no NPU):

| case | dev @ a7a95c9 | first version (srcpad-segment deadline) | this branch |
|---|---|---|---|
| live, one pad silent → the other pad's buffers come out | **FAIL: 0 of 30** | pass | pass |
| live, no data → srcpad task sleeps | pass | **FAIL: 1.00 s CPU per 1 s idle** | pass |
| non-live, one pad silent → still waits (unchanged) | pass | pass | pass |

Stable over 5 consecutive runs. Existing multistream tests: identical results on
dev and this branch.

Existing `test_dxinputselector` (10/10) and `test_dxinputselector_release_pad`
pass on both.

## Field test

Integration build = `dev` @ `a7a95c9` + PRs 1–4 of this series, plus
application-specific changes that are not part of these PRs. DX-M1,
16 live RTSP cameras (640x360, inference 10 fps each), one shared
`dxinputselector -> dxpreprocess -> dxinfer -> dxpostprocess -> dxtracker ->
dxmsgconv -> dxmsgbroker` path. Measured as Kafka messages per second
(2026-09-28).

| step | result |
|---|---|
| start | 16 cameras publishing after 20 s, 165.5 msg/s |
| freeze one camera's RTSP publisher (SIGSTOP) for 13 s | other 15 cameras: **156.7 msg/s = 100.8 % of baseline** |
| resume; the application detects the stall, tears the source down and re-attaches it on the same pad | frozen camera publishing again **25 s after resume** (10.0 msg/s) |

Without these fixes the same procedure stopped all 16 cameras for the
duration of the freeze (0.0 msg/s), and the re-attached camera never
published again until the process was restarted.
