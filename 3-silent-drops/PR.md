## Title

feat(infer,msgbroker,inputselector): stop hiding silent drops — count them, and fix the broker failure streak

**Type: Feature (includes 1 bug fix)** — commit 1 is a bug fix in dxmsgbroker; commits 2–3 add the `stats` property. They are one PR because both are about drops that returned `GST_FLOW_OK` and went unnoticed, and the stats commit counts the same render paths the fix changes.

## Bug fix — dxmsgbroker failure streak

`render()` counts consecutive publish failures and raises an element error at
`MAX_CONSECUTIVE_FAILURES` (10). The counter was reset at the end of **every**
`render()`, including buffers that carry no `DxMsgMeta` and publish nothing —
the normal result of `dxmsgconv message-interval > 1`, and what a convert library
that fails for a frame leaves behind.

Interleaved with failing publishes, such buffers keep the streak below the limit
indefinitely: the broker can fail every single publish and the error never fires.

## Feature — `stats`

These elements discard buffers while returning `GST_FLOW_OK`, so from outside
a frame dropped by design (QoS throttling, `message-interval`) and a frame lost
to a fault look the same. Each gets a read-only `stats` property — a
`GstStructure` of `guint64` counters, the convention of `identity`,
`rtpjitterbuffer` and `rtpsession`: one read is a consistent snapshot, and a
later counter does not change the API.

| Element | Structure | Fields |
|---|---|---|
| dxinfer | `application/x-dxinfer-stats` | `dropped-qos`, `dropped-throttled`, `dropped-no-meta`, `dropped-after-eos` |
| dxmsgbroker | `application/x-dxmsgbroker-stats` | `published`, `failed`, `no-message` |
| dxinputselector | `application/x-dxinputselector-stats` | `pads-skipped` (live deadline) |

Counters reset when the element starts. Documented in the element pages.

**Depends on** PR 1 (dxinfer drain) and PR 2 (live deadline — the skip it counts).

## Verification

Base: `dev` @ `a7a95c9`.

| `test_dxmsgbroker_failure_streak` (new, stub broker) | dev @ a7a95c9 | this branch |
|---|---|---|
| 10 failing publishes interleaved with message-less buffers → error on the 10th | **FAIL: GST_FLOW_OK throughout** | pass |
| a successful publish still ends the streak | pass | pass |

Existing `test_dxmsgbroker`: 6/6 on both.

Not reachable with the default `message-interval=1` and a convert library that
always produces a message, which is why it has not shown up in practice.

New `test_dxinfer_stats` (5 cases, white-box, stub backend) and stats cases in
`test_dxmsgbroker_failure_streak` and `test_dxinputselector_live_deadline`.
All fail before (no `stats` property) and pass here. Field: read every 5 s
from Python (`get_property("stats").get_uint64(...)`) on a 16-camera pipeline
for 24 h.
