## Title

fix: sources can be removed, re-added and restarted safely (stream end, stream id reuse, drain bound)

**Type: Bug fix** — no new behaviour except one opt-out property (`eos-drain-timeout`).

## Summary

The stream id is the `dxinputselector` sink pad index, so when an application
removes a source while PLAYING and later requests that pad again, the new
source gets the same id. Several elements kept the previous source's
per-stream state, and the end-of-stream signal itself could be lost.

| Element | Before | After |
|---|---|---|
| dxinputselector | Releasing a pad flushed its queue, dropping an EOS `aggregate()` had not seen yet; a source removed without EOS sent none. Downstream never learned the stream ended. | `release_pad` sends the wrapped per-stream EOS unless already sent (like `PAD_DELETED`/`STREAM_EOS` in other muxers). |
| dxinfer | The EOS latch was never released: every buffer of the next source on the id was dropped forever. | Latch released after the drain; in-flight counter erased only when drained. |
| dxtracker | New source inherited the old tracks and track ids. | Tracker dropped on per-stream EOS. |
| dxmsgconv | New source continued the old seqId. | Counter dropped on per-stream EOS. |
| dxrate | Old stream's last frame never pushed; new source got one frame of the old source. | Stream flushed and its state dropped on per-stream EOS. |
| dxosd | First CAPS per id kept: a smaller new source → `GST_FLOW_ERROR` (stops the shared path). | Every CAPS replaces the stream's info. |
| dxpreprocess | Same first-CAPS rule: warning per frame, tight-stride fallback. | Same fix. |
| dxinputselector | A stream that ended, restarted on the same pad (e.g. its source bin set to NULL and back to PLAYING) and was later released got no end-of-stream: the "already ended" record was never cleared, while GstAggregator clears the pad's EOS on STREAM_START. Downstream state then passed to the next source. | Record cleared on STREAM_START and FLUSH_STOP. |
| dxinputselector | `clip()` stamped DXFrameMeta's size from `gst_pad_get_current_caps()`. GstAggregator clips lazily, when a buffer is consumed, so buffers queued before a resolution change were labelled with the **new** size. | Each stream's caps kept as of the last CAPS event processed in order (as `GstVideoAggregatorPad` does). |
| transforms (`GstSrcFrame`) | Mapped a buffer and read it with the layout it was given, without checking the size. A mislabelled buffer was read past its end (**SIGSEGV in libyuv scaling in dxpreprocess**). | Refuses a buffer smaller than its layout, as `gst_video_frame_map()` does; dxpreprocess skips the frame with a warning. |

One commit per defect; each can be reviewed and taken on its own, except
that the element fixes rely on the per-stream EOS actually arriving (commit 1).

The last three were found by field-testing the first five on a 16-camera
pipeline: removing a camera that had been restarted once left its tracker and
sequence state to the next camera on that stream id, and a source whose
resolution changed mid-stream crashed the process within seconds.

### Bounded drain (dxinfer)

Both EOS drains (per-stream and global) waited without a bound on the
streaming thread. A device that stops completing work pinned that thread
forever, and on the global EOS — the shutdown path — the pipeline could not be
torn down.

New property `eos-drain-timeout` (ns, default 5 s, `0` = previous unbounded
wait). On expiry the EOS is forwarded with a WARNING. The backend `Put()`
backpressure wait is deliberately left unbounded: blocking there under load is
the intended degradation (an upstream leaky queue drops).

Touches the same drain predicate as the dxinfer fix above, so it is part of this PR.

## Verification

Base `dev` @ `a7a95c9`. New tests fail on dev and pass here:

| test | dev | this branch |
|---|---|---|
| test_dxinputselector_release_eos | 2/4 fail (release w/o EOS, EOS then immediate release: 0 EOS) | 4/4 |
| test_dxinfer_per_stream_eos | 3/4 fail | 4/4 |
| test_dxtracker_eviction / test_dxmsgconv_eviction | fail (seqId 3) | pass |
| test_dxrate_per_stream_eos | fail | pass |
| test_dxosd_caps_replace | 2/2 fail (flow -5) | 2/2 |
| test_dxpreprocess_caps_replace | fail (width 1920 ≠ 640) | pass |
| test_dxinputselector_release_eos `release_after_restart_ends_the_new_stream` | fail (1 EOS, expected 2) | pass |
| test_dxinputselector `CE_inputsel_caps_change_labels_queued_buffers` | fail (48-byte buffer labelled 8x8) | pass |
| test_dxpreprocess_short_buffer | fail (short buffer preprocessed as a full frame) | pass |

Field: 16 RTSP sources, sources restarted and removed/re-attached on reused
ids (sequence numbers restart at 1), a source switching 640x360 ↔ 1280x720
every 6 s (7 switches, no crash, 2,791 boxes all inside the frame, other
streams unaffected).

All existing element and metadata tests: identical results on dev and this
branch. Not run here: pipeline tests (no sample models on this machine) and the
valgrind pass for `test_dxpreprocess_wrapped_caps` (valgrind not installed).

Local test note: `meta_helpers.hpp` includes the installed headers, and
`resolve_lib_path()` uses `/usr/local`. Run with `INSTALL_PREFIX` pointing at an
install of the tree under test, or struct layouts can mismatch.

`test_dxinfer_eos_drain_timeout` (white-box, stub backend): dev 0/4 (property
missing) → 4/4. Replacing the bounded wait with the unbounded one fails the two
bounded cases (blocked > 2 s on a 200 ms timeout).
