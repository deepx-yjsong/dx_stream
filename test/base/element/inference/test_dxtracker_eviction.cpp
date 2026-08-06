// dxtracker per-stream EOS eviction test — B1 (Topology-B per-stream
// lifecycle, design 2026-06-16-topology-b-protobuf-design.md §4.1)
//
// Contract (CLAUDE.md C.5 / E.6 / Part G): a single dxtracker used inside the
// dxvideoraw domain keeps a per-stream tracker instance map
// (`_trackers`, std::map<int, unique_ptr<Tracker>>). When a stream ends,
// dxinputselector emits an L2 wrapped per-stream EOS (CUSTOM_DOWNSTREAM,
// application/x-dx-wrapped-event, inner=EOS). dxtracker must consume it and
// ERASE that stream's tracker so a later buffer reusing the same stream_id gets
// a fresh OC_SORT instance (no leak, no stale tracks).
//
// Defect (pre-B1): dxtracker's sink_event only clears ALL trackers on
// FLUSH_STOP; there is no per-stream erase — leak under camera churn.
//
// Oracle (black-box): OC_SORT id_count is per-instance. A fresh tracker fed a
// single static box over N identical frames produces a deterministic surviving
// track_id (the same value regardless of the box coordinates, since id
// assignment depends only on track-creation order).
//   phase 1 (stream 1, box A)           -> surviving track_id == id_a
//   wrapped-EOS(stream 1)               -> tracker evicted
//   phase 2 (stream 1, box B, far away) -> surviving track_id == id_b
// Evicted: phase 2 runs on a FRESH tracker reproducing the same sequence, so
//   id_b == id_a.
// Not evicted: the same tracker survives; its id_count is already past id_a, and
//   box B is a new non-matching track, so id_b > id_a (id_b != id_a).

#include <gst/check/gstcheck.h>
#include <gst/check/gstharness.h>
#include <gst/gst.h>
#include "harness_helpers.hpp"
#include "meta_helpers.hpp"
#include "utils.hpp"

using namespace dxtest;

static const char *CAPS_320 =
    "video/x-raw,format=RGB,width=320,height=240,framerate=30/1";
static const guint BUF_SIZE = 320 * 240 * 3;

// Feed `frames` buffers of the same box on `stream_id`, return the track_id of
// the (single) surviving object on the last frame (-1 if none survived).
static int feed_box_and_last_track_id(GstHarness *h, int stream_id,
                                      float bx, float by, float bw, float bh,
                                      int frames, GstClockTime base_pts) {
    int last_id = -1;
    for (int i = 0; i < frames; i++) {
        GstBuffer *b = gst_harness_create_buffer(h, BUF_SIZE);
        GST_BUFFER_PTS(b) = base_pts + (GstClockTime)i * GST_SECOND / 30;
        GST_BUFFER_DURATION(b) = GST_SECOND / 30;
        DXFrameMeta *fm = make_frame_meta(b, stream_id, 320, 240);
        add_object_to_frame(fm, 0, 0.9f, bx, by, bw, bh);
        fail_unless_equals_int(gst_harness_push(h, b), GST_FLOW_OK);

        GstBuffer *out = gst_harness_pull(h);
        fail_unless(out != nullptr, "frame %d must produce output", i);
        DXFrameMeta *ofm = dx_get_frame_meta(out);
        fail_unless(ofm != nullptr);
        if (!ofm->_object_meta_list.empty())
            last_id = ofm->_object_meta_list[0]->_track_id;
        gst_buffer_unref(out);
    }
    return last_id;
}

GST_START_TEST(EV_tracker_per_stream_eos_evicts_tracker) {
    Harness h("dxtracker");
    gst_harness_set_src_caps_str(h.h, CAPS_320);

    // Phase 1: stream 1, box A → fresh tracker assigns some track_id id_a.
    int id_a = feed_box_and_last_track_id(h.h, 1, 10, 10, 50, 50, 5, 0);
    fail_unless(id_a >= 1,
                "phase 1: a stable track must be assigned an id (got %d)", id_a);

    // L2 wrapped per-stream EOS for stream 1 → evict its tracker.
    GstEvent *wrapped = dx_event_wrap_downstream(1, gst_event_new_eos());
    fail_unless(gst_harness_push_event(h.h, wrapped),
                "wrapped per-stream EOS push must succeed");

    // Phase 2: reuse stream 1 with a far-away box B.
    int id_b = feed_box_and_last_track_id(h.h, 1, 250, 180, 300, 230, 5,
                                          10 * GST_SECOND / 30);

    fail_unless(id_b == id_a,
                "after wrapped per-stream EOS(1), reused stream 1 must get a "
                "FRESH tracker — expected track_id %d (same as a fresh tracker), "
                "got %d — dxtracker did not erase _trackers[1]", id_a, id_b);

    // h destructed by Harness dtor.
}
GST_END_TEST;

static Suite *dxtracker_eviction_suite(void) {
    Suite *s = suite_create("dxtracker_eviction");
    TCase *tc = tcase_create("eviction");
    tcase_set_timeout(tc, 20.0);
    suite_add_tcase(s, tc);
    tcase_add_test(tc, EV_tracker_per_stream_eos_evicts_tracker);
    return s;
}

GST_CHECK_MAIN(dxtracker_eviction);
