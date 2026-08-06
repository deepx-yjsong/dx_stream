// dxmsgconv per-stream EOS eviction test — B1 (Topology-B per-stream
// lifecycle, design 2026-06-16-topology-b-protobuf-design.md §4.1)
//
// Contract (CLAUDE.md C.5 / E.6 / Part G): a single dxmsgconv used inside the
// dxvideoraw domain keeps per-stream state (`_seq_ids`, std::map<int,guint64>).
// When a stream ends, dxinputselector emits an L2 wrapped per-stream EOS
// (CUSTOM_DOWNSTREAM, application/x-dx-wrapped-event, inner=EOS). dxmsgconv
// must consume it and ERASE that stream's `_seq_ids` entry so a later buffer
// reusing the same stream_id starts fresh (no leak, reuse-safe).
//
// Defect (pre-B1): dxmsgconv has NO sink_event handler at all, so the per-stream
// entry is never erased — leak under camera churn, and stale seq on stream_id
// reuse.
//
// Oracle (black-box): default library emits {"streamId":N,"seqId":M,...}.
//   stream 1 first buffer  -> seqId 1
//   stream 1 second buffer -> seqId 2
//   wrapped-EOS(stream 1)  -> entry erased
//   stream 1 third buffer  -> seqId 1 again (reset)
// Without eviction the third buffer would emit seqId 3.

#include <gst/check/gstcheck.h>
#include <gst/check/gstharness.h>
#include <gst/gst.h>
#include "harness_helpers.hpp"
#include "meta_helpers.hpp"
#include "npu_env.hpp"
#include "utils.hpp"
#include <cstring>
#include <string>
#include <vector>

using namespace dxtest;

static const char *CAPS_STR =
    "video/x-raw,format=RGB,width=4,height=4,framerate=30/1";
static std::string MSGCONV_LIB_PATH() {
    return dxtest::resolve_lib_path("libdx_msgconvl.so");
}
#define MSGCONV_LIB (MSGCONV_LIB_PATH().c_str())

static GstBuffer *make_buf_with_meta(GstClockTime pts, int stream_id) {
    gsize sz = 4 * 4 * 3;
    GstBuffer *b = gst_buffer_new_allocate(nullptr, sz, nullptr);
    GstMapInfo m; gst_buffer_map(b, &m, GST_MAP_WRITE);
    std::memset(m.data, 0x80, sz); gst_buffer_unmap(b, &m);
    GST_BUFFER_PTS(b) = pts;
    GST_BUFFER_DURATION(b) = GST_SECOND / 30;
    DXFrameMeta *fm = make_frame_meta(b, stream_id, 4, 4);
    add_object_to_frame(fm, 1, 0.9f, 0, 0, 1, 1);
    return b;
}

// Extract integer value of "<key>": from a JSON snippet (first match).
static int find_int_field(const std::string &json, const char *key) {
    std::string needle = std::string("\"") + key + "\"";
    auto p = json.find(needle);
    if (p == std::string::npos) return -1;
    p = json.find(':', p);
    if (p == std::string::npos) return -1;
    p++;
    while (p < json.size() && (json[p] == ' ' || json[p] == '\t')) p++;
    int sign = 1;
    if (p < json.size() && json[p] == '-') { sign = -1; p++; }
    int val = 0;
    bool any = false;
    while (p < json.size() && json[p] >= '0' && json[p] <= '9') {
        val = val * 10 + (json[p] - '0'); p++; any = true;
    }
    return any ? sign * val : -1;
}

static int pull_seq_id(GstHarness *h) {
    GstBuffer *out = gst_harness_pull(h);
    fail_unless(out != nullptr, "expected an output buffer");
    GstDxMsgMeta *mm = (GstDxMsgMeta *)gst_buffer_get_meta(
        out, gst_dxmsg_meta_api_get_type());
    fail_unless(mm != nullptr, "output buffer missing DxMsgMeta");
    DxMsgPayload *pl = (DxMsgPayload *)mm->_payload;
    fail_unless(pl && pl->_data && pl->_size > 0);
    std::string json((const char *)pl->_data, pl->_size);
    int seq = find_int_field(json, "seqId");
    gst_buffer_unref(out);
    return seq;
}

GST_START_TEST(EV_msgconv_per_stream_eos_evicts_seq) {
    GstElement *e = gst_element_factory_make("dxmsgconv", nullptr);
    g_object_set(e, "library-file-path", MSGCONV_LIB, nullptr);
    GstHarness *h = gst_harness_new_with_element(e, "sink", "src");
    fail_unless(h != nullptr);
    gst_harness_set_src_caps_str(h, CAPS_STR);

    // Two buffers on stream 1 → seqId 1, 2.
    fail_unless_equals_int(
        gst_harness_push(h, make_buf_with_meta(0, 1)), GST_FLOW_OK);
    int first = pull_seq_id(h);
    fail_unless_equals_int(
        gst_harness_push(h, make_buf_with_meta(GST_SECOND / 30, 1)), GST_FLOW_OK);
    int second = pull_seq_id(h);
    fail_unless(first == 1 && second == 2,
                "baseline seq must be 1,2 (got %d,%d)", first, second);

    // L2 wrapped per-stream EOS for stream 1.
    GstEvent *wrapped = dx_event_wrap_downstream(1, gst_event_new_eos());
    fail_unless(gst_harness_push_event(h, wrapped),
                "wrapped per-stream EOS push must succeed");

    // Reuse stream 1 → seqId must reset to 1 (entry was erased).
    fail_unless_equals_int(
        gst_harness_push(h, make_buf_with_meta(2 * GST_SECOND / 30, 1)),
        GST_FLOW_OK);
    int third = pull_seq_id(h);

    fail_unless(third == 1,
                "after wrapped per-stream EOS(1), stream 1 seqId must reset to "
                "1 (got %d) — dxmsgconv did not erase _seq_ids[1]", third);

    gst_harness_teardown(h);
    gst_object_unref(e);
}
GST_END_TEST;

static Suite *dxmsgconv_eviction_suite(void) {
    Suite *s = suite_create("dxmsgconv_eviction");
    TCase *tc = tcase_create("eviction");
    tcase_set_timeout(tc, 15.0);
    suite_add_tcase(s, tc);
    tcase_add_test(tc, EV_msgconv_per_stream_eos_evicts_seq);
    return s;
}

GST_CHECK_MAIN(dxmsgconv_eviction);
