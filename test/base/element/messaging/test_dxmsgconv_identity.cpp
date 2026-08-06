// dxmsgconv identity test (B2) — sensor_id / node_id emission.
//
// Contract:
//   - sensor_id: stable per-stream identity carried on DXFrameMeta._sensor_id
//     (stamped by dxinputselector). dxmsgconv passes frame_meta through to the
//     custom library, which emits it as JSON "sensor_id".
//   - node_id: edge-global identity configured via the dxmsgconv "node-id"
//     element property → GstDxMsgMetaInfo._node_id → JSON "node_id".
//
// Oracle: push one buffer with _sensor_id="cam01" through a dxmsgconv whose
// node-id="edge-001". The default library payload JSON must contain
// "sensor_id":"cam01" and "node_id":"edge-001".

#include <gst/check/gstcheck.h>
#include <gst/check/gstharness.h>
#include <gst/gst.h>
#include "harness_helpers.hpp"
#include "meta_helpers.hpp"
#include "npu_env.hpp"
#include <cstring>
#include <string>

using namespace dxtest;

static const char *CAPS_STR =
    "video/x-raw,format=RGB,width=4,height=4,framerate=30/1";
static std::string MSGCONV_LIB_PATH() {
    return dxtest::resolve_lib_path("libdx_msgconvl.so");
}
#define MSGCONV_LIB (MSGCONV_LIB_PATH().c_str())

static GstBuffer *make_buf_with_sensor(GstClockTime pts, int stream_id,
                                       const char *sensor_id) {
    gsize sz = 4 * 4 * 3;
    GstBuffer *b = gst_buffer_new_allocate(nullptr, sz, nullptr);
    GstMapInfo m; gst_buffer_map(b, &m, GST_MAP_WRITE);
    std::memset(m.data, 0x80, sz); gst_buffer_unmap(b, &m);
    GST_BUFFER_PTS(b) = pts;
    GST_BUFFER_DURATION(b) = GST_SECOND / 30;
    DXFrameMeta *fm = make_frame_meta(b, stream_id, 4, 4);
    fm->_sensor_id = sensor_id;
    add_object_to_frame(fm, 1, 0.9f, 0, 0, 1, 1);
    return b;
}

// Extract string value of "<key>":"<value>" from JSON (first match).
static std::string find_string_field(const std::string &json, const char *key) {
    std::string needle = std::string("\"") + key + "\"";
    auto p = json.find(needle);
    if (p == std::string::npos) return "";
    p = json.find(':', p);
    if (p == std::string::npos) return "";
    p++;
    while (p < json.size() && (json[p] == ' ' || json[p] == '\t')) p++;
    if (p >= json.size() || json[p] != '"') return "";
    p++;
    std::string out;
    while (p < json.size() && json[p] != '"') { out += json[p]; p++; }
    return out;
}

GST_START_TEST(ID_msgconv_sensor_and_node_id) {
    GstElement *e = gst_element_factory_make("dxmsgconv", nullptr);
    g_object_set(e, "library-file-path", MSGCONV_LIB, nullptr);
    g_object_set(e, "node-id", "edge-001", nullptr);
    GstHarness *h = gst_harness_new_with_element(e, "sink", "src");
    fail_unless(h != nullptr);
    gst_harness_set_src_caps_str(h, CAPS_STR);

    GstBuffer *b = make_buf_with_sensor(0, 0, "cam01");
    fail_unless_equals_int(gst_harness_push(h, b), GST_FLOW_OK);

    GstBuffer *out = gst_harness_pull(h);
    fail_unless(out != nullptr);
    GstDxMsgMeta *mm = (GstDxMsgMeta *)gst_buffer_get_meta(
        out, gst_dxmsg_meta_api_get_type());
    fail_unless(mm != nullptr, "buffer missing DxMsgMeta");
    DxMsgPayload *pl = (DxMsgPayload *)mm->_payload;
    fail_unless(pl && pl->_data && pl->_size > 0);
    std::string json((const char *)pl->_data, pl->_size);

    std::string sensor = find_string_field(json, "sensor_id");
    std::string node = find_string_field(json, "node_id");
    fail_unless_equals_string(sensor.c_str(), "cam01");
    fail_unless_equals_string(node.c_str(), "edge-001");

    gst_buffer_unref(out);
    gst_harness_teardown(h);
    gst_object_unref(e);
}
GST_END_TEST;

static Suite *dxmsgconv_identity_suite(void) {
    Suite *s = suite_create("dxmsgconv_identity");
    TCase *tc = tcase_create("identity");
    tcase_set_timeout(tc, 15.0);
    suite_add_tcase(s, tc);
    tcase_add_test(tc, ID_msgconv_sensor_and_node_id);
    return s;
}

GST_CHECK_MAIN(dxmsgconv_identity);
