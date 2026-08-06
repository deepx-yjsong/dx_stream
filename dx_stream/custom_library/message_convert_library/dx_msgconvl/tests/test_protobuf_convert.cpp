#include "dx_msgconvl_priv.hpp"
#include "dx_frame.pb.h"
#include "gstdxstream/gst-dxframemeta.hpp"
#include "gstdxstream/gst-dxobjectmeta.hpp"

#include <glib.h>
#include <cassert>
#include <cstdio>

int main() {
    DXObjectMeta obj{};
    obj._label = 1;
    obj._track_id = 42;
    obj._confidence = 0.87f;
    obj._label_name = "person";
    obj._box = {300.f, 400.f, 500.f, 600.f};

    DXFrameMeta fm{};
    fm._stream_id = 0;
    fm._sensor_id = "cam01";
    fm._width = 1920;
    fm._height = 1080;
    fm._object_meta_list.push_back(&obj);

    GstDxMsgMetaInfo info{};
    info._frame_meta = &fm;
    info._seq_id = 123;
    info._pts_ns = 1234567890;
    info._ntp_timestamp_ns = -1;
    info._node_id = "edge-001";

    DxMsgContext ctx{};
    ctx._priv_data = nullptr;
    ctx._payload_type = DX_PAYLOAD_TYPE_PROTOBUF;

    size_t size = 0;
    void *buf = dxpayload_convert_to_protobuf(&ctx, &info, &size);
    assert(buf != nullptr);
    assert(size > 0);

    dxargus::SensorFrame frame;
    bool ok = frame.ParseFromArray(buf, (int)size);
    assert(ok);
    assert(frame.sensor_id() == "cam01");
    assert(frame.node_id() == "edge-001");
    assert(frame.stream_id() == 0);
    assert(frame.seq_id() == 123u);
    assert(frame.width() == 1920);
    assert(frame.height() == 1080);
    assert(frame.pts_ns() == 1234567890);
    assert(frame.ntp_timestamp_ns() == -1);
    assert(frame.objects_size() == 1);
    const dxargus::Detection &d = frame.objects(0);
    assert(d.label_id() == 1);
    assert(d.track_id() == 42);
    assert(d.name() == "person");
    assert(d.box().start_x() == 300.f);
    assert(d.box().start_y() == 400.f);
    assert(d.box().end_x() == 500.f);
    assert(d.box().end_y() == 600.f);

    g_free(buf);
    std::printf("PASS\n");
    return 0;
}
