#ifndef __GST_DXMSGMETA_H__
#define __GST_DXMSGMETA_H__

#include "dxcommon.hpp"
#include <gst/gst.h>
#include <gst/video/video.h>

G_BEGIN_DECLS

#define GST_DXMSG_META_API_TYPE (gst_dxmsg_meta_api_get_type())
#define GST_DXMSG_META_INFO (gst_dxmsg_meta_get_info())

// Payload serialization format selector. Set on dxmsgconv via the
// "payload-type" property and read by the custom convert library to route
// serialization. JSON stays 0 so existing pipelines keep their behaviour.
typedef enum {
    DX_PAYLOAD_TYPE_JSON     = 0,
    DX_PAYLOAD_TYPE_PROTOBUF = 1,
} DxPayloadType;

#define DX_TYPE_PAYLOAD_TYPE (dx_payload_type_get_type())
GType dx_payload_type_get_type(void);

struct _DxMsgPayload {
    gpointer _data;
    guint _size;
    // Optional broker message key (e.g. sensor_id) for partitioned delivery.
    // nullptr = no key. Owned by the payload; copied through GstDxMsgMeta.
    gchar *_key;
};

struct _DxMsgContext {
    gpointer _priv_data;
    // Serialization format (DX_PAYLOAD_TYPE_*). Zero-initialized to JSON by
    // g_new0 in the custom lib; dxmsgconv overwrites it in start().
    gint _payload_type;
};

struct _GstDxMsgMetaInfo {
    gpointer _frame_meta;
    gpointer _input_info;
    gboolean _include_frame;

    guint64 _seq_id;

    // Frame capture timestamps (Phase 1B native timestamp support).
    // _pts_ns: GstBuffer PTS in nanoseconds (-1 if GST_CLOCK_TIME_NONE).
    // _ntp_timestamp_ns: wall-clock capture time in nanoseconds, sourced from
    //   GstReferenceTimestampMeta (e.g. rtspsrc RTCP NTP). -1 if unavailable.
    gint64 _pts_ns;
    gint64 _ntp_timestamp_ns;

    // Edge-global node identity (B2). Configured via dxmsgconv "node-id"
    // element property. nullptr when unset. Borrowed pointer (owned by the
    // element); valid only for the duration of the convert call.
    const gchar *_node_id;

    const gchar *_frame_base64;
};

struct _GstDxMsgMeta {
    GstMeta meta;

    gpointer _payload;
};

using DxMsgPayload = struct _DxMsgPayload;
using DxMsgContext = struct _DxMsgContext;
using GstDxMsgMetaInfo = struct _GstDxMsgMetaInfo;
using GstDxMsgMeta = struct _GstDxMsgMeta;

DX_API GType gst_dxmsg_meta_api_get_type(void);

DX_API const GstMetaInfo *gst_dxmsg_meta_get_info(void);

DX_API GstBuffer*dx_create_msg_meta(GstBuffer *buffer);
DX_API GstDxMsgMeta *dx_get_msg_meta(GstBuffer *buffer);
DX_API void dx_add_payload_to_buffer(GstBuffer *buffer, const DxMsgPayload *payload);

G_END_DECLS

#endif /* __GST_DXMSGMETA_H__ */