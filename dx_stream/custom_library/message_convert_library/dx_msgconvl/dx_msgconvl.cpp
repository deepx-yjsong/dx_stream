#include "dx_msgconvl_priv.hpp"
#include "gstdxstream/gst-dxframemeta.hpp"
#include <glib.h>
#include <stddef.h>
#include <string.h>

#define MAX_EXPECTED_JSON_SIZE ((size_t)(10 * 1024 * 1024))

DX_CUSTOM_EXPORT DxMsgContext *dxmsg_create_context() {
    auto *context = g_new0(DxMsgContext, 1);

    context->_priv_data = (void *)dxcontext_create_contextPriv();

    return context;
}

DX_CUSTOM_EXPORT void dxmsg_delete_context(DxMsgContext *context) {
    g_return_if_fail(context != nullptr);

    dxcontext_delete_contextPriv((DxMsgContextPriv *)context->_priv_data);
    g_free(context);
}

DX_CUSTOM_EXPORT DxMsgPayload *dxmsg_convert_payload(DxMsgContext *context,
                                               GstDxMsgMetaInfo *meta_info) {
    auto *payload = g_new0(DxMsgPayload, 1);
    if (!payload) {
        g_warning("Failed to allocate DxMsgPayload");
        return nullptr;
    }

    // Derive the broker partition key (native sensor_id, ADR-023/024) for both
    // serialization formats. nullptr when unset.
    const auto *fm = (DXFrameMeta *)meta_info->_frame_meta;
    if (fm && !fm->_sensor_id.empty()) {
        payload->_key = g_strdup(fm->_sensor_id.c_str());
    }

    if (context && context->_payload_type == DX_PAYLOAD_TYPE_PROTOBUF) {
        size_t pb_size = 0;
        void *pb_data = dxpayload_convert_to_protobuf(context, meta_info, &pb_size);
        if (!pb_data) {
            g_warning("dxpayload_convert_to_protobuf returned null");
            g_free(payload->_key);
            g_free(payload);
            return nullptr;
        }
        if (pb_size > G_MAXUINT) {
            g_warning("Protobuf data size (%zu bytes) exceeds maximum guint value", pb_size);
            g_free(pb_data);
            g_free(payload->_key);
            g_free(payload);
            return nullptr;
        }
        payload->_size = (guint)pb_size;
        payload->_data = pb_data;
        return payload;
    }

    gchar *json_data = dxpayload_convert_to_json(context, meta_info);
    if (json_data == nullptr) {
        g_warning("dxpayload_convert_to_json returned null");
        g_free(payload->_key);
        g_free(payload);
        return nullptr;
    }
    size_t json_len = strnlen(json_data, MAX_EXPECTED_JSON_SIZE);
    if (json_len == MAX_EXPECTED_JSON_SIZE &&
        json_data[MAX_EXPECTED_JSON_SIZE - 1] != '\0') {
        g_warning("JSON data is too long (>= %zu bytes) or not null-terminated "
                  "within the checked limit.",
                  MAX_EXPECTED_JSON_SIZE);
        g_free(json_data);
        g_free(payload->_key);
        g_free(payload);
        return nullptr;
    }

    if (json_len > G_MAXUINT) {
        g_warning("JSON data size (%zu bytes) exceeds maximum guint value", json_len);
        g_free(json_data);
        g_free(payload->_key);
        g_free(payload);
        return nullptr;
    }

    payload->_size = (guint)json_len;
    payload->_data = json_data;

    return payload;
}
