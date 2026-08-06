#ifndef __DX_MSGCONVL_PRIV_H__
#define __DX_MSGCONVL_PRIV_H__

#include "gstdxstream/gst-dxmsgmeta.hpp"
#include <vector>
#include <string>

// private property from config file
struct _DxMsgContextPriv {
    guint _customId;
    std::vector<std::string> _object_include_list;

};
using DxMsgContextPriv = _DxMsgContextPriv;

DxMsgContextPriv *dxcontext_create_contextPriv(void);
void dxcontext_delete_contextPriv(DxMsgContextPriv *contextPriv);

bool dxcontext_parse_json_config(const gchar *file,
                                 DxMsgContextPriv *contextPriv);

gchar *dxpayload_convert_to_json(DxMsgContext *context,
                                 GstDxMsgMetaInfo *meta_info);

// Serialize the frame meta as a dxargus.SensorFrame protobuf message.
// Returns a g_malloc'd buffer of *out_size bytes (caller frees with g_free),
// or nullptr on failure. Bytes are NOT null-terminated.
void *dxpayload_convert_to_protobuf(DxMsgContext *context,
                                    GstDxMsgMetaInfo *meta_info,
                                    size_t *out_size);

#endif /* __DX_MSGCONVL_PRIV_H__ */