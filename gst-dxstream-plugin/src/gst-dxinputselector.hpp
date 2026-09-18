#ifndef GST_DXINPUTSELECTOR_H
#define GST_DXINPUTSELECTOR_H

#include <gst/base/gstaggregator.h>
#include <gst/gst.h>
#include <set>

G_BEGIN_DECLS

#define GST_TYPE_DXINPUTSELECTOR (gst_dxinputselector_get_type())
G_DECLARE_FINAL_TYPE(GstDxInputSelector, gst_dxinputselector, GST,
                     DXINPUTSELECTOR, GstAggregator)

struct _GstDxInputSelector {
    GstAggregator parent_instance;
    std::set<int> _stream_eos_sent;
    guint _max_queue_size;
    /* Pads left out of an aggregation round because the live deadline passed.
     * Read-only "pads-skipped" property: the deadline is a tolerance for
     * inter-stream arrival skew, and without a count there is no way to tell a
     * value that is too small (skipping constantly) from one that never fires. */
    guint64 _pads_skipped;
};

G_END_DECLS

#endif // GST_DXINPUTSELECTOR_H
