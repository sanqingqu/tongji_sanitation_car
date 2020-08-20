#include "nvinfer_node.hpp"
#include "gstnvdsmeta.h"

static void nvstreammux_single_connect(GstElement * nvstreammux, GstElement * upstream) {
  GstPad *sinkpad, *srcpad;
  gchar pad_name_sink[16] = "sink_0";
  gchar pad_name_src[16] = "src";

  assert(sinkpad = gst_element_get_request_pad (nvstreammux, pad_name_sink));
  assert(srcpad = gst_element_get_static_pad (upstream, pad_name_src));

  assert(gst_pad_link (srcpad, sinkpad) == GST_PAD_LINK_OK);

  gst_object_unref (sinkpad);
  gst_object_unref (srcpad);
}

NvInfer::NvInfer(Pipeline& pipeline, Node& upstream, const char * config_file_path) {
  assert(this->nvstreammux = pipeline.make("nvstreammux"));
  assert(this->nvinfer = pipeline.make("nvinfer"));

  g_object_set (G_OBJECT (this->nvstreammux), "batch-size", 1, NULL);
  g_object_set (G_OBJECT (this->nvstreammux), "width", UNWARP_WIDTH, "height", UNWARP_HEIGHT,
      "batched-push-timeout", MUXER_BATCH_TIMEOUT_USEC, NULL);
  g_object_set (G_OBJECT (this->nvinfer),
      "config-file-path", config_file_path, NULL);

  // Add elements //
  gst_bin_add_many(GST_BIN(pipeline.as_element()), this->nvstreammux, this->nvinfer, NULL);
  // Link elements //
  nvstreammux_single_connect(this->nvstreammux, upstream.as_source_pad());
  assert(gst_element_link_many(this->nvstreammux, this->nvinfer, NULL));
}
