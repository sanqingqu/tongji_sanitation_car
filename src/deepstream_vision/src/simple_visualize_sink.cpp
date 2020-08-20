#include "simple_visualize_sink.hpp"

SimpleVisualizeSink::SimpleVisualizeSink(Pipeline& pipeline, Node& upstream){
  // Configure elements //
  assert(this->videoconvert0 = pipeline.make("videoconvert"));
  assert(this->xvimagesink0 = pipeline.make("xvimagesink"));
  g_object_set(G_OBJECT(this->xvimagesink0),"sync",false,NULL);
  // Add elements //
  gst_bin_add_many(GST_BIN(pipeline.as_element()), this->videoconvert0, this->xvimagesink0, NULL);
  // Link elements //
  assert(gst_element_link_many(upstream.as_source_pad(), this->videoconvert0, this->xvimagesink0, NULL));
}

FakeSink::FakeSink(Pipeline& pipeline, Node& upstream){
  // Configure elements //
  assert(this->fakesink = pipeline.make("fakesink"));
  g_object_set(G_OBJECT(this->fakesink),"sync",false,NULL);
  // Add elements //
  gst_bin_add_many(GST_BIN(pipeline.as_element()), this->fakesink, NULL);
  // Link elements //
  assert(gst_element_link_many(upstream.as_source_pad(), this->fakesink, NULL));
}

// static GstPadProbeReturn
// osd_sink_pad_buffer_probe (GstPad * pad, GstPadProbeInfo * info,
//     gpointer u_data)
// {
//     GstBuffer *buf = (GstBuffer *) info->data;
//     NvDsObjectMeta *obj_meta = NULL;
//     NvDsMetaList * l_frame = NULL;
//     NvDsMetaList * l_obj = NULL;

//     NvDsBatchMeta *batch_meta = gst_buffer_get_nvds_batch_meta (buf);

//     for (l_frame = batch_meta->frame_meta_list; l_frame != NULL; l_frame = l_frame->next) {
//         NvDsFrameMeta *frame_meta = (NvDsFrameMeta *) (l_frame->data);
//         for (l_obj = frame_meta->obj_meta_list; l_obj != NULL; l_obj = l_obj->next) {
//             obj_meta = (NvDsObjectMeta *) (l_obj->data);
//             //
//         }
//     }
//     return GST_PAD_PROBE_OK;
// }

EGLVisualizeSink::EGLVisualizeSink(Pipeline& pipeline, Node& upstream){
  // Configure elements //
  assert(this->nvvideoconvert = pipeline.make("nvvideoconvert"));
  assert(this->nvdsosd = pipeline.make("nvdsosd"));
  assert(this->nvegltransform = pipeline.make("nvegltransform"));
  assert(this->nveglglessink = pipeline.make("nveglglessink"));
  g_object_set(G_OBJECT(this->nveglglessink),"sync",false,NULL);
  // Add elements //
  gst_bin_add_many(GST_BIN(pipeline.as_element()), this->nvvideoconvert, this->nvdsosd, this->nvegltransform, this->nveglglessink, NULL);
  // Link elements //
  assert(gst_element_link_many(upstream.as_source_pad(), this->nvvideoconvert, this->nvdsosd, this->nvegltransform, this->nveglglessink, NULL));
  // callback registration //
  // GstPad * osd_sink_pad = gst_element_get_static_pad (this->nvdsosd, "sink");
  // gst_pad_add_probe (osd_sink_pad, GST_PAD_PROBE_TYPE_BUFFER,
  // osd_sink_pad_buffer_probe, NULL, NULL);
}

