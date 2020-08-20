#include "simple_pipeline.hpp"

SimplePipeline::SimplePipeline(int* argc, char*** argv) : Pipeline() {
  gst_init(argc, argv);
  this->pipeline0 = gst_pipeline_new("simple_pipeline0");
}

SimplePipeline::~SimplePipeline() {
}

void SimplePipeline::spin() {
  // Construct bus and start pipeline0 //
  this->bus0 = gst_element_get_bus(this->pipeline0);
  gst_bus_add_signal_watch(this->bus0);
  gst_object_unref(this->bus0);
  gst_element_set_state(this->pipeline0, GST_STATE_PLAYING);
  this->main_loop = g_main_loop_new(NULL, FALSE);
  g_main_loop_run(this->main_loop);
  // Free resources //
  gst_element_set_state(this->pipeline0, GST_STATE_NULL);
  gst_object_unref(this->pipeline0);
}

GstElement * SimplePipeline::as_element() {
  return this->pipeline0;
}

