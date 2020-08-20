#include "memory_mover.hpp"

MoveToGPU::MoveToGPU(Pipeline& pipeline, Node& upstream) {
  // Configure elements //
  assert(this->videoconvert0 = pipeline.make("videoconvert"));
	assert(this->nvvideoconvert0 = pipeline.make("nvvideoconvert"));
	assert(this->capsfilter0 = pipeline.make("capsfilter"));
	g_object_set(G_OBJECT(this->capsfilter0), "caps",
               gst_caps_from_string("video/x-raw(memory:NVMM)"), NULL);
  // Add elements //
  gst_bin_add_many(GST_BIN(pipeline.as_element()), this->videoconvert0, this->nvvideoconvert0, this->capsfilter0, NULL);
  // Link elements //
  assert(gst_element_link_many(upstream.as_source_pad(), this->videoconvert0, this->nvvideoconvert0, this->capsfilter0, NULL));
}

MoveToCPU::MoveToCPU(Pipeline& pipeline, Node& upstream){
  // Configure elements //
	assert(this->nvvideoconvert0 = pipeline.make("nvvideoconvert"));
	assert(this->capsfilter0 = pipeline.make("capsfilter"));
  assert(this->videoconvert0 = pipeline.make("videoconvert"));
	g_object_set(G_OBJECT(this->capsfilter0), "caps",
               gst_caps_from_string("video/x-raw"), NULL);
  // Add elements //
  gst_bin_add_many(GST_BIN(pipeline.as_element()), this->nvvideoconvert0, this->capsfilter0, this->videoconvert0, NULL);
  // Link elements //
  assert(gst_element_link_many(upstream.as_source_pad(), this->nvvideoconvert0, this->capsfilter0, this->videoconvert0, NULL));
  // assert(gst_element_link_many(upstream, this->nvvideoconvert0, NULL));
  // assert(gst_element_link_many(this->nvvideoconvert0, this->capsfilter0, NULL));
  // assert(gst_element_link_many(this->capsfilter0, this->videoconvert0, NULL));
}
