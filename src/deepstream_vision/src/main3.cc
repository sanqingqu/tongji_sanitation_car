#include <cassert>
#include <gst/gst.h>
#include <opencv2/opencv.hpp>
#include <gstnvdsmeta.h>

/* Muxer batch formation timeout, for e.g. 40 millisec. Should ideally be set
 * based on the fastest source's framerate. */
#define MUXER_BATCH_TIMEOUT_USEC 33333

typedef struct _AppContext {
  GstElement * pipeline0, * appsrc0, * videoconvert0, * xvimagesink0;
  GstElement * nvvideoconvert0, * capsfilter0, * nvstreammux0, * nvinfer0;
  GstElement * nvvideoconvert1, * nvdsosd0, * nvegltransform0, *nveglelessink0;
  GMainLoop * main_loop;
  GstBus * bus;
  cv::VideoCapture cv_cap;
  cv::Mat dewarp_map1, dewarp_map2;
  bool enable_dewarp;
  bool enable_detect;
  bool enable_display;
  const char * nvinfer_config_file_path;
} AppContext;




static void init_deepstream_preprocess_bin(AppContext* app){
  assert(app->videoconvert0 = gst_element_factory_make("videoconvert", "videoconvert0"));
	assert(app->nvvideoconvert0 = gst_element_factory_make("nvvideoconvert","nvvideoconvert0"));
	assert(app->capsfilter0 = gst_element_factory_make("capsfilter", "capsfilter0"));
	g_object_set(G_OBJECT(app->capsfilter0), "caps", gst_caps_from_string("video/x-raw(memory:NVMM)"), NULL);
  assert(app->nvstreammux0 = gst_element_factory_make ("nvstreammux", "nvstreammux0"));
  g_object_set (G_OBJECT (app->nvstreammux0), "width", UNWARP_WIDTH, "height",
                UNWARP_HEIGHT, "batch-size", 1,
                "batched-push-timeout", MUXER_BATCH_TIMEOUT_USEC, NULL);
  // link capsfilter0_src(nvmm) to requested sink of nvstreammux0, which forms batch.
  GstPad * capsfilter0_src, * nvstreammux0_sink_0;
  assert(capsfilter0_src = gst_element_get_static_pad(capsfilter, "src"));
  assert(nvstreammux0_sink_0 = gst_element_get_request_pad (nvstreammux, "sink_0"));
  assert(gst_pad_link (capsfilter0_src, nvstreammux0_sink_0) == GST_PAD_LINK_OK);
  gst_object_unref(capsfilter0_src); gst_object_unref(nvstreammux0_sink_0);
}



static void app_basic_sink(AppContext * app) {
  // Configure opencv videocapture//
  system(CAMERA_CONFIGURE_CMD);
  app->cv_cap.open(VIDEO_SOURCE, cv::CAP_GSTREAMER);
  assert(app->cv_cap.isOpened());
  if (app->enable_dewarp) init_undistortion_map(app->dewarp_map1, app->dewarp_map2);

  // Configure appsrc //
  assert(app->appsrc0 = gst_element_factory_make("appsrc", "appsrc0"));
	g_object_set(G_OBJECT(app->appsrc0), "caps",
		gst_caps_new_simple("video/x-raw",
			"format", G_TYPE_STRING, "BGR",
			"width", G_TYPE_INT, (app->enable_dewarp ? UNWARP_WIDTH : IMAGE_WIDTH),
			"height", G_TYPE_INT, (app->enable_dewarp ? UNWARP_HEIGHT : IMAGE_HEIGHT),
			"framerate", GST_TYPE_FRACTION, IMAGE_FPS, 1,
			NULL), NULL);
  g_signal_connect(app->appsrc0, "need-data", G_CALLBACK (appsrc0_on_need_data), app);
  g_signal_connect(app->appsrc0, "enough-data", G_CALLBACK (appsrc0_on_enough_data), app);


  // No Detect, only display //
  if (app->enable_display && !app->enable_detect) {

    // Link all elements //
    app->pipeline0 = gst_pipeline_new("pipeline0");
    gst_bin_add_many(GST_BIN(app->pipeline0), app->appsrc0, app->videoconvert0, app->xvimagesink0, NULL);
    assert(gst_element_link_many(app->appsrc0, app->videoconvert0, app->xvimagesink0, NULL));
  }
  // Detect using neural networks //
  else if (app->enable_detect) {

    // Configure nvinver element //
    assert(app->nvinfer0 = gst_element_factory_make ("nvinfer", "nvinfer0"));
    g_object_set (G_OBJECT (app->nvinfer0),
                  "config-file-path", app->nvinfer_config_file_path, NULL);

    // Configure downstreams //
		assert(app->nvvideoconvert1 = gst_element_factory_make("nvvideoconvert","nvvideoconvert1"));
    assert(app->nvdsosd0 = gst_element_factory_make ("nvdsosd", "nvdsosd0"));
    assert(app->nvegltransform0 = gst_element_factory_make ("nvegltransform", "nvegltransform0"));
    assert(app->nveglelessink0 = gst_element_factory_make ("nveglglessink", "nveglelessink0"));

    // Link all elements //
    app->pipeline0 = gst_pipeline_new("pipeline0");
    gst_bin_add_many(GST_BIN(app->pipeline0),
                     app->appsrc0, app->videoconvert0, app->nvvideoconvert0, app->capsfilter0,
                     app->nvstreammux0, app->nvvideoconvert1, app->nvdsosd0,
                     app->nvegltransform0, app->nveglelessink0, NULL);
    assert(gst_element_link_many(app->appsrc0, app->videoconvert0, app->nvvideoconvert0, app->capsfilter0, NULL));
    assert(gst_element_link_many(app->nvstreammux0, app->nvinfer0, app->nvvideoconvert1, app->nvdsosd0, app->nvegltransform0, app->nveglelessink0, NULL));
    if(app->enable_display) {;} else {assert(0);}
  } else{
    assert(0);
  }

  // Construct bus and start pipeline0 //
  app->bus = gst_element_get_bus(app->pipeline0);
  gst_bus_add_signal_watch(app->bus);
  gst_object_unref(app->bus);
  gst_element_set_state(app->pipeline0, GST_STATE_PLAYING);
  app->main_loop = g_main_loop_new(NULL, FALSE);
  g_main_loop_run(app->main_loop);

  // Free resources //
  gst_element_set_state(app->pipeline0, GST_STATE_NULL);
  gst_object_unref(app->pipeline0);
}

int main(int argc, char** argv) {
  static AppContext app;
  gst_init(&argc, &argv);
  app.enable_dewarp = true;
  app.enable_detect = true;
  app.enable_display = true;
#ifdef UPPER
  app.nvinfer_config_file_path = "./config_nvinfer_upper.yaml";
#elif defined LOWER
  app.nvinfer_config_file_path = "./config_nvinfer_lower.yaml";
#else
  assert(0);
#endif

  //test_read_video_frame(&app);
  app_basic_sink(&app);
  return 0;
}

