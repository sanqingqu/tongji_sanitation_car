#include "opencv_dewarp_source.hpp"
#include <cstdio>

#define CAMERA_CONFIGURE_CMD "v4l2-ctl -d /dev/video0 -c exposure_auto=3"
#define VIDEO_SOURCE "v4l2src device=/dev/video0 ! " \
                     "image/jpeg, width=800, height=600, format=MJPG, framerate=30/1 ! " \
                     "jpegdec ! videoconvert ! appsink"

static void destroy_cv_mat(gpointer data) {
  cv::Mat * done = (cv::Mat*) data;
  delete done;
}

static gboolean feed_data_to_appsrc0(OpencvDewarpSource * cv_src) {
  bool ok;
  cv::Mat * pframe;
  // read a video frame
  pframe = new cv::Mat(); 
  ok = cv_src->cv_cap.read(*pframe);
  //fprintf(stderr, "cv_cap read ok:%d\n", ok); fflush(0);
  if (!ok) { delete pframe; return FALSE; }
  // preprocessing (dewarp)
  if (cv_src->enable_dewarp) cv::remap(*pframe, *pframe, cv_src->dewarp_map1, cv_src->dewarp_map2, cv::INTER_LINEAR);
  // convert into buffer
	gsize frame_bufsize = pframe->total() * pframe->elemSize();
	GstBuffer * gbuffer = gst_buffer_new_wrapped_full((GstMemoryFlags)0, (gpointer)(pframe->data), frame_bufsize, 0, frame_bufsize, (gpointer)(pframe), (GDestroyNotify)destroy_cv_mat);
  // push buffer
	g_signal_emit_by_name(cv_src->appsrc0, "push-buffer", gbuffer, &(cv_src->gfreturn));
  // clean resources
	gst_buffer_unref(gbuffer);
  if (cv_src->gfreturn != GST_FLOW_OK) { return FALSE; }
  return TRUE;
}

static void appsrc0_on_need_data(GstElement *src, guint size, OpencvDewarpSource *cv_src) {
  if (cv_src->appsrc0_sourceid == 0) {
    cv_src->appsrc0_sourceid = g_idle_add((GSourceFunc) feed_data_to_appsrc0, cv_src);
  }
}

static void appsrc0_on_enough_data(GstElement *sink, OpencvDewarpSource *cv_src) {
  if (cv_src->appsrc0_sourceid != 0) {
    g_source_remove (cv_src->appsrc0_sourceid);
    cv_src->appsrc0_sourceid = 0;
  }
}

static void init_undistortion_map(cv::Mat& map1, cv::Mat& map2) {
#ifdef LOWER
  cv::Mat K = (cv::Mat_<float>(3,3) << 146.4624354954575f, 0.0f, 409.6204399145412f,
                    0.0f, 146.57899752482163f, 312.0369280551138f, 0.0f, 0.0f, 1.0f);
  cv::Mat D = (cv::Mat_<float>(4,1) << 0.0675689868977554f, -0.010030342828346497f,
                    0.002192441212796896f, -0.001370717875218288f);
#elif defined UPPER
  cv::Mat K = (cv::Mat_<float>(3,3) << 150.9090714699482f, 0.0f, 396.08488043881437f,
                    0.0f, 151.32315692903708f, 302.34946200626666f, 0.0f, 0.0f, 1.0f);
  cv::Mat D = (cv::Mat_<float>(4,1) << 0.06225584907963871f, 0.002170237210013563f,
                    -0.0004619119344029414f, -0.0034679050957732425f);
#else
  assert(0);
#endif
  cv::Mat R = (cv::Mat_<float>(3,3) << 1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f);
  cv::Mat P = (cv::Mat_<float>(3,3) << 150.0f, 0.0f, UNWARP_WIDTH/2.0f,
                    0.0f, 150.0f, UNWARP_HEIGHT/2.0f, 0.0f, 0.0f, 1.0f);
  cv::Size net_input_sz(UNWARP_WIDTH, UNWARP_HEIGHT);
  cv::fisheye::initUndistortRectifyMap ( K, D, R, P, net_input_sz, CV_16SC2, map1, map2 );
}

OpencvDewarpSource::OpencvDewarpSource(Pipeline& pipeline, bool _enable_dewarp) : enable_dewarp(_enable_dewarp), appsrc0_sourceid(0) {
  // Configure OpenCV //
  system(CAMERA_CONFIGURE_CMD);
  this->cv_cap.open(VIDEO_SOURCE, cv::CAP_GSTREAMER);
  assert(this->cv_cap.isOpened());
  if (enable_dewarp) init_undistortion_map(this->dewarp_map1, this->dewarp_map2);
  // Configure Gstream Elements //
  assert(this->appsrc0 = gst_element_factory_make("appsrc", "cv_appsrc0"));
	g_object_set(G_OBJECT(this->appsrc0), "caps",
		gst_caps_new_simple("video/x-raw",
			"format", G_TYPE_STRING, "BGR",
			"width", G_TYPE_INT, this->frame_width(),
			"height", G_TYPE_INT, this->frame_height(),
			"framerate", GST_TYPE_FRACTION, IMAGE_FPS, 1,
			NULL), NULL);
  g_signal_connect(this->appsrc0, "need-data", G_CALLBACK (appsrc0_on_need_data), this);
  g_signal_connect(this->appsrc0, "enough-data", G_CALLBACK (appsrc0_on_enough_data), this);
  // Add elements //
  gst_bin_add_many(GST_BIN(pipeline.as_element()), this->appsrc0, NULL);
}

int OpencvDewarpSource::frame_width(void) {
  return (enable_dewarp ? UNWARP_WIDTH : IMAGE_WIDTH);
}

int OpencvDewarpSource::frame_height(void) {
  return (enable_dewarp ? UNWARP_HEIGHT : IMAGE_HEIGHT);
}

OpencvDewarpSource::~OpencvDewarpSource(void) {
  this->cv_cap.release();
}
