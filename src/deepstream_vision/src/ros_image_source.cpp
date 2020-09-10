#include "ros_image_source.hpp"
#include <vector>

namespace cv {

Mat equalizeIntensity(const Mat& inputImage)
{
    if(inputImage.channels() >= 3)
    {
        Mat ycrcb;

        cvtColor(inputImage,ycrcb,CV_BGR2YCrCb);

        std::vector<Mat> channels;
        split(ycrcb,channels);

        Mat temp;
        equalizeHist(channels[0], temp);
        channels[0] = channels[0] / 2 + temp / 2;

        Mat result;
        merge(channels,ycrcb);

        cvtColor(ycrcb,result,CV_YCrCb2BGR);

        return result;
    }
    return Mat();
}

}// end namespace cv


void RosJPEGImageSource::callback(const sensor_msgs::CompressedImageConstPtr& msg)
{
  // fprintf(stderr, "1"); fflush(0);
  try
  {
    std::lock_guard<std::mutex> lck (this->frame_mutex);
    this->frame = cv::imdecode(cv::Mat(msg->data),1);//convert compressed image data to cv::Mat
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("[ros_image_source] failed to parse image!");
  }
}

static void destroy_cv_mat(gpointer data) {
  cv::Mat * done = (cv::Mat*) data;
  delete done;
}

static gboolean feed_data_to_appsrc0(RosJPEGImageSource * cv_src) {
  // fprintf(stderr, "2");fflush(0);
  cv::Mat * pframe;
  // read a video frame
  pframe = new cv::Mat(); 
  {
    do {
        std::lock_guard<std::mutex> lck (cv_src->frame_mutex);
        if (!cv_src->frame.empty()) break;
    }while(ros::ok());

    if (!ros::ok()) return FALSE;
      
      // if (!cv_src->frame_mutex.try_lock()) {
      //   return FALSE;
      // }
      // if (cv_src->frame.empty()) {
      //   // cv_src->frame_mutex.unlock();
      //   return FALSE;
      // }
      *pframe = cv_src->frame;
      cv_src->frame = cv::Mat();
      // cv_src->frame_mutex.unlock();
  }
  //fprintf(stderr, "cv_cap read ok:%d\n", ok); fflush(0);
  // preprocessing (crop&pad)
//   if (cv_src->enable_dewarp) cv::remap(*pframe, *pframe, cv_src->dewarp_map1, cv_src->dewarp_map2, cv::INTER_LINEAR); 
    cv::cvtColor(*pframe, *pframe, cv::COLOR_BGR2GRAY);
    // cv::equalizeHist(*pframe, *pframe);
    cv::cvtColor(*pframe, *pframe, cv::COLOR_GRAY2BGR);
    cv::remap(*pframe, *pframe, cv_src->dewarp_map1, cv_src->dewarp_map2, cv::INTER_LINEAR); 
    //*pframe = cv::equalizeIntensity(*pframe);
    //fprintf(stderr, "[%d, %d] ", pframe->rows, pframe->cols); fflush(0);
    //cv::resize(*pframe, *pframe, cv::Size(UNWARP_WIDTH, UNWARP_HEIGHT)); // FIXME
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

static void appsrc0_on_need_data(GstElement *src, guint size, RosJPEGImageSource *cv_src) {
  // fprintf(stderr, "3");fflush(0);
  if (cv_src->appsrc0_sourceid == 0) {
    cv_src->appsrc0_sourceid = g_idle_add((GSourceFunc) feed_data_to_appsrc0, cv_src);
  }
}

static void appsrc0_on_enough_data(GstElement *sink, RosJPEGImageSource *cv_src) {
  // fprintf(stderr, "4");fflush(0);  
  if (cv_src->appsrc0_sourceid != 0) {
    g_source_remove (cv_src->appsrc0_sourceid);
    cv_src->appsrc0_sourceid = 0;
  }
}

#if 0
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
  cv::Mat P = (cv::Mat_<float>(3,3) << 200.0f, 0.0f, UNWARP_WIDTH/2.0f,
                    0.0f, 200.0f, UNWARP_HEIGHT/2.0f, 0.0f, 0.0f, 1.0f);
  cv::Size net_input_sz(UNWARP_WIDTH, UNWARP_HEIGHT);
  cv::fisheye::initUndistortRectifyMap ( K, D, R, P, net_input_sz, CV_16SC2, map1, map2 );
}
#endif

static void init_undistortion_map_nodist(cv::Mat& map1, cv::Mat& map2) {
#ifdef LOWER
  cv::Mat K = (cv::Mat_<float>(3,3) << 367.0543074903704 * 0.5, 0.0, 990.6330750325744 * 0.5, 0.0, 366.7370079611347 * 0.5, 575.1183044201284 * 0.5, 0.0, 0.0, 1.0);

#elif defined UPPER
  cv::Mat K = (cv::Mat_<float>(3,3) << 367.2867502156721*0.5, 0.0, 960.8303932411943*0.5, 0.0, 369.06857590098275*0.5, 562.6211777029157*0.5, 0.0, 0.0, 1.0);
#else
  assert(0);
#endif
  std::vector<float> D;
  cv::Mat R = (cv::Mat_<float>(3,3) << 1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f);
  cv::Mat P = (cv::Mat_<float>(3,3) << 200.0f, 0.0f, UNWARP_WIDTH/2.0f,
                    0.0f, 200.0f, UNWARP_HEIGHT/2.0f, 0.0f, 0.0f, 1.0f);
  cv::Size net_input_sz(UNWARP_WIDTH, UNWARP_HEIGHT);
  cv::initUndistortRectifyMap ( K, D, R, P, net_input_sz, CV_16SC2, map1, map2 );
}


RosJPEGImageSource::RosJPEGImageSource(Pipeline& pipeline, bool _enable_dewarp) : enable_dewarp(_enable_dewarp), appsrc0_sourceid(0) {
  // Configure ROS Node //
  ros::NodeHandle nh;
  #ifdef LOWER
  this->subscriber = nh.subscribe("/undistort_lower/compressed", 1, &RosJPEGImageSource::callback, this);
  #elif defined UPPER
  this->subscriber = nh.subscribe("/undistort_upper/compressed", 1, &RosJPEGImageSource::callback, this);
  #else
  assert(0);
  #endif  
  assert (!enable_dewarp); // FIXME: for sanitation project only
  //if (enable_dewarp) init_undistortion_map(this->dewarp_map1, this->dewarp_map2);
  init_undistortion_map_nodist(this->dewarp_map1, this->dewarp_map2);
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

int RosJPEGImageSource::frame_width(void) {
//   return (enable_dewarp ? UNWARP_WIDTH : IMAGE_WIDTH);
  // FIXME: for sanitation project only
  return UNWARP_WIDTH;
}

int RosJPEGImageSource::frame_height(void) {
//   return (enable_dewarp ? UNWARP_HEIGHT : IMAGE_HEIGHT);
  // FIXME: for sanitation project only
  return UNWARP_HEIGHT;
}

RosJPEGImageSource::~RosJPEGImageSource(void) {
//   this->cv_cap.release();
}

