#include "common.hpp"
#include <opencv2/opencv.hpp>
#include <thread>
#include <mutex>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

class RosJPEGImageSource : public Node {
  public:
    RosJPEGImageSource(Pipeline& pipeline, bool dewarp);
    ~RosJPEGImageSource(void);
    virtual GstElement * as_source_pad(void) { return appsrc0; }
    int frame_width(void);
    int frame_height(void);
    void callback(const sensor_msgs::CompressedImageConstPtr& msg);
  public:
    cv::Mat dewarp_map1, dewarp_map2;
    cv::Mat frame;
    std::mutex frame_mutex;
    ros::Subscriber subscriber;
    // cv::VideoCapture cv_cap;
    bool enable_dewarp;
    GstElement * appsrc0;
    guint appsrc0_sourceid;
    GstFlowReturn gfreturn;
};