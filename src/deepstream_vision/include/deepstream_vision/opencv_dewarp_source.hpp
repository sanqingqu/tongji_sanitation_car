#include "common.hpp"
#include <opencv2/opencv.hpp>

class OpencvDewarpSource : public Node {
  public:
    OpencvDewarpSource(Pipeline& pipeline, bool dewarp);
    ~OpencvDewarpSource(void);
    virtual GstElement * as_source_pad(void) { return appsrc0; }
    int frame_width(void);
    int frame_height(void);
  public:
    cv::Mat dewarp_map1, dewarp_map2;
    cv::VideoCapture cv_cap;
    bool enable_dewarp;
    GstElement * appsrc0;
    guint appsrc0_sourceid;
    GstFlowReturn gfreturn;
};

