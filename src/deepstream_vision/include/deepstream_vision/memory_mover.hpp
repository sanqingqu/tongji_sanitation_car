#include "common.hpp"

class MoveToGPU : public Node {
  public:
    MoveToGPU(Pipeline& pipeline, Node& upstream);
    virtual GstElement * as_source_pad(void) { return capsfilter0; }
  private:
    GstElement * videoconvert0;
    GstElement * nvvideoconvert0;
    GstElement * capsfilter0;
};

class MoveToCPU : public Node {
  public:
    MoveToCPU(Pipeline& pipeline, Node& upstream);
    virtual GstElement * as_source_pad(void) { return videoconvert0; }
  private:
    GstElement * nvvideoconvert0;
    GstElement * capsfilter0;
    GstElement * videoconvert0;
};
