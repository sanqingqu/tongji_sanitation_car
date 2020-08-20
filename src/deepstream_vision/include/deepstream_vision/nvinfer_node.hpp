#include "common.hpp"

class NvInfer : public Node {
    public:
        NvInfer(Pipeline& pipeline, Node& upstream, const char * config_file_path);
        virtual GstElement * as_source_pad(void) { return nvinfer; }
    private:
        GstElement * nvstreammux;
        GstElement * nvinfer;
};