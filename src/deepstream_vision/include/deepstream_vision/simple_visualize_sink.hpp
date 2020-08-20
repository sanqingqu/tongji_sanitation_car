#include "common.hpp"

class SimpleVisualizeSink : public Node {
  public:
    SimpleVisualizeSink(Pipeline& pipeline, Node& upstream);
    virtual GstElement * as_source_pad(void) {return NULL;}
  private:
    GstElement * videoconvert0, * xvimagesink0;
};

class EGLVisualizeSink : public Node {
  public:
    EGLVisualizeSink(Pipeline& pipeline, Node& upstream);
    virtual GstElement * as_source_pad(void) {return NULL;}
  private:
    GstElement * nvvideoconvert;
    GstElement * nvdsosd;
    GstElement * nvegltransform;
    GstElement * nveglglessink;
};

class FakeSink : public Node {
  public:
    FakeSink(Pipeline& pipeline, Node& upstream);
    virtual GstElement * as_source_pad(void) {return NULL;}
  private:
    GstElement * fakesink;
};

/*
assert(this->nvegltransform = pipeline.make("nvegltransform"));
assert(this->nveglglessink = pipeline.make("nveglglessink"));
assert(this->fakesink = pipeline.make("fakesink"));
*/