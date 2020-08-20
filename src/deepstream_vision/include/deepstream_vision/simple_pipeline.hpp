#include "common.hpp"

class SimplePipeline : public Pipeline {
  public:
    SimplePipeline(int* argc, char*** argv);
    ~SimplePipeline();
    virtual GstElement * as_element();
    virtual void spin();
  private:
    GstElement * pipeline0;
    GstBus * bus0;
    GMainLoop * main_loop;
};

