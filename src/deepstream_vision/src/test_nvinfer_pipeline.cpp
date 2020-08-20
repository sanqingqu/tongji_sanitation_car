#include "simple_pipeline.hpp"
#include "opencv_dewarp_source.hpp"
#include "simple_visualize_sink.hpp"
#include "memory_mover.hpp"
#include "nvinfer_node.hpp"

int main(int argc, char** argv) {
  /* Check input arguments */
  if (argc != 2) {
    g_printerr ("Usage: %s <model config filename>\n", argv[0]);
    return -1;
  }
  bool enable_dewarp = true;
  SimplePipeline pipeline0(&argc, &argv);
  OpencvDewarpSource cv_src(pipeline0, enable_dewarp);
  MoveToGPU mtg0(pipeline0, cv_src);
  NvInfer nvinfer(pipeline0, mtg0, argv[1]);
  EGLVisualizeSink  eglsink(pipeline0, nvinfer);
  // MoveToCPU mtc0(pipeline0, nvinfer);
  // SimpleVisualizeSink simple_sink(pipeline0, mtc0);
  pipeline0.spin();
  return 0;
}

