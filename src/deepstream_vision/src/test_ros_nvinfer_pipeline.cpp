#include "simple_pipeline.hpp"
#include "ros_image_source.hpp"
#include "simple_visualize_sink.hpp"
#include "memory_mover.hpp"
#include "nvinfer_node.hpp"

#include <cstdlib>
#include <csignal>

void trap_ctrl_c(int signum) {
  exit(signum);
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "yolo_object_detector");
  /* Check input arguments */
  if (argc != 2) {
    g_printerr ("Usage: %s <model config filename>\n", argv[0]);
    return -1;
  }
  bool enable_dewarp = false;
  SimplePipeline pipeline0(&argc, &argv);
  RosJPEGImageSource cv_src(pipeline0, enable_dewarp);
  MoveToGPU mtg0(pipeline0, cv_src);
  NvInfer nvinfer(pipeline0, mtg0, argv[1]);
  EGLVisualizeSink  eglsink(pipeline0, nvinfer);
  // MoveToCPU mtc0(pipeline0, nvinfer);
  // SimpleVisualizeSink simple_sink(pipeline0, cv_src);
  ros::AsyncSpinner spinner(1); // Use 1 threads
  signal(SIGINT, trap_ctrl_c);
  spinner.start();
  pipeline0.spin();
  return 0;
}
