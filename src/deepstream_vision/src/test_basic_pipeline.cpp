#include "simple_pipeline.hpp"
#include "opencv_dewarp_source.hpp"
#include "simple_visualize_sink.hpp"
#include "memory_mover.hpp"

int main(int argc, char** argv) {
  bool enable_dewarp = true;
  SimplePipeline pipeline0(&argc, &argv);
  OpencvDewarpSource cv_src(pipeline0, enable_dewarp);
  MoveToGPU mtg0(pipeline0, cv_src);
  MoveToCPU mtc0(pipeline0, mtg0);
  SimpleVisualizeSink simple_sink(pipeline0, mtc0);
  pipeline0.spin();
  return 0;
}

