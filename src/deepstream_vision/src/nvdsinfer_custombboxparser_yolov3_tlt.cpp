/**
 * Copyright (c) 2020, NVIDIA CORPORATION.  All rights reserved.
 *
 * NVIDIA Corporation and its licensors retain all intellectual property
 * and proprietary rights in and to this software, related documentation
 * and any modifications thereto.  Any use, reproduction, disclosure or
 * distribution of this software and related documentation without an express
 * license agreement from NVIDIA Corporation is strictly prohibited.
 *
 */

#include <cstring>
#include <iostream>
#include "nvdsinfer_custom_impl.h"
#include <cassert>
#include <deepstream_vision/BBoxArray.h>

#ifdef _ROS_PIPELINE_
#include <ros/ros.h>
#endif

#define MIN(a,b) ((a) < (b) ? (a) : (b))
#define MAX(a,b) ((a) > (b) ? (a) : (b))
#define CLIP(a,min,max) (MAX(MIN(a, max), min))
#define DIVIDE_AND_ROUND_UP(a, b) ((a + b - 1) / b)

using namespace ::deepstream_vision;

/* This is a sample bounding box parsing function for the sample yolov3
 *
 * detector model provided with the SDK. */

/* C-linkage to prevent name-mangling */
extern "C"
bool NvDsInferParseCustomYOLOV3TLT (
         std::vector<NvDsInferLayerInfo> const &outputLayersInfo,
         NvDsInferNetworkInfo  const &networkInfo,
         NvDsInferParseDetectionParams const &detectionParams,
         std::vector<NvDsInferObjectDetectionInfo> &objectList);

extern "C"
bool NvDsInferParseCustomYOLOV3TLT (std::vector<NvDsInferLayerInfo> const &outputLayersInfo,
                                   NvDsInferNetworkInfo  const &networkInfo,
                                   NvDsInferParseDetectionParams const &detectionParams,
                                   std::vector<NvDsInferObjectDetectionInfo> &objectList) {
    if(outputLayersInfo.size() != 4)
    {
        std::cerr << "Mismatch in the number of output buffers."
                  << "Expected 4 output buffers, detected in the network :"
                  << outputLayersInfo.size() << std::endl;
        return false;
    }

    /* Host memory for "BatchedNMS"
       BatchedNMS has 4 output bindings, the order is:
       keepCount, bboxes, scores, classes
    */
    int* p_keep_count = (int *) outputLayersInfo[0].buffer;
    float* p_bboxes = (float *) outputLayersInfo[1].buffer;
    float* p_scores = (float *) outputLayersInfo[2].buffer;
    float* p_classes = (float *) outputLayersInfo[3].buffer;

    const int out_class_size = detectionParams.numClassesConfigured;
    //const float threshold = detectionParams.perClassThreshold[0];
    // assert(7 == detectionParams.perClassThreshold.size());
    const float thresholds[] = {0.3, 0.3, 0.3, 0.3, 0.3, 0.3, 0.3};

    const int keep_top_k = 200;
    const char* log_enable = std::getenv("ENABLE_DEBUG");

    // if(log_enable != NULL && std::stoi(log_enable)) {
    // std::cout <<"keep cout"
    //           <<p_keep_count[0] << std::endl;
    // }

#ifdef _ROS_PIPELINE_
    static ros::NodeHandle nh;
#ifdef LOWER
    static ros::Publisher pub = nh.advertise<BBoxArray>("/yolo_detect_lower/bbox_results", 1);
#elif defined UPPER
    static ros::Publisher pub = nh.advertise<BBoxArray>("/yolo_detect_upper/bbox_results", 1);
#endif
    BBoxArray ros_msg;
#endif

    for (int i = 0; i < p_keep_count[0] && objectList.size() <= keep_top_k; i++) {

        assert((int) p_classes[i] < out_class_size);
        if ( p_scores[i] < thresholds[(int)p_classes[i]]) continue;
        if (p_bboxes[4*i+2] < p_bboxes[4*i] || p_bboxes[4*i+3] < p_bboxes[4*i+1]) continue;

        if(log_enable != NULL && std::stoi(log_enable)) {
            std::cout << "label/conf/ x/y x/y -- "
                      << p_classes[i] << " " << p_scores[i] << " "
                      << p_bboxes[4*i] << " " << p_bboxes[4*i+1] << " " << p_bboxes[4*i+2] << " "<< p_bboxes[4*i+3] << " " << std::endl;
        }

        NvDsInferObjectDetectionInfo object;
        object.classId = (int) p_classes[i];
        object.detectionConfidence = p_scores[i];

        /* Clip object box co-ordinates to network resolution */
        object.left = CLIP(p_bboxes[4*i], 0, networkInfo.width - 1);
        object.top = CLIP(p_bboxes[4*i+1], 0, networkInfo.height - 1);
        object.width = CLIP((p_bboxes[4*i+2] - p_bboxes[4*i]), 0, networkInfo.width - 1);
        object.height = CLIP((p_bboxes[4*i+3] - p_bboxes[4*i+1]), 0, networkInfo.height - 1);

        objectList.push_back(object);

#ifdef _ROS_PIPELINE_
        /* construct ros message */
        //3:people, 5:rubbish_bin
        BBox bbox;
        bbox.class_id = object.classId;
        bbox.x_top_left = object.left;
        bbox.y_top_left = object.top;
        bbox.x_bottom_right = object.left + object.width;
        bbox.y_bottom_right = object.top + object.height;
        ros_msg.bboxes.push_back(bbox);
#endif
    // printf("ok"); fflush(0);
    }

#ifdef _ROS_PIPELINE_
    if(ros::ok()) pub.publish(ros_msg);
#endif

    return true;
}

/* Check that the custom function has been defined correctly */
CHECK_CUSTOM_PARSE_FUNC_PROTOTYPE(NvDsInferParseCustomYOLOV3TLT);
