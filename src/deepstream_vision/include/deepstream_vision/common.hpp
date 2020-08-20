#ifndef _DEEPSTREAM_VISION_COMMON_H_
#define _DEEPSTREAM_VISION_COMMON_H_

#include <cassert>
#include <gst/gst.h>

#define IMAGE_WIDTH 800
#define IMAGE_HEIGHT 600
#define UNWARP_WIDTH 416
#define UNWARP_HEIGHT 416
#define IMAGE_FPS 30

/* Muxer batch formation timeout. Should ideally be set
 * based on the fastest source's framerate. */
#define MUXER_BATCH_TIMEOUT_USEC 4000

class Pipeline {
    public:
        Pipeline();
        virtual ~Pipeline(){}
        virtual void spin() = 0; 
        virtual const char * indexed_name(const char * basename);
        virtual GstElement * make(const char *);
        virtual GstElement * as_element() = 0;
    private:
        int object_index;
};

class Node {
    public:
        virtual GstElement * as_source_pad(void) = 0;
};

#endif