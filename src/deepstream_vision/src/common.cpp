#include "common.hpp"
#include <string>

const char * Pipeline::indexed_name(const char * basename) {
    static std::string buffer;
    buffer = std::string(basename) + std::to_string(object_index++);
    return buffer.c_str();
}

GstElement *  Pipeline::make(const char * element_name) {
    return gst_element_factory_make(element_name,  indexed_name(element_name));
}

Pipeline::Pipeline(){
    object_index = 0;
}