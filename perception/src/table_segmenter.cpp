#include "perception/table_segmenter.h"
#include "ros/ros.h"
#include "perception/segmentation.h"

namespace perception {
    TableSegmenter::TableSegmenter(){}

    void start() {
        ROS_INFO("Starting segmentation");
    }
}