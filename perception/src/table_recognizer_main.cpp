#include "perception/crop.h"
#include "perception/downsampler.h"
#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"
#include "perception/segmentation.h"
#include "visualization_msgs/Marker.h"
#include <vector>

#include "perception_msgs/ObjectFeatures.h"
#include "perception_msgs/ObjectCoordinates.h"

int main(int argc, char** argv) {
     // LAB 30
    ros::init(argc, argv, "table_recognizer_main");
    ros::NodeHandle nh;

    ros::Publisher crop_pub =
        nh.advertise<sensor_msgs::PointCloud2>("cropped_cloud", 1, true);
    perception::Cropper cropper(crop_pub);
    ros::Subscriber flag_sub = nh.subscribe("perception_flag", 1, &perception::Cropper::FlagCallback, &cropper);

    ros::Subscriber sub =
        nh.subscribe("cloud_in", 1, &perception::Cropper::Callback, &cropper);

    // LAB 31
    ros::Publisher table_pub =
        nh.advertise<sensor_msgs::PointCloud2>("table_cloud", 1, true);
    ros::Publisher marker_pub =
        nh.advertise<visualization_msgs::Marker>("visualization_marker", 1, true);
    ros::Publisher object_pub =
        nh.advertise<sensor_msgs::PointCloud2>("object_cloud", 1, true);

    // LAB 34

    ros::Publisher coord_pub =
        nh.advertise<perception_msgs::ObjectCoordinates>("object_coordinates", 1, true);

    perception::Segmenter segmenter(table_pub, marker_pub, object_pub, coord_pub);

    ros::Subscriber seg_sub =
        nh.subscribe("cropped_cloud", 1, &perception::Segmenter::Callback, &segmenter);
    ros::spin();
    return 0;
}
