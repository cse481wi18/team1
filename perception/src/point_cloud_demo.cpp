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
    if (argc < 2) {
        ROS_INFO("Usage: rosrun perception point_cloud_demo DATA_DIR");
        ros::spinOnce();
    }
    std::string data_dir(argv[1]);

     // LAB 30
    ros::init(argc, argv, "point_cloud_demo");
    ros::NodeHandle nh;
    ros::Publisher crop_pub =
        nh.advertise<sensor_msgs::PointCloud2>("cropped_cloud", 1, true);
    perception::Cropper cropper(crop_pub);
    ros::Subscriber sub =
        nh.subscribe("cloud_in", 1, &perception::Cropper::Callback, &cropper);


    ros::Publisher down_pub =
        nh.advertise<sensor_msgs::PointCloud2>("downsampled_cloud", 1, true);
        perception::Downsampler downsampler(down_pub);

    ros::Subscriber down_sub =
        nh.subscribe("cropped_cloud", 1, &perception::Downsampler::Callback, &downsampler);

    // LAB 31
    ros::Publisher table_pub =
        nh.advertise<sensor_msgs::PointCloud2>("table_cloud", 1, true);
    ros::Publisher marker_pub =
        nh.advertise<visualization_msgs::Marker>("visualization_marker", 1, true);
    ros::Publisher object_pub =
        nh.advertise<sensor_msgs::PointCloud2>("object_cloud", 1, true);

    // LAB 34
    std::vector<perception_msgs::ObjectFeatures> dataset;
    
   

    ros::Publisher coord_pub =
        nh.advertise<perception_msgs::ObjectCoordinates>("object_coordinates", 1, true);

    perception::Segmenter segmenter(table_pub, marker_pub, object_pub,coord_pub);

    ros::Subscriber seg_sub =
        nh.subscribe("cropped_cloud", 1, &perception::Segmenter::Callback, &segmenter);
    ros::spin();
    return 0;
}

