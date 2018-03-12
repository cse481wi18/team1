#include "perception/crop.h"
#include "pcl_conversions/pcl_conversions.h"
#include "pcl/filters/crop_box.h"
#include "pcl/filters/voxel_grid.h"
#include "pcl_ros/transforms.h"



// Add these typedefs after your #includes
typedef pcl::PointXYZRGB PointC;
typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloudC;


perception::Cropper::Cropper(const ros::Publisher& pub) : pub_(pub), tf_listener_() {}

void perception::Cropper::Callback(const sensor_msgs::PointCloud2& msg) {


  tf_listener_.waitForTransform("base_link", msg.header.frame_id,                     
                                    ros::Time(0), ros::Duration(5.0));                       
        tf::StampedTransform transform;                                                       
        try {                                                                                 
            tf_listener_.lookupTransform("base_link", msg.header.frame_id,                    
                                        ros::Time(0), transform);                               
        } catch (tf::LookupException& e) {                                                    
            std::cerr << e.what() << std::endl;                                                 
            return ;                                                                           
        } catch (tf::ExtrapolationException& e) {                                             
            std::cerr << e.what() << std::endl;                                                 
            return ;                                                                           
        }       

    sensor_msgs::PointCloud2 cloud_out;        


    pcl_ros::transformPointCloud("base_link", transform, msg, cloud_out);
  PointCloudC::Ptr cloud(new PointCloudC());
    pcl::fromROSMsg(cloud_out, *cloud);
      ROS_INFO("Got point cloud with %ld points", cloud->size());

  ros::NodeHandle nh;
  ros::Publisher pub = nh.advertise<sensor_msgs::PointCloud2>("test", 1, true);
  // while (pub.getNumSubscribers() == 0) {
    //ros::spinOnce();
  //}
  pub.publish(cloud_out);

  PointCloudC::Ptr cropped_cloud(new PointCloudC());
  double min_x, min_y, min_z, max_x, max_y, max_z;
    ros::param::param("crop_min_x", min_x, 0.3);
    ros::param::param("crop_min_y", min_y, -1.0);
    ros::param::param("crop_min_z", min_z, 0.5);
    ros::param::param("crop_max_x", max_x, 0.9);
    ros::param::param("crop_max_y", max_y, 1.0);
    ros::param::param("crop_max_z", max_z, 1.5);
    
    Eigen::Vector4f min_pt(min_x, min_y, min_z, 1);
    Eigen::Vector4f max_pt(max_x, max_y, max_z, 1);
    ROS_INFO_STREAM("min: " << min_pt << ", max: " << max_pt);
    pcl::CropBox<PointC> crop;
    crop.setInputCloud(cloud);
    crop.setMin(min_pt);
    crop.setMax(max_pt);
    crop.filter(*cropped_cloud);
    ROS_INFO("Cropped to %ld points", cropped_cloud->size());
    sensor_msgs::PointCloud2 msg_out;
    pcl::toROSMsg(*cropped_cloud, msg_out);
    pub_.publish(msg_out);
}
