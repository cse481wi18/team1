#include "pcl/PointIndices.h"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Vector3.h"
#include "visualization_msgs/Marker.h"
#include "perception/object.h"
#include "pcl/ModelCoefficients.h"
#include "perception/box_fitter.h"
#include "perception/object_recognizer.h"
#include "tf/transform_listener.h"

namespace perception {
// Finds the largest horizontal surface in the given point cloud.
// This is useful for adding a collision object to MoveIt.
//
// Args:
//  cloud: The point cloud to extract a surface from.
//  indices: The indices of points in the point cloud that correspond to the
//    surface. Empty if no surface was found.
void SegmentSurface(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,
                    pcl::PointIndices::Ptr indices, pcl::ModelCoefficients::Ptr coeff);

// Computes the axis-aligned bounding box of a point cloud.
//
// Args:
//  cloud: The point cloud
//  pose: The output pose. Because this is axis-aligned, the orientation is just
//    the identity. The position refers to the center of the box.
//  dimensions: The output dimensions, in meters.
void GetAxisAlignedBoundingBox(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,
                               geometry_msgs::Pose* pose,
                               geometry_msgs::Vector3* dimensions);

// Does a complete tabletop segmentation pipeline.
//
// Args:
//  cloud: The point cloud with the surface and the objects above it.
//  objects: The output objects.
void SegmentTabletopScene(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,
                          std::vector<Object>* objects, const ros::Publisher& surface_points_pub_);


void SegmentSurfaceObjects(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,
                           pcl::PointIndices::Ptr surface_indices,
                           std::vector<pcl::PointIndices>* object_indices);
                           
class Segmenter {
 public:
 
  void Callback(const sensor_msgs::PointCloud2& msg);
  Segmenter(const ros::Publisher& surface_points_pub,
            const ros::Publisher& marker_pub,
            const ros::Publisher& object_pub,
            const ros::Publisher& coord_pub,
            const ObjectRecognizer& object_recognizer);

 private:
  ros::Publisher surface_points_pub_;
  ros::Publisher marker_pub_;
  ros::Publisher object_pub_;
  ros::Publisher coord_pub_;
  ObjectRecognizer recognizer_;
  tf::TransformListener tf_listener_; 
};
}  // namespace perception