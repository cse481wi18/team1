#include "perception/segmentation.h"

#include "pcl/PointIndices.h"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl_conversions/pcl_conversions.h"
#include "ros/ros.h"
#include "sensor_msgs/point_cloud_conversion.h"
#include "sensor_msgs/PointCloud2.h"
#include "pcl/common/angles.h"
#include "pcl/sample_consensus/method_types.h"
#include "pcl/sample_consensus/model_types.h"
#include "pcl/segmentation/sac_segmentation.h"
#include "pcl/filters/extract_indices.h"
#include "pcl/common/common.h"
#include <pcl/segmentation/extract_clusters.h>
#include "simple_grasping/shape_extraction.h"
#include "perception/object.h"
#include "perception/feature_extraction.h"
#include "perception_msgs/ObjectCoordinates.h"
#include "geometry_msgs/Point.h"
#include "pcl_ros/transforms.h"
#include "tf/transform_listener.h"
#include "Eigen/Dense"
#include "math.h"


typedef pcl::PointXYZRGB PointC;
typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloudC;

namespace perception {
    void NormalizeBox(Eigen::Matrix3f matrix, geometry_msgs::Vector3 dim, Eigen::Matrix3f* matrix_norm, geometry_msgs::Vector3* dim_norm) {
        float dims[3];
        dims[0] = dim.x;
        dims[1] = dim.y;
        dims[2] = dim.z;

        float max_x = -2;
        int max_x_index = -1;
        for (int i = 0; i< 3; i++) {
            if (fabs(matrix(0, i)) > max_x) {
                max_x = fabs(matrix(0, i));
                max_x_index = i;
            }
        }
        if (matrix(0, max_x_index) < 0)
            matrix_norm->col(0) = -1*matrix.col(max_x_index);
        else 
            matrix_norm->col(0) = matrix.col(max_x_index);
        dim_norm->x = dims[max_x_index];


        float max_y = -2;
        int max_y_index = -1;
        for (int i = 0; i< 3; i++) {
            if (fabs(matrix(1, i)) > max_y) {
                max_y = fabs(matrix(1, i));
                max_y_index = i;
            }
        }
       if (matrix(1, max_y_index) < 0)
            matrix_norm->col(1) = -1*matrix.col(max_y_index);
        else 
            matrix_norm->col(1) = matrix.col(max_y_index);
        dim_norm->y = dims[max_y_index];

        float max_z = -2;
        int max_z_index = -1;
        for (int i = 0; i< 3; i++) {
            if (fabs(matrix(2, i)) > max_z) {
                max_z = fabs(matrix(2, i));
                max_z_index = i;
            }
        }
        if (matrix(2, max_z_index) < 0)
            matrix_norm->col(2) = -1*matrix.col(max_z_index);
        else 
            matrix_norm->col(2) = matrix.col(max_z_index);
        dim_norm->z = dims[max_z_index];
    }


    int SegmentSurface(PointCloudC::Ptr cloud, pcl::PointIndices::Ptr indices, pcl::ModelCoefficients::Ptr coeff) {
        pcl::PointIndices indices_internal;
        pcl::SACSegmentation<PointC> seg;
        ROS_INFO("beginning of segment surface");
        seg.setOptimizeCoefficients(true);
        // Search for a plane perpendicular to some axis (specified below).
        seg.setModelType(pcl::SACMODEL_PARALLEL_PLANE);
        seg.setMethodType(pcl::SAC_RANSAC);
        // Set the distance to the plane for a point to be an inlier.
        seg.setDistanceThreshold(.01);
        seg.setInputCloud(cloud);

        // Make sure that the plane is perpendicular to Z-axis, 10 degree tolerance.
        Eigen::Vector3f axis;
        axis << 1, 0, 0;
        seg.setAxis(axis);
        seg.setEpsAngle(pcl::deg2rad(10.0));

        // coeff contains the coefficients of the plane:
        // ax + by + cz + d = 0
        seg.segment(indices_internal, *coeff);
   

        double distance_above_plane;
        ros::param::param("distance_above_plane", distance_above_plane, 0.05);
 
        // Build custom indices that ignores points above the plane.
        for (size_t i = 0; i < cloud->size(); ++i) {
            const PointC& pt = cloud->points[i];
            float val = coeff->values[0] * pt.x + coeff->values[1] * pt.y + coeff->values[2] * pt.z + coeff->values[3];
            if (val <= distance_above_plane) {
                indices->indices.push_back(i);
            }
        }
        // Comment this out
        //*indices = indices_internal;
        if (indices->indices.size() == 0) {
            //ROS_ERROR("Unable to find surface.");
            return false;
        }
        return true;
    }


    // Computes the axis-aligned bounding box of a point cloud.
    //
    // Args:
    //  cloud: The point cloud
    //  pose: The output pose. Because this is axis-aligned, the orientation is just
    //    the identity. The position refers to the center of the box.
    //  dimensions: The output dimensions, in meters.
    void GetAxisAlignedBoundingBox(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,
                                geometry_msgs::Pose* pose,
                                geometry_msgs::Vector3* dimensions) {
        PointC min;
        PointC max;
        pcl::getMinMax3D<PointC>(*cloud, min, max);
         ROS_INFO("POSE %f %f %f %f ", (float)  min.x, (float)min.y,(float)  max.x, (float)max.y);

        pose->position.x = (max.x + min.x) / 2;
        pose->position.y = (max.y + min.y) / 2;
        pose->position.z = (max.z + min.z) / 2;

        pose->orientation.x = 0;
        pose->orientation.y = 0;
        pose->orientation.z = 0;
        pose->orientation.w = 1;


        dimensions->x = max.x - min.x;
        dimensions->y = max.y - min.y;
        dimensions->z = max.z - min.z;

        ROS_INFO("POSE %f %f %f", (float)  pose->position.x,(float)  pose->position.y,(float) pose->position.z);
        ROS_INFO("Dim %f %f %f", (float) dimensions->x,(float) dimensions->y,(float)dimensions->z);
    }

   Segmenter::Segmenter(const ros::Publisher& surface_points_pub,
                     const ros::Publisher& marker_pub,
                     const ros::Publisher& object_pub,
                     const ros::Publisher& coord_pub)
    : surface_points_pub_(surface_points_pub),
      marker_pub_(marker_pub),
      object_pub_(object_pub),
      coord_pub_(coord_pub),
      tf_listener_() {}

    void SegmentTabletopScene(PointCloudC::Ptr cloud,
                          std::vector<Object>* objects, const ros::Publisher& surface_points_pub_) {

        pcl::PointIndices::Ptr table_inliers(new pcl::PointIndices());

        PointCloudC::Ptr segmented(new PointCloudC());

        // Extract subset of original_cloud into subset_cloud:
        pcl::ExtractIndices<PointC> extract;
        extract.setInputCloud(cloud);
        pcl::ModelCoefficients::Ptr model(new pcl::ModelCoefficients);

        int success = SegmentSurface(cloud, table_inliers, model);
        extract.setIndices(table_inliers);
        extract.filter(*segmented);

        if (!success) {
            return;
        }

        ROS_INFO("Segmented to %ld points", segmented->size());
        sensor_msgs::PointCloud2 msg_out;
        pcl::toROSMsg(*segmented, msg_out);
        surface_points_pub_.publish(msg_out);
        shape_msgs::SolidPrimitive shape;
        geometry_msgs::Pose pose;
            
      
    
        pcl::PointCloud<pcl::PointXYZRGB> output;
         FitBox(*segmented, model, output, shape, pose);
        geometry_msgs::Vector3 dimensions;
          //GetAxisAlignedBoundingBox(segmented, &pose,&dimensions);
                    
            dimensions.x = shape.dimensions[0];
            dimensions.y = shape.dimensions[1];
            dimensions.z = shape.dimensions[2];

        ROS_INFO("POSE %f %f %f", (float)  pose.position.x,(float)  pose.position.y,(float) pose.position.z);
        ROS_INFO("Dim %f %f %f", (float) dimensions.x,(float) dimensions.y,(float)dimensions.z);

        Eigen::Quaternionf q(pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z);

        Eigen::Matrix3f matrix(q);
        Eigen::Matrix3f matrix_norm;
        geometry_msgs::Vector3 dim_norm;
        
        NormalizeBox(matrix, dimensions, &matrix_norm, &dim_norm);
        ROS_INFO_STREAM_THROTTLE(1, matrix);
        ROS_INFO_STREAM_THROTTLE(1, matrix_norm);

        Eigen::Quaternionf q_norm(matrix_norm);

        geometry_msgs::Pose pose_norm;
        pose_norm.position = pose.position;

        pose_norm.orientation.w = q_norm.w();
        pose_norm.orientation.x = q_norm.x();
        pose_norm.orientation.y = q_norm.y();
        pose_norm.orientation.z = q_norm.z();

	    ROS_INFO_STREAM("normalized pose: " << pose_norm);

        Object object;
        object.name = "table";
        object.confidence = .5;
        object.cloud = segmented;
        object.pose = pose_norm;
        object.dimensions = dim_norm;

        objects->push_back(object);
    }

    void Segmenter::Callback(const sensor_msgs::PointCloud2& msg) {
        
        PointCloudC::Ptr cloud_w_nan(new PointCloudC());

        pcl::fromROSMsg(msg, *cloud_w_nan);
        PointCloudC::Ptr cloud(new PointCloudC());
        std::vector<int> index;
         pcl::removeNaNFromPointCloud(*cloud_w_nan, *cloud, index);


       
        std::vector<Object> objects;
        SegmentTabletopScene(cloud, &objects, surface_points_pub_);

        for (size_t i = 0; i < objects.size(); ++i) {
            const Object& object = objects[i];

            // Publish a bounding box around it.
            visualization_msgs::Marker object_marker;
            object_marker.ns = "table";
            object_marker.id = i;
            object_marker.header.frame_id = "base_link";
            object_marker.type = visualization_msgs::Marker::CUBE;
            object_marker.pose = object.pose;
            object_marker.scale = object.dimensions;
            object_marker.color.g = 1;
            object_marker.color.a = 0.3;
            marker_pub_.publish(object_marker);

            perception_msgs::ObjectCoordinates object_coords;
            object_coords.object_name = object.name;
            object_coords.pose = object.pose;
            
            geometry_msgs::Point point1;
            geometry_msgs::Point point2;
            geometry_msgs::Point point3;
            geometry_msgs::Point point4;

            geometry_msgs::Pose left_corner;
            left_corner.orientation = object.pose.orientation;
            left_corner.position.x = object.pose.position.x - object.dimensions.x/2;
            left_corner.position.y = object.pose.position.y + object.dimensions.y/2;
            left_corner.position.z = object.pose.position.z;

            geometry_msgs::Vector3 dim;
            dim.x = .07;
            dim.y = .07;
            dim.z = .07;

            visualization_msgs::Marker left_marker;
            left_marker.ns = "left_corner";
            left_marker.id = i*2;
            left_marker.header.frame_id = "base_link";
            left_marker.type = visualization_msgs::Marker::CUBE;
            left_marker.pose = left_corner;
            left_marker.scale = dim;
            left_marker.color.b = 1;
            left_marker.color.a = 0.5;
            marker_pub_.publish(left_marker);

            ROS_INFO("Object dims %f %f %f", object.dimensions.x, object.dimensions.y, object.dimensions.z);

            point1.x = object.pose.position.x - object.dimensions.x/2;
            point1.y = object.pose.position.y - object.dimensions.y/2;
            point1.z = object.pose.position.z;

            point2.x = object.pose.position.x - object.dimensions.x/2;
            point2.y = object.pose.position.y + object.dimensions.y/2;
            point2.z = object.pose.position.z;

            point3.x = object.pose.position.x + object.dimensions.x/2;
            point3.y = object.pose.position.y + object.dimensions.y/2;
            point3.z = object.pose.position.z;

            point4.x = object.pose.position.x + object.dimensions.x/2;
            point4.y = object.pose.position.y - object.dimensions.y/2;
            point4.z = object.pose.position.z;

            object_coords.pose = left_corner;

            object_coords.corners.push_back(point1);
            object_coords.corners.push_back(point2);
            object_coords.corners.push_back(point3);
            object_coords.corners.push_back(point4);
           
            coord_pub_.publish(object_coords);
        }
    }

    void SegmentSurfaceObjects(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,
                           pcl::PointIndices::Ptr surface_indices,
                           std::vector<pcl::PointIndices>* object_indices) {
        pcl::ExtractIndices<PointC> extract;
        pcl::PointIndices::Ptr above_surface_indices(new pcl::PointIndices());
        extract.setInputCloud(cloud);
        extract.setIndices(surface_indices);
        extract.setNegative(true);
        extract.filter(above_surface_indices->indices);

        double cluster_tolerance;
        int min_cluster_size, max_cluster_size;
        ros::param::param("ec_cluster_tolerance", cluster_tolerance, 0.01);
        ros::param::param("ec_min_cluster_size", min_cluster_size, 10);
        ros::param::param("ec_max_cluster_size", max_cluster_size, 10000);

        pcl::EuclideanClusterExtraction<PointC> euclid;
        euclid.setInputCloud(cloud);
        euclid.setIndices(above_surface_indices);
        euclid.setClusterTolerance(cluster_tolerance);
        euclid.setMinClusterSize(min_cluster_size);
        euclid.setMaxClusterSize(max_cluster_size);
        euclid.extract(*object_indices);

        // Find the size of the smallest and the largest object,
        // where size = number of points in the cluster
        size_t min_size = std::numeric_limits<size_t>::max();
        size_t max_size = std::numeric_limits<size_t>::min();
        for (size_t i = 0; i < object_indices->size(); ++i) {
           ROS_INFO("OBJECT with %ld \n", (size_t)object_indices->at(i).indices.size());
            size_t cluster_size = object_indices->at(i).indices.size();

            if (cluster_size < min_cluster_size) {
                min_cluster_size = cluster_size;
            }
            if (cluster_size > max_cluster_size) {
                max_cluster_size = cluster_size;
            }
        }

        ROS_INFO("Found %ld objects, min size: %ld, max size: %ld",
                object_indices->size(), min_size, max_size);          
    }
}  // namespace perception
