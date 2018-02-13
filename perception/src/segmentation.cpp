#include "perception/segmentation.h"

#include "pcl/PointIndices.h"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl_conversions/pcl_conversions.h"
#include "ros/ros.h"
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


typedef pcl::PointXYZRGB PointC;
typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloudC;

namespace perception {
    void SegmentSurface(PointCloudC::Ptr cloud, pcl::PointIndices::Ptr indices) {
        pcl::PointIndices indices_internal;
        pcl::SACSegmentation<PointC> seg;
        seg.setOptimizeCoefficients(true);
        // Search for a plane perpendicular to some axis (specified below).
        seg.setModelType(pcl::SACMODEL_PERPENDICULAR_PLANE);
        seg.setMethodType(pcl::SAC_RANSAC);
        // Set the distance to the plane for a point to be an inlier.
        seg.setDistanceThreshold(0.01);
        seg.setInputCloud(cloud);

        // Make sure that the plane is perpendicular to Z-axis, 10 degree tolerance.
        Eigen::Vector3f axis;
        axis << 0, 0, 1;
        seg.setAxis(axis);
        seg.setEpsAngle(pcl::deg2rad(10.0));

        // coeff contains the coefficients of the plane:
        // ax + by + cz + d = 0
        pcl::ModelCoefficients coeff;
        seg.segment(indices_internal, coeff);

        *indices = indices_internal;

        if (indices->indices.size() == 0) {
            ROS_ERROR("Unable to find surface.");
            return;
        }
        // // coeff contains the coefficients of the plane:
        // // ax + by + cz + d = 0
        // pcl::ModelCoefficients coeff;
        // // seg.segment(indices_internal, coeff);
        // double distance_above_plane;
        // ros::param::param("distance_above_plane", distance_above_plane, 0.005);

        // // Build custom indices that ignores points above the plane.
        // for (size_t i = 0; i < cloud->size(); ++i) {
        //     const PointC& pt = cloud->points[i];
        //     float val = coeff.values[0] * pt.x + coeff.values[1] * pt.y +
        //                 coeff.values[2] * pt.z + coeff.values[3];
        //     if (val <= distance_above_plane) {
        //         indices->indices.push_back(i);
        //     }
        // }

        // // Comment this out
        // //*indices = indices_internal;
        // if (indices->indices.size() == 0) {
        //     ROS_ERROR("Unable to find surface.");
        //     return;
        // }
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

    Segmenter::Segmenter(const ros::Publisher& surface_points_pub, const ros::Publisher& marker_pub, const ros::Publisher& object_pub ) {
        surface_points_pub_ = surface_points_pub;
        marker_pub_ = marker_pub;
        object_pub_ = object_pub;
    }

    void SegmentTabletopScene(PointCloudC::Ptr cloud,
                          std::vector<Object>* objects) {
         // Extract subset of original_cloud into subset_cloud:
        pcl::PointIndices::Ptr table_inliers(new pcl::PointIndices());
        pcl::ExtractIndices<PointC> extract;
        extract.setInputCloud(cloud);
        SegmentSurface(cloud, table_inliers);
        extract.setIndices(table_inliers);
        extract.filter(*cloud);

        ROS_INFO("Segmented to %ld points", cloud->size());
        sensor_msgs::PointCloud2 msg_out;

        std::vector<pcl::PointIndices> object_indices;
        SegmentSurfaceObjects(cloud, table_inliers, &object_indices);
        // We are reusing the extract object created earlier in the callback.
        extract.setNegative(true);
        extract.filter(*cloud);

         for (size_t i = 0; i < object_indices.size(); ++i) {
            // Reify indices into a point cloud of the object.
            pcl::PointIndices::Ptr indices(new pcl::PointIndices);
            *indices = object_indices[i];
            // TODO: fill in object_cloud using indices
            extract.setInputCloud(cloud);
            extract.setNegative(false);
            extract.setIndices(indices);
            extract.filter(*cloud);

            Object temp = 

            objects->push_back()
         }
    }

    void Segmenter::Callback(const sensor_msgs::PointCloud2& msg) {
        PointCloudC::Ptr cloud_unfiltered(new PointCloudC());
        pcl::fromROSMsg(msg, *cloud_unfiltered);
        PointCloudC::Ptr cloud(new PointCloudC());
        std::vector<int> index;
        pcl::removeNaNFromPointCloud(*cloud_unfiltered, *cloud, index);

        std::vector<Object> objects;
        SegmentTabletopScene(cloud, &objects);

        for (size_t i = 0; i < objects.size(); ++i) {
            const Object& object = objects[i];

            // Publish a bounding box around it.
            visualization_msgs::Marker object_marker;
            object_marker.ns = "objects";
            object_marker.id = i;
            object_marker.header.frame_id = "base_link";
            object_marker.type = visualization_msgs::Marker::CUBE;
            object_marker.pose = object.pose;
            object_marker.scale = object.dimensions;
            object_marker.color.g = 1;
            object_marker.color.a = 0.3;
            marker_pub_.publish(object_marker);
        }
    }

    //     for (size_t i = 0; i < object_indices.size(); ++i) {
    //         // Reify indices into a point cloud of the object.
    //         pcl::PointIndices::Ptr indices(new pcl::PointIndices);
    //         *indices = object_indices[i];
    //         PointCloudC::Ptr object_cloud(new PointCloudC());
    //         // TODO: fill in object_cloud using indices
    //         extract.setInputCloud(cloud);
    //         extract.setNegative(false);
    //         extract.setIndices(indices);
    //         extract.filter(*object_cloud);

    //         ROS_INFO("object to %ld points", object_cloud->size());

    //         // Publish a bounding box around it.
    //         visualization_msgs::Marker object_marker;
    //         object_marker.ns = "objects";
    //         object_marker.id = i;
    //         object_marker.header.frame_id = "base_link";
    //         object_marker.type = visualization_msgs::Marker::CUBE;
    //         GetAxisAlignedBoundingBox(object_cloud, &object_marker.pose,
    //                                     &object_marker.scale);
    //         object_marker.color.g = 1;
    //         object_marker.color.a = 0.3;
    //         marker_pub_.publish(object_marker);
    //     }

    // }


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