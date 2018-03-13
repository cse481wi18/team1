#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"
#include "std_msgs/Bool.h"
#include "tf/transform_listener.h"


namespace perception {
    class Cropper {
        public:
            Cropper(const ros::Publisher& pub);
            void Callback(const sensor_msgs::PointCloud2& msg);
            void FlagCallback(const std_msgs::Bool& msg);
         private:
            ros::Publisher pub_;
            tf::TransformListener tf_listener_; 
            bool should_crop_;
    };
}  // namespace perception