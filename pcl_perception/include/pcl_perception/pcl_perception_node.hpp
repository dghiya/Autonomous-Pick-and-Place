#ifndef PCL_PERCEPTION__PCL_PERCEPTION_NODE_HPP_
#define PCL_PERCEPTION__PCL_PERCEPTION_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose.hpp>

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include "pcl_perception/msg/detected_object_array.hpp"



namespace pcl_perception
{
    class PclPerception : public rclcpp::Node
    {
        public:
            PclPerception();

        private:

            typedef message_filters::sync_policies::ApproximateTime<
            sensor_msgs::msg::PointCloud2,
            sensor_msgs::msg::PointCloud2,
            sensor_msgs::msg::PointCloud2,
            sensor_msgs::msg::PointCloud2> SyncPolicy;


            void cloudCallback(
                const sensor_msgs::msg::PointCloud2::ConstSharedPtr& msg_fl,
                const sensor_msgs::msg::PointCloud2::ConstSharedPtr& msg_fr,
                const sensor_msgs::msg::PointCloud2::ConstSharedPtr& msg_bl,
                const sensor_msgs::msg::PointCloud2::ConstSharedPtr& msg_br);
            
            message_filters::Subscriber<sensor_msgs::msg::PointCloud2> sub_fl_;
            message_filters::Subscriber<sensor_msgs::msg::PointCloud2> sub_fr_;
            message_filters::Subscriber<sensor_msgs::msg::PointCloud2> sub_bl_;
            message_filters::Subscriber<sensor_msgs::msg::PointCloud2> sub_br_;
            
            std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> sync_;

            rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_;

            // rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;

            std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
            std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

            rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
            rclcpp::Publisher<pcl_perception::msg::DetectedObjectArray>::SharedPtr pose_pub_;
            rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr tray_pub_;
            

            double passthrough_min_;
            double passthrough_max_;
            double voxel_leaf_size_;
            double cluster_tolerance_;
            int min_cluster_size_;
            int max_cluster_size_;

    };
}

#endif