#include <pcl_perception/pcl_perception_node.hpp>

// Include your generated custom message headers
#include "pcl_perception/msg/detected_object.hpp"
#include "pcl_perception/msg/detected_object_array.hpp"

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/common/centroid.h>
#include <pcl/common/common.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp> 
#include <algorithm>

namespace pcl_perception
{
    PclPerception::PclPerception() : Node("pcl_perception_node")
    {
        // Parameter setup
        passthrough_min_ = this->declare_parameter<double>("passthrough_min", 0.0);
        passthrough_max_ = this->declare_parameter<double>("passthrough_max", 1.5);
        voxel_leaf_size_     = this->declare_parameter<double>("voxel_leaf_size", 0.01);
        cluster_tolerance_   = this->declare_parameter<double>("cluster_tolerance", 0.02);
        min_cluster_size_    = this->declare_parameter<int>("min_cluster_size", 15);
        max_cluster_size_    = this->declare_parameter<int>("max_cluster_size", 25000);

        // 1. Initialize TF2 for frame transforms
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_); 

        // 2. Initialize the 4 message_filter subscribers
        auto rmw_qos_profile = rclcpp::SensorDataQoS().get_rmw_qos_profile();
        sub_fl_.subscribe(this, "/camera_fl/points", rmw_qos_profile);
        sub_fr_.subscribe(this, "/camera_fr/points", rmw_qos_profile);
        sub_bl_.subscribe(this, "/camera_bl/points", rmw_qos_profile);
        sub_br_.subscribe(this, "/camera_br/points", rmw_qos_profile);

        // 3. Initialize the Synchronizer
        sync_ = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(
            SyncPolicy(10), sub_fl_, sub_fr_, sub_bl_, sub_br_);
        sync_->registerCallback(&PclPerception::cloudCallback, this);

        cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("filtered_cloud", rclcpp::SensorDataQoS());
        marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("object_marker", 10);
        pose_pub_ = this->create_publisher<pcl_perception::msg::DetectedObjectArray>("target_poses", 10);
        tray_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("drop_target", rclcpp::QoS(1).transient_local().reliable());

        RCLCPP_INFO(this->get_logger(), "pcl_perception started with 4-camera synchronization");
    }

    void PclPerception::cloudCallback(
        const sensor_msgs::msg::PointCloud2::ConstSharedPtr& msg_fl,
        const sensor_msgs::msg::PointCloud2::ConstSharedPtr& msg_fr,
        const sensor_msgs::msg::PointCloud2::ConstSharedPtr& msg_bl,
        const sensor_msgs::msg::PointCloud2::ConstSharedPtr& msg_br)
    {
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr master_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr green_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

        auto transform_and_append = [&](const sensor_msgs::msg::PointCloud2::ConstSharedPtr& msg) {
            sensor_msgs::msg::PointCloud2 transformed_msg;
            try {
                geometry_msgs::msg::TransformStamped transform = tf_buffer_->lookupTransform(
                    "world", msg->header.frame_id, tf2::TimePointZero, tf2::durationFromSec(0.1));
                tf2::doTransform(*msg, transformed_msg, transform);

                pcl::PointCloud<pcl::PointXYZRGB> rgb_cloud;
                pcl::fromROSMsg(transformed_msg, rgb_cloud);

                for (const auto & p : rgb_cloud.points) {
                    if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) continue;
                    const bool is_green = (p.g > p.r + 40) && (p.g > p.b + 40);
                    if (is_green) {
                        green_cloud->push_back(p);
                    } else {
                        master_cloud->push_back(p);
                    }
                }
            } catch (const tf2::TransformException & ex) {
                RCLCPP_WARN(this->get_logger(), "Transform error: %s", ex.what());
            }
        };

        transform_and_append(msg_fl);
        transform_and_append(msg_fr);
        transform_and_append(msg_bl);
        transform_and_append(msg_br);

        // --- 1. Dynamic Drop Tray Detection (Centroid) ---
        if (green_cloud->size() > 200) {
            Eigen::Vector4f tray_centroid;
            pcl::compute3DCentroid(*green_cloud, tray_centroid);

            pcl::PointXYZRGB min_pt, max_pt;
            pcl::getMinMax3D(*green_cloud, min_pt, max_pt);

            geometry_msgs::msg::PoseStamped tray;
            tray.header.frame_id = "world";
            tray.header.stamp = msg_fl->header.stamp;
            tray.pose.position.x = tray_centroid[0];
            tray.pose.position.y = tray_centroid[1];
            tray.pose.position.z = max_pt.z;
            tray.pose.orientation.w = 1.0;
            tray_pub_->publish(tray);
        }

        if (master_cloud->empty()) return;

        // --- 2. Workspace Crop ---
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_crop_x(new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::PassThrough<pcl::PointXYZRGB> pass_x;
        pass_x.setInputCloud(master_cloud);
        pass_x.setFilterFieldName("x");
        pass_x.setFilterLimits(0.20, 0.85);
        pass_x.filter(*cloud_crop_x);

        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_crop_y(new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::PassThrough<pcl::PointXYZRGB> pass_y;
        pass_y.setInputCloud(cloud_crop_x);
        pass_y.setFilterFieldName("y");
        pass_y.setFilterLimits(-0.40, 0.12);
        pass_y.filter(*cloud_crop_y);

        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_passthrough(new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::PassThrough<pcl::PointXYZRGB> pass_z;
        pass_z.setInputCloud(cloud_crop_y);
        pass_z.setFilterFieldName("z");
        pass_z.setFilterLimits(0.418, 0.52);
        pass_z.filter(*cloud_passthrough);

        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_objects(new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::VoxelGrid<pcl::PointXYZRGB> voxel;
        voxel.setInputCloud(cloud_passthrough);
        voxel.setLeafSize(voxel_leaf_size_, voxel_leaf_size_, voxel_leaf_size_);
        voxel.filter(*cloud_objects);

        if (cloud_objects->empty()) return;

        pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>);
        tree->setInputCloud(cloud_objects);

        std::vector<pcl::PointIndices> cluster_indices;
        pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ec;
        ec.setClusterTolerance(cluster_tolerance_);
        ec.setMinClusterSize(10);
        ec.setMaxClusterSize(max_cluster_size_);
        ec.setSearchMethod(tree);
        ec.setInputCloud(cloud_objects);
        ec.extract(cluster_indices);

        visualization_msgs::msg::MarkerArray marker_array;
        pcl_perception::msg::DetectedObjectArray object_array_msg;
        object_array_msg.header.stamp = msg_fl->header.stamp;
        object_array_msg.header.frame_id = "world";

        int object_id = 1;
        for (const auto &cluster : cluster_indices)
        {
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZRGB>);
            long r_sum = 0, g_sum = 0, b_sum = 0;

            for (const auto &idx : cluster.indices) {
                const auto& pt = (*cloud_objects)[idx];
                cloud_cluster->push_back(pt);
                r_sum += pt.r;
                g_sum += pt.g;
                b_sum += pt.b;
            }

            long n = cluster.indices.size();
            long r_avg = r_sum / n, g_avg = g_sum / n, b_avg = b_sum / n;

            std::string obj_name = "";
            bool is_red = (r_avg > g_avg + 40 && r_avg > b_avg + 40);
            bool is_blue = (b_avg > r_avg + 40 && b_avg > g_avg + 40);

            if (is_red) {
                obj_name = "Red Box";
            } else if (is_blue) {
                obj_name = "Blue Cylinder";
            } else {
                continue; // Ignore background/noise
            }

            Eigen::Vector4f centroid;
            pcl::compute3DCentroid(*cloud_cluster, centroid);

            pcl::PointXYZRGB min_pt, max_pt;
            pcl::getMinMax3D(*cloud_cluster, min_pt, max_pt);

            // Create RViz Bounding Box Marker
            visualization_msgs::msg::Marker bbox;
            bbox.header.frame_id = "world";
            bbox.header.stamp = msg_fl->header.stamp;
            bbox.ns = "cluster_boxes";
            bbox.id = object_id;
            bbox.type = visualization_msgs::msg::Marker::CUBE;
            bbox.action = visualization_msgs::msg::Marker::ADD;
            bbox.lifetime = rclcpp::Duration::from_seconds(0.2);

            bbox.pose.position.x = centroid[0];
            bbox.pose.position.y = centroid[1];
            bbox.pose.position.z = centroid[2];
            bbox.pose.orientation.w = 1.0;

            bbox.scale.x = max_pt.x - min_pt.x;
            bbox.scale.y = max_pt.y - min_pt.y;
            bbox.scale.z = max_pt.z - min_pt.z;

            bbox.color.r = is_red ? 1.0f : 0.0f;
            bbox.color.g = 0.2f;
            bbox.color.b = is_blue ? 1.0f : 0.0f;
            bbox.color.a = 0.6f;
            marker_array.markers.push_back(bbox);

            // Populate custom message object
            pcl_perception::msg::DetectedObject obj_msg;
            obj_msg.name = obj_name;
            obj_msg.id = object_id++;
            obj_msg.pose.position.x = centroid[0];
            obj_msg.pose.position.y = centroid[1];
            obj_msg.pose.position.z = centroid[2];
            
            // Clean, valid unit quaternion
            obj_msg.pose.orientation.x = 0.0;
            obj_msg.pose.orientation.y = 0.0;
            obj_msg.pose.orientation.z = 0.0;
            obj_msg.pose.orientation.w = 1.0;

            object_array_msg.objects.push_back(obj_msg);
        }

        pose_pub_->publish(object_array_msg);
        marker_pub_->publish(marker_array);

        sensor_msgs::msg::PointCloud2 output;
        pcl::toROSMsg(*cloud_objects, output);
        output.header.frame_id = "world";
        output.header.stamp = msg_fl->header.stamp;
        cloud_pub_->publish(output);
    }
}