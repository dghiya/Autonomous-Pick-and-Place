#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <std_srvs/srv/empty.hpp>
#include <chrono>
#include <thread>
#include <mutex>
#include <cmath>
#include "pcl_perception/msg/detected_object_array.hpp"
#include "pcl_perception/msg/detected_object.hpp"

class PickPlannerNode : public rclcpp::Node
{
public:
    PickPlannerNode() : Node("pick_planner_node", rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true))
    {
        RCLCPP_INFO(this->get_logger(), "Pick Planner Node started. Booting MoveIt...");
        clear_octomap_client_ = this->create_client<std_srvs::srv::Empty>("/clear_octomap");

        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    }

    void initializeMoveIt()
    {
        move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(shared_from_this(), "ur_manipulator");
        gripper_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(shared_from_this(), "gripper");

        move_group_->setPoseReferenceFrame("world");
        move_group_->setEndEffectorLink("tool0");
        move_group_->startStateMonitor();

        RCLCPP_INFO(get_logger(), "MoveIt planning frame: %s", move_group_->getPlanningFrame().c_str());
        RCLCPP_INFO(get_logger(), "MoveIt end-effector link: %s", move_group_->getEndEffectorLink().c_str());

        gripper_group_->setMaxVelocityScalingFactor(1.0);
        gripper_group_->setMaxAccelerationScalingFactor(1.0);
        move_group_->setMaxVelocityScalingFactor(0.1);
        move_group_->setMaxAccelerationScalingFactor(0.1);

        goHome();

        // Capture the true downward orientation directly from the robot's safe home pose!
        downward_orientation_ = move_group_->getCurrentPose("tool0").pose.orientation;
        RCLCPP_INFO(get_logger(), "Captured true downward orientation: x=%.3f, y=%.3f, z=%.3f, w=%.3f",
                    downward_orientation_.x, downward_orientation_.y, downward_orientation_.z, downward_orientation_.w);

        gripper_group_->setNamedTarget("open");
        gripper_group_->move();

        tray_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "drop_target", rclcpp::QoS(1).transient_local().reliable(),
            [this](geometry_msgs::msg::PoseStamped::SharedPtr m) {
                std::lock_guard<std::mutex> lock(data_mutex_);
                if (!have_tray_) {
                    tray_pose_ = m->pose;
                    have_tray_ = true;
                }
            });

        pose_sub_ = this->create_subscription<pcl_perception::msg::DetectedObjectArray>(
            "target_poses", rclcpp::QoS(1).best_effort(),
            [this](pcl_perception::msg::DetectedObjectArray::SharedPtr msg) {
                std::lock_guard<std::mutex> lock(data_mutex_);
                latest_detected_objects_ = *msg;
                last_pose_received_ = this->now();
                have_pose_timestamp_ = true;
            });

        worker_thread_ = std::thread(&PickPlannerNode::runPickAndPlaceLoop, this);
        worker_thread_.detach();
    }

private:
    bool getTrueToolPose(double & x, double & y, double & z)
    {
        try {
            auto tf = tf_buffer_->lookupTransform("world", "tool0", tf2::TimePointZero, tf2::durationFromSec(0.5));
            x = tf.transform.translation.x;
            y = tf.transform.translation.y;
            z = tf.transform.translation.z;
            return true;
        } catch (const tf2::TransformException & ex) {
            RCLCPP_WARN(this->get_logger(), "TF lookup failed: %s", ex.what());
            return false;
        }
    }

    void goHome()
    {
        move_group_->clearPoseTargets();
        std::vector<double> home_joint_positions = {0.0, -1.80, 1.40, -1.17, -1.57, 0.0};
        move_group_->setJointValueTarget(home_joint_positions);
        moveit::planning_interface::MoveGroupInterface::Plan home_plan;
        if (move_group_->plan(home_plan) == moveit::core::MoveItErrorCode::SUCCESS) {
            move_group_->execute(home_plan);
        } else {
            RCLCPP_WARN(this->get_logger(), "goHome() plan failed, continuing from current pose.");
        }
        move_group_->clearPoseTargets();
        rclcpp::sleep_for(std::chrono::milliseconds(500));
    }

    void clearOctomap()
    {
        if (clear_octomap_client_->service_is_ready()) {
            auto req = std::make_shared<std_srvs::srv::Empty::Request>();
            clear_octomap_client_->async_send_request(req);
            rclcpp::sleep_for(std::chrono::milliseconds(500));
        }
    }

    bool findObjectByName(const std::string & target_name, geometry_msgs::msg::Pose & out_pose, geometry_msgs::msg::Pose & out_tray)
    {
        std::lock_guard<std::mutex> lock(data_mutex_);
        if (!have_tray_ || latest_detected_objects_.objects.empty() || !have_pose_timestamp_) return false;

        const double age_seconds = (this->now() - last_pose_received_).seconds();
        if (age_seconds < 0.0 || age_seconds > 2.0) {
            return false;
        }

        for (const auto & obj_item : latest_detected_objects_.objects) {
            if (obj_item.name == target_name) {
                RCLCPP_INFO(get_logger(), "Found object: %s at (%.3f, %.3f, %.3f)", 
                            obj_item.name.c_str(), obj_item.pose.position.x, obj_item.pose.position.y, obj_item.pose.position.z);
                out_pose = obj_item.pose;
                out_tray = tray_pose_;
                return true;
            }
        }
        return false;
    }

    bool executeSinglePickAndPlace(const std::string & obj_name, int slot_index)
    {
        RCLCPP_INFO(get_logger(), "=== Waiting to detect %s ===", obj_name.c_str());

        geometry_msgs::msg::Pose obj, tray;
        while (rclcpp::ok()) {
            if (findObjectByName(obj_name, obj, tray)) break;
            rclcpp::sleep_for(std::chrono::milliseconds(200));
        }

        const double tcp_len = 0.163;
        const double grasp_z = obj.position.z + tcp_len;

        RCLCPP_INFO(get_logger(), "[LOCKED] %s Pose: (x=%.3f, y=%.3f, z=%.3f) | Tray Pose: (x=%.3f, y=%.3f, z=%.3f)",
                    obj_name.c_str(), obj.position.x, obj.position.y, obj.position.z,
                    tray.position.x, tray.position.y, tray.position.z);

        clearOctomap();

        geometry_msgs::msg::Pose hover = obj;
        hover.orientation = downward_orientation_; 
        hover.position.z = grasp_z + 0.08;

        move_group_->clearPoseTargets();

        // --- STEP 1: Fast Analytical Base Pivot (Bypasses OMPL search bottleneck) ---
        std::vector<double> current_joints = move_group_->getCurrentJointValues();
        double target_pan = std::atan2(hover.position.y, hover.position.x);
        
        double diff = target_pan - current_joints[0];
        while (diff > M_PI)  { target_pan -= 2.0 * M_PI; diff = target_pan - current_joints[0]; }
        while (diff < -M_PI) { target_pan += 2.0 * M_PI; diff = target_pan - current_joints[0]; }

        current_joints[0] = target_pan;
        move_group_->setJointValueTarget(current_joints);

        moveit::planning_interface::MoveGroupInterface::Plan pivot_plan;
        if (move_group_->plan(pivot_plan) != moveit::core::MoveItErrorCode::SUCCESS ||
            move_group_->execute(pivot_plan) != moveit::core::MoveItErrorCode::SUCCESS) {
            RCLCPP_ERROR(get_logger(), "Failed base pivot for %s", obj_name.c_str());
            return false;
        }
        move_group_->clearPoseTargets();
        rclcpp::sleep_for(std::chrono::milliseconds(300));

        // --- STEP 2: Safe Joint-Space Hover from Aligned Posture ---
        move_group_->setStartStateToCurrentState();
        move_group_->setPoseTarget(hover, "tool0");
        
        moveit::planning_interface::MoveGroupInterface::Plan hover_plan;
        if (move_group_->plan(hover_plan) != moveit::core::MoveItErrorCode::SUCCESS ||
            move_group_->execute(hover_plan) != moveit::core::MoveItErrorCode::SUCCESS) {
            RCLCPP_ERROR(get_logger(), "Failed joint-space hover for %s", obj_name.c_str());
            return false;
        }
        move_group_->clearPoseTargets();

        // --- STEP 3: Short Cartesian Snap for Precision ---
        std::vector<geometry_msgs::msg::Pose> hover_pts{hover};
        moveit_msgs::msg::RobotTrajectory hover_traj;
        double hover_f = move_group_->computeCartesianPath(hover_pts, 0.005, hover_traj, false);
        RCLCPP_INFO(get_logger(), "[PHASE 1] Hover Cartesian snap fraction for %s: %.2f", obj_name.c_str(), hover_f);

        if (hover_f > 0.80) {
            move_group_->execute(hover_traj);
        }
        rclcpp::sleep_for(std::chrono::milliseconds(600));

        double tx = 0.0, ty = 0.0, tz = 0.0;
        if (!getTrueToolPose(tx, ty, tz)) {
            RCLCPP_ERROR(get_logger(), "Cannot verify actual tool pose. Aborting pick.");
            return false;
        }

        const double dx = tx - hover.position.x;
        const double dy = ty - hover.position.y;
        const double dz = tz - hover.position.z;
        const double xy_error = std::hypot(dx, dy);

        if (xy_error > 0.03 || std::abs(dz) > 0.04) {
            RCLCPP_ERROR(get_logger(), "Tool pose mismatch. XY error=%.3f m, Z error=%.3f m", xy_error, dz);
            return false;
        }
        RCLCPP_INFO(get_logger(), "[CONFIRMED] Hover position verified! Dropping down to grasp...");

        // Phase 2: Straight Cartesian Drop
        geometry_msgs::msg::Pose grasp = hover;
        grasp.position.z = grasp_z;

        std::vector<geometry_msgs::msg::Pose> drop_pts{grasp};
        moveit_msgs::msg::RobotTrajectory drop_traj;
        double drop_f = move_group_->computeCartesianPath(drop_pts, 0.005, drop_traj, false);
        if (drop_f < 0.90 || move_group_->execute(drop_traj) != moveit::core::MoveItErrorCode::SUCCESS) {
            RCLCPP_ERROR(get_logger(), "Cartesian drop failed for %s", obj_name.c_str());
            return false;
        }

        // Phase 3: Close Gripper & Safe Lift
        gripper_group_->setNamedTarget("close");
        gripper_group_->move();
        rclcpp::sleep_for(std::chrono::milliseconds(1000));

        geometry_msgs::msg::Pose lift = grasp;
        lift.position.z += 0.07; 
        std::vector<geometry_msgs::msg::Pose> lift_pts{lift};
        moveit_msgs::msg::RobotTrajectory lift_traj;
        if (move_group_->computeCartesianPath(lift_pts, 0.005, lift_traj, false) < 0.90 ||
            move_group_->execute(lift_traj) != moveit::core::MoveItErrorCode::SUCCESS) {
            RCLCPP_ERROR(get_logger(), "Lift failed for %s", obj_name.c_str());
            return false;
        }

        // Phase 4: Arching "Mountain Outline" Transit to Tray Slot
        double y_offset = (slot_index * 0.08) - 0.04;
        geometry_msgs::msg::Pose tray_hover = lift;
        tray_hover.position.x = tray.position.x;
        tray_hover.position.y = tray.position.y + y_offset;
        tray_hover.position.z = tray.position.z + 0.06 + tcp_len;

        geometry_msgs::msg::Pose mountain_peak = lift;
        mountain_peak.position.x = (lift.position.x + tray_hover.position.x) / 2.0;
        mountain_peak.position.y = (lift.position.y + tray_hover.position.y) / 2.0;
        mountain_peak.position.z = 0.62; 

        std::vector<geometry_msgs::msg::Pose> transit_pts{mountain_peak, tray_hover};
        moveit_msgs::msg::RobotTrajectory transit_traj;
        double transit_f = move_group_->computeCartesianPath(transit_pts, 0.005, transit_traj, false);
        RCLCPP_INFO(get_logger(), "[PHASE 4] Mountain Arch Transit fraction: %.2f", transit_f);
        
        if (transit_f < 0.90 || move_group_->execute(transit_traj) != moveit::core::MoveItErrorCode::SUCCESS) {
            RCLCPP_ERROR(get_logger(), "Mountain transit failed for %s", obj_name.c_str());
            return false;
        }

        // Phase 5: Lower & Release
        geometry_msgs::msg::Pose tray_place = tray_hover;
        tray_place.position.z -= 0.03;
        std::vector<geometry_msgs::msg::Pose> place_pts{tray_place};
        moveit_msgs::msg::RobotTrajectory place_traj;
        if (move_group_->computeCartesianPath(place_pts, 0.005, place_traj, false) > 0.90) {
            move_group_->execute(place_traj);
        }

        gripper_group_->setNamedTarget("open");
        gripper_group_->move();
        rclcpp::sleep_for(std::chrono::milliseconds(500));
        RCLCPP_INFO(get_logger(), "[DROPPED] %s placed in Tray Slot #%d!", obj_name.c_str(), slot_index + 1);

        // Phase 6: Safe Short Retract
        geometry_msgs::msg::Pose high_retract = tray_hover;
        high_retract.position.z += 0.05;
        std::vector<geometry_msgs::msg::Pose> retract_pts{high_retract};
        moveit_msgs::msg::RobotTrajectory retract_traj;
        double retract_f = move_group_->computeCartesianPath(retract_pts, 0.005, retract_traj, false);
        RCLCPP_INFO(get_logger(), "[PHASE 6] Safe Retract fraction: %.2f", retract_f);
        if (retract_f < 0.90 || move_group_->execute(retract_traj) != moveit::core::MoveItErrorCode::SUCCESS) {
            RCLCPP_ERROR(get_logger(), "Retract failed for %s", obj_name.c_str());
            return false;
        }

        clearOctomap();
        clearOctomap();
        rclcpp::sleep_for(std::chrono::milliseconds(1500));
        return true;
    }

    void runPickAndPlaceLoop()
    {
        RCLCPP_INFO(get_logger(), "=== STARTING CLEAN STATE RESET ===");
        clearOctomap();
        goHome(); // Go home ONLY at the very beginning of the entire run
        rclcpp::sleep_for(std::chrono::milliseconds(2000));

        // 1. Pick and Place Red Box
        if (!executeSinglePickAndPlace("Red Box", 0)) {
            RCLCPP_ERROR(get_logger(), "Sequence stopped: failed to pick/place the Red Box.");
            return;
        }

        RCLCPP_INFO(get_logger(), "=== Red Box complete. Transitioning directly to Blue Cylinder... ===");
        rclcpp::sleep_for(std::chrono::seconds(1));

        // 2. Pick and Place Blue Cylinder (Starts directly from post-tray position, skipping goHome!)
        if (!executeSinglePickAndPlace("Blue Cylinder", 1)) {
            RCLCPP_ERROR(get_logger(), "Sequence stopped: failed to pick/place the Blue Cylinder.");
            return;
        }

        RCLCPP_INFO(get_logger(), "=== ALL OPERATIONS SUCCEEDED. Returning home. ===");
        goHome(); // Return home only after everything is fully done!
    }

    std::mutex data_mutex_;
    pcl_perception::msg::DetectedObjectArray latest_detected_objects_; // Moved here as class member!
    geometry_msgs::msg::Pose tray_pose_;
    bool have_tray_ = false;

    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    rclcpp::Subscription<pcl_perception::msg::DetectedObjectArray>::SharedPtr pose_sub_; // Updated type
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr tray_sub_;
    rclcpp::Client<std_srvs::srv::Empty>::SharedPtr clear_octomap_client_;
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> gripper_group_;
    std::thread worker_thread_;
    rclcpp::Time last_pose_received_{0, 0, RCL_ROS_TIME};
    bool have_pose_timestamp_ = false;

    geometry_msgs::msg::Quaternion downward_orientation_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PickPlannerNode>();

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);

    std::thread([node]() {
        rclcpp::sleep_for(std::chrono::seconds(2));
        node->initializeMoveIt();
    }).detach();

    executor.spin();
    rclcpp::shutdown();
    return 0;
}