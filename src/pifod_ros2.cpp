#include "pifod_ros2/pifod_ros2.hpp"

namespace pifod_ros2
{
    PIFOD_ROS2::PIFOD_ROS2(const rclcpp::NodeOptions& option) : Node("PIFOD_ROS2", option)
    {
        pointcloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/pointcloud",
            rclcpp::SystemDefaultsQoS(),
            std::bind(&PIFOD_ROS2::pointcloud_callback, this, _1)
        );

        timer_ = this->create_wall_timer(1ms, std::bind(&PIFOD_ROS2::timer_callback, this));

        pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/pose", rclcpp::SystemDefaultsQoS());
        path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/path", rclcpp::SystemDefaultsQoS());

        icp_max_iter_ = this->declare_parameter("icp_max_iter", 30);
        icp_tolerance_ = this->declare_parameter("icp_tolerance", 0.001);
        icp_setTargetFlag_ = false;

        icp_ = std::make_shared<ICPOdometer>(icp_max_iter_, icp_tolerance_);

        icp_odom_ = geometry_msgs::msg::PoseStamped();
        icp_odom_.header.frame_id = "camera_link";
        path_ = nav_msgs::msg::Path();
        path_.header.frame_id = "camera_link";

        RCLCPP_INFO(this->get_logger(), "Start PointCloud IMU Fusion Odometry");
    }

    void PIFOD_ROS2::pointcloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        if(!icp_setTargetFlag_)
        {
            icp_->setTarget(msg);
            icp_setTargetFlag_ = true;
            RCLCPP_INFO(this->get_logger(), "Set Target PointCloud");
            
            return;
        }

        icp_->compute(msg);

        const auto t = icp_->getTranslation();
        const auto q = icp_->getPosture();

        icp_odom_.pose.position.x = t.z();
        icp_odom_.pose.position.y = t.y();
        icp_odom_.pose.position.z = 0.0;
        RCLCPP_INFO(this->get_logger(), "x:%lf, y:%lf, z:%lf", t.x(), t.y(), t.z());
        icp_odom_.pose.orientation.w = q.w();
        icp_odom_.pose.orientation.x = q.x();
        icp_odom_.pose.orientation.y = q.y();
        icp_odom_.pose.orientation.z = q.z();
    }

    void PIFOD_ROS2::timer_callback()
    {
        path_.poses.push_back(icp_odom_);

        pose_pub_->publish(icp_odom_);
        path_pub_->publish(path_);
    }
}

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(pifod_ros2::PIFOD_ROS2)