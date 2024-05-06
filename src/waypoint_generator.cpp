#include "waypoint_generator.hpp"

#include <cstdlib>
#include <fstream>
#include <iostream>
#include <memory>
#include <string>
#include <chrono>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono;

WaypointGenerator::WaypointGenerator() : Node("waypoint_generator_node") {
    this->declare_parameter("odom_topic", "/ego_racecar/odom");
    this->declare_parameter("min_distance", 0.01);
    this->declare_parameter("save_path", "/home/onebean/sim_ws/traj/waypoints_odom.csv");

    odom_topic = this->get_parameter("odom_topic").as_string();
    min_distance = this->get_parameter("min_distance").as_double();
    save_path = this->get_parameter("save_path").as_string();
    start_time = steady_clock::now(); // 초기 시간 설정

    subscription_odom = this->create_subscription<nav_msgs::msg::Odometry>(
        odom_topic, 1000, std::bind(&WaypointGenerator::odom_callback, this, std::placeholders::_1));
}

void WaypointGenerator::odom_callback(const nav_msgs::msg::Odometry::ConstSharedPtr odom_submsgObj) {
    csv_odom.open(save_path, std::ios::out | std::ios::app);

    auto now = steady_clock::now();
    auto elapsed_time = duration<double>(now - start_time).count(); // 시간차이 계산

    double x = odom_submsgObj->pose.pose.position.x;
    double y = odom_submsgObj->pose.pose.position.y;
    double diff = sqrt(pow((x - x_old), 2) + pow((y - y_old), 2));

    if (diff > min_distance) {
        csv_odom << elapsed_time << ", " << x << ", " << y << "\n"; // 시간, x, y 저장
        RCLCPP_INFO(this->get_logger(), "Time: %f, X: %f, Y: %f", elapsed_time, x, y);

        x_old = x;
        y_old = y;
    }

    csv_odom.close();
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node_ptr = std::make_shared<WaypointGenerator>();
    rclcpp::spin(node_ptr);
    rclcpp::shutdown();
    return 0;
}

