#include "ros/ros.h"
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>

typedef struct {
    float x;
    float y;
    float yaw;
} RobotPosition;

typedef struct {
    float x;
    float y;
} TargetPosition;

void show_robot_position(const RobotPosition& robot) {
    std::cout << "The robot position is: ";
    std::cout << "[" << robot.x << ", ";
    std::cout << robot.y << ", ";
    std::cout << robot.yaw << "]" << std::endl;
}

void odometry_callback(const nav_msgs::Odometry::ConstPtr& msg) {
    RobotPosition robot_position;
    robot_position.x = msg->pose.pose.position.x;
    robot_position.y = msg->pose.pose.position.y;
    robot_position.yaw = 2*asin(msg->pose.pose.orientation.z);
}

void laser_callback(const sensor_msgs::LaserScan::ConstPtr& msg) {
    std::vector<float> scan = msg->ranges;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "stage_controller_node");
    ros::NodeHandle node;

    ros::Publisher cmd_publisher = node.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
    ros::Subscriber odometry_sub = node.subscribe("/base_pose_ground_truth", 1, odometry_callback);
    ros::Subscriber laserscan_sub = node.subscribe("/base_scan", 1, laser_callback);

    ros::spin();

    return 0;
}