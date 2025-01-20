#include <chrono>
#include <memory>
#include <string>
#include "costmap_node.hpp"
const int GRID = 100;

CostmapNode::CostmapNode() : Node("costmap"), costmap_(robot::CostmapCore(this->get_logger())) {
  // Subscribe to /lidar
  auto lidar_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
    "/lidar", 10, std::bind(&CostmapNode::laserCallback, this, std::placeholders::_1));
  
  // Publish to /costmap
  string_pub_ = this->create_publisher<std_msgs::msg::String>("/costmap", 10);

  sub_grid_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "/odom/filtered", 10, std::bind(&CostmapNode::odomCallback, this, std::placeholders::_1));
  pub_grid_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/costmap", 10);
  // timer_ = this->create_wall_timer(std::chrono::milliseconds(500), std::bind(&CostmapNode::publishMessage, this));
}

void CostmapNode::laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan) {
  RCLCPP_INFO(this->get_logger(), "Received a message from /lidar");
  
  // Step 1: Initialize costmap
  // initializeCostmap();

  // Step 2: Convert LaserScan to grid and mark obstacles
  // for (size_t i = 0; i < scan->ranges.size(); ++i) {
  //     double angle = scan->angle_min + i * scan->angle_increment;
  //     double range = scan->ranges[i];
  //     if (range < scan->range_max && range > scan->range_min) {
  //         // Calculate grid coordinates
  //         int x_grid, y_grid;
  //         convertToGrid(range, angle, x_grid, y_grid);
  //         markObstacle(x_grid, y_grid);
  //     }
  // }

  // Step 3: Inflate obstacles
  // inflateObstacles();

  // Step 4: Publish costmap
  // publishCostmap();
}
 
// Define the timer to publish a message every 500ms
void CostmapNode::publishMessage() {
  auto message = std_msgs::msg::String();
  message.data = "Hello, ROS 2!";
  RCLCPP_INFO(this->get_logger(), "TESTING: '%s'", message.data.c_str());
  string_pub_->publish(message);
}
 
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CostmapNode>());
  rclcpp::shutdown();
  return 0;
}