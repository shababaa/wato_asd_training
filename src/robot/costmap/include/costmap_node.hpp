#ifndef COSTMAP_NODE_HPP_
#define COSTMAP_NODE_HPP_
 
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "costmap_core.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"  // For LaserScan message type
#include "nav_msgs/msg/occupancy_grid.hpp"  // For OccupancyGrid message type
 
class CostmapNode : public rclcpp::Node {
  public:
    CostmapNode();
    
    // Place callback function here
    void publishMessage();

    void initializeCostmap();
    void laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
    void convertToGrid(double range, double angle, int &x_grid, int &y_grid);  // Declare convertToGrid
    void markObstacle(int x_grid, int y_grid);  // Declare markObstacle
    void inflateObstacles();
    void publishCostmap();

  private:
    robot::CostmapCore costmap_;
    // Place these constructs here
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr string_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar_sub_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_pub_;
    nav_msgs::msg::OccupancyGrid costmap_msg_;
};
 
#endif