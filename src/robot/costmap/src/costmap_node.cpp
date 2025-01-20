#include <chrono>
#include <memory>
 
#include "costmap_node.hpp"
 
CostmapNode::CostmapNode() : Node("costmap"), costmap_(robot::CostmapCore(this->get_logger())) {
  lidar_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "/lidar", 10, std::bind(&CostmapNode::laserCallback, this, std::placeholders::_1));
  costmap_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/costmap", 10);
  string_pub_ = this->create_publisher<std_msgs::msg::String>("/test_topic", 10);
  timer_ = this->create_wall_timer(std::chrono::milliseconds(500), std::bind(&CostmapNode::publishMessage, this));

  initializeCostmap();

  RCLCPP_INFO(this->get_logger(), "Costmap Node initialized.");
}

void CostmapNode::initializeCostmap() {
    costmap_msg_.header.frame_id = "map";
    costmap_msg_.info.resolution = 0.1; 
    costmap_msg_.info.width = 100;      
    costmap_msg_.info.height = 100;    
    costmap_msg_.info.origin.position.x = -5.0;  
    costmap_msg_.info.origin.position.y = -5.0;
    costmap_msg_.info.origin.orientation.w = 1.0;
    costmap_msg_.data.resize(costmap_msg_.info.width * costmap_msg_.info.height, 0);
}

void CostmapNode::laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    // Reset the costmap
    std::fill(costmap_msg_.data.begin(), costmap_msg_.data.end(), 0);

    for (size_t i = 0; i < msg->ranges.size(); ++i) {
        double angle = msg->angle_min + i * msg->angle_increment;
        double range = msg->ranges[i];

        if (range < msg->range_max && range > msg->range_min) {
            int x_grid, y_grid;
            convertToGrid(range, angle, x_grid, y_grid);
            markObstacle(x_grid, y_grid);
        }
    }

    inflateObstacles();
    publishCostmap();
}

void CostmapNode::convertToGrid(double range, double angle, int &x_grid, int &y_grid) {
    double x = range * cos(angle);
    double y = range * sin(angle);

    x_grid = static_cast<int>((x - costmap_msg_.info.origin.position.x) / costmap_msg_.info.resolution);
    y_grid = static_cast<int>((y - costmap_msg_.info.origin.position.y) / costmap_msg_.info.resolution);
}

void CostmapNode::markObstacle(int x_grid, int y_grid) {
    if (x_grid >= 0 && x_grid < static_cast<int>(costmap_msg_.info.width) &&
        y_grid >= 0 && y_grid < static_cast<int>(costmap_msg_.info.height)) {
        int index = y_grid * costmap_msg_.info.width + x_grid;
        costmap_msg_.data[index] = 100;  // Mark as obstacle
    }
}

void CostmapNode::inflateObstacles() {
    std::vector<int8_t> inflated_costmap = costmap_msg_.data;

    int inflation_radius = static_cast<int>(1.0 / costmap_msg_.info.resolution);  // 1 meter
    for (size_t i = 0; i < costmap_msg_.data.size(); ++i) {
        if (costmap_msg_.data[i] == 100) {
            int x = i % costmap_msg_.info.width;
            int y = i / costmap_msg_.info.width;

            for (int dx = -inflation_radius; dx <= inflation_radius; ++dx) {
                for (int dy = -inflation_radius; dy <= inflation_radius; ++dy) {
                    int nx = x + dx;
                    int ny = y + dy;
                    if (nx >= 0 && nx < static_cast<int>(costmap_msg_.info.width) &&
                        ny >= 0 && ny < static_cast<int>(costmap_msg_.info.height)) {
                        int index = ny * costmap_msg_.info.width + nx;
                        double distance = sqrt(dx * dx + dy * dy) * costmap_msg_.info.resolution;
                        if (distance <= 1.0) {
                            inflated_costmap[index] = std::max<int8_t>(
                                inflated_costmap[index], static_cast<int8_t>(100 * (1 - distance)));
                        }
                    }
                }
            }
        }
    }

    costmap_msg_.data = inflated_costmap;
}

void CostmapNode::publishCostmap() {
    costmap_msg_.header.stamp = this->now();
    costmap_pub_->publish(costmap_msg_);
}
 
// Define the timer to publish a message every 500ms
void CostmapNode::publishMessage() {
  auto message = std_msgs::msg::String();
  message.data = "Hello, ROS 2!";
  RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
  string_pub_->publish(message);
}
 
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CostmapNode>());
  rclcpp::shutdown();
  return 0;
}