#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/bool.hpp>
#include <cmath>
#include <algorithm>

class ObstacleDetector : public rclcpp::Node
{
public:
    ObstacleDetector() : Node("obstacle_detector")
    {
        // Declare parameters
        this->declare_parameter("safety_distance", 0.5);  // meters
        this->declare_parameter("stop_distance", 0.3);     // meters
        this->declare_parameter("sector_angle", 60.0);     // degrees (front sector)
        
        // Get parameters
        safety_distance_ = this->get_parameter("safety_distance").as_double();
        stop_distance_ = this->get_parameter("stop_distance").as_double();
        sector_angle_ = this->get_parameter("sector_angle").as_double();
        
        // Subscribers
        scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10,
            std::bind(&ObstacleDetector::scanCallback, this, std::placeholders::_1));
        
        // Publishers
        cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
        obstacle_detected_pub_ = this->create_publisher<std_msgs::msg::Bool>("/obstacle_detected", 10);
        
        RCLCPP_INFO(this->get_logger(), "Obstacle Detector Node Started!");
        RCLCPP_INFO(this->get_logger(), "Safety distance: %.2f m", safety_distance_);
        RCLCPP_INFO(this->get_logger(), "Stop distance: %.2f m", stop_distance_);
        RCLCPP_INFO(this->get_logger(), "Monitoring front sector: +/- %.1f degrees", sector_angle_ / 2.0);
    }

private:
    void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        // Find minimum distance in front sector
        double min_distance = std::numeric_limits<double>::infinity();
        int min_index = -1;
        
        // Calculate front sector indices
        int total_points = msg->ranges.size();
        double angle_increment = msg->angle_increment;
        double sector_rad = sector_angle_ * M_PI / 180.0;
        
        // Calculate how many points to check in front sector
        int sector_points = static_cast<int>(sector_rad / angle_increment);
        int start_index = (total_points / 2) - (sector_points / 2);
        int end_index = (total_points / 2) + (sector_points / 2);
        
        // Ensure indices are within bounds
        start_index = std::max(0, start_index);
        end_index = std::min(total_points - 1, end_index);
        
        // Find minimum distance in front sector
        for (int i = start_index; i <= end_index; i++)
        {
            float range = msg->ranges[i];
            
            // Filter out invalid readings (inf, nan, 0)
            if (std::isfinite(range) && range > msg->range_min && range < msg->range_max)
            {
                if (range < min_distance)
                {
                    min_distance = range;
                    min_index = i;
                }
            }
        }
        
        // Publish obstacle detection status
        std_msgs::msg::Bool obstacle_msg;
        obstacle_msg.data = (min_distance < safety_distance_);
        obstacle_detected_pub_->publish(obstacle_msg);
        
        // React based on distance
        // React based on distance
        if (min_distance < stop_distance_)
        {
            // STOP! Obstacle too close - publish continuously to override other commands
            auto stop_cmd = geometry_msgs::msg::Twist();
            stop_cmd.linear.x = 0.0;
            stop_cmd.angular.z = 0.0;
            
            // Publish multiple times to ensure it overrides teleop
            for(int i = 0; i < 5; i++) {
                cmd_vel_pub_->publish(stop_cmd);
            }
            
            if (frame_count_ % 10 == 0) {  // Don't spam console
                RCLCPP_WARN(this->get_logger(), 
                    "⚠️  OBSTACLE TOO CLOSE! Distance: %.2f m - STOPPING", min_distance);
            }
        }
        else if (min_distance < safety_distance_)
        {
            // Warning zone - obstacle detected but not critical
            double angle = msg->angle_min + (min_index * angle_increment);
            double angle_deg = angle * 180.0 / M_PI;
            
            RCLCPP_INFO(this->get_logger(), 
                "⚠️  Obstacle detected at %.2f m, angle: %.1f°", 
                min_distance, angle_deg);
        }
        else
        {
            // All clear
            if (frame_count_ % 50 == 0)  // Print every 50 frames to avoid spam
            {
                RCLCPP_INFO(this->get_logger(), "✓ Path clear - Min distance: %.2f m", min_distance);
            }
        }
        
        frame_count_++;
    }
    
    // Subscribers
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    
    // Publishers
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr obstacle_detected_pub_;
    
    // Parameters
    double safety_distance_;
    double stop_distance_;
    double sector_angle_;
    
    // State
    int frame_count_ = 0;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ObstacleDetector>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
