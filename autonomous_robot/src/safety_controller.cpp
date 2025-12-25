#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/bool.hpp>

class SafetyController : public rclcpp::Node
{
public:
    SafetyController() : Node("safety_controller")
    {
        // Parameters
        this->declare_parameter("stop_distance", 0.4);
        this->declare_parameter("slow_distance", 0.8);
        this->declare_parameter("sector_angle", 80.0);
        
        stop_distance_ = this->get_parameter("stop_distance").as_double();
        slow_distance_ = this->get_parameter("slow_distance").as_double();
        sector_angle_ = this->get_parameter("sector_angle").as_double();
        
        // Subscribe to teleop commands (input)
        cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel", 10,
            std::bind(&SafetyController::cmdVelCallback, this, std::placeholders::_1));
        
        // Subscribe to lidar
        scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10,
            std::bind(&SafetyController::scanCallback, this, std::placeholders::_1));
        
        // Publish safe commands (output to robot)
        cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
        
        // Publish obstacle status
        obstacle_pub_ = this->create_publisher<std_msgs::msg::Bool>("/obstacle_detected", 10);
        
        RCLCPP_INFO(this->get_logger(), "🛡️  Safety Controller Started!");
        RCLCPP_INFO(this->get_logger(), "Stop distance: %.2f m", stop_distance_);
        RCLCPP_INFO(this->get_logger(), "Slow distance: %.2f m", slow_distance_);
        // Timer to continuously monitor and enforce safety
        safety_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(20),  // 50 Hz
            std::bind(&SafetyController::safetyCheckTimer, this));
        
        RCLCPP_INFO(this->get_logger(), "🛡️  Safety monitoring at 50 Hz");
    }

private:
    void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        // Find minimum distance in front sector
        min_front_distance_ = std::numeric_limits<double>::infinity();
        
        int total_points = msg->ranges.size();
        double angle_increment = msg->angle_increment;
        double sector_rad = sector_angle_ * M_PI / 180.0;
        
        int sector_points = static_cast<int>(sector_rad / angle_increment);
        int start_index = (total_points / 2) - (sector_points / 2);
        int end_index = (total_points / 2) + (sector_points / 2);
        
        start_index = std::max(0, start_index);
        end_index = std::min(total_points - 1, end_index);
        
        for (int i = start_index; i <= end_index; i++)
        {
            float range = msg->ranges[i];
            if (std::isfinite(range) && range > msg->range_min && range < msg->range_max)
            {
                min_front_distance_ = std::min(min_front_distance_, static_cast<double>(range));
            }
        }
        
        // Publish obstacle status
        std_msgs::msg::Bool obstacle_msg;
        obstacle_msg.data = (min_front_distance_ < slow_distance_);
        obstacle_pub_->publish(obstacle_msg);
    }
    
    /*void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        auto safe_cmd = geometry_msgs::msg::Twist();
        
        // Check if we're trying to move forward
        bool moving_forward = (msg->linear.x > 0.01);
        
        if (moving_forward && min_front_distance_ < stop_distance_)
        {
            // STOP - obstacle too close and trying to move forward
            safe_cmd.linear.x = 0.0;
            safe_cmd.angular.z = msg->angular.z;  // Allow rotation
            
            if (warning_count_ % 20 == 0) {
                RCLCPP_WARN(this->get_logger(), 
                    "🛑 BLOCKED! Obstacle at %.2fm - Stopping forward motion", 
                    min_front_distance_);
            }
            warning_count_++;
        }
        else if (moving_forward && min_front_distance_ < slow_distance_)
        {
            // SLOW DOWN - obstacle ahead but not critical
            double scale = (min_front_distance_ - stop_distance_) / (slow_distance_ - stop_distance_);
            scale = std::max(0.3, std::min(1.0, scale));  // 30-100% speed
            
            safe_cmd.linear.x = msg->linear.x * scale;
            safe_cmd.linear.y = msg->linear.y * scale;
            safe_cmd.angular.z = msg->angular.z;
            
            if (warning_count_ % 20 == 0) {
                RCLCPP_INFO(this->get_logger(), 
                    "⚠️  Slowing down (%.0f%%) - Obstacle at %.2fm", 
                    scale * 100, min_front_distance_);
            }
            warning_count_++;
        }
        else
        {
            // ALL CLEAR - pass through command unchanged
            safe_cmd = *msg;
            warning_count_ = 0;
        }
        
        // Publish safe command
        cmd_vel_pub_->publish(safe_cmd);
    }*/
    void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        auto safe_cmd = geometry_msgs::msg::Twist();
        
        // Track if trying to move forward
        last_cmd_was_forward_ = (msg->linear.x > 0.01);
        last_angular_z_ = msg->angular.z;
        
        if (last_cmd_was_forward_ && min_front_distance_ < stop_distance_)
        {
            // HARD STOP - obstacle too close
            safe_cmd.linear.x = 0.0;
            safe_cmd.linear.y = 0.0;
            safe_cmd.linear.z = 0.0;
            safe_cmd.angular.z = msg->angular.z;  // Allow rotation
            
            for(int i = 0; i < 3; i++){
                cmd_vel_pub_->publish(safe_cmd);
            }

            if (warning_count_ % 20 == 0) {
                RCLCPP_WARN(this->get_logger(), 
                    "🛑 BLOCKED! Obstacle at %.2fm - Stopping forward motion", 
                    min_front_distance_);
            }
            warning_count_++;
        }
        else if (last_cmd_was_forward_ && min_front_distance_ < slow_distance_)
        {
            // SLOW DOWN - obstacle ahead
            double scale = (min_front_distance_ - stop_distance_) / (slow_distance_ - stop_distance_);
            scale = std::max(0.2, std::min(1.0, scale));
            
            safe_cmd.linear.x = msg->linear.x * scale;
            safe_cmd.linear.y = msg->linear.y * scale;
            safe_cmd.angular.z = msg->angular.z;
            
            if (warning_count_ % 20 == 0) {
                RCLCPP_INFO(this->get_logger(), 
                    "⚠️  Slowing to %.0f%% - Obstacle at %.2fm", 
                    scale * 100, min_front_distance_);
            }
            warning_count_++;
        }
        else
        {
            // PASS THROUGH - all clear or moving backward/sideways
            safe_cmd = *msg;
            warning_count_ = 0;
        }
        
        cmd_vel_pub_->publish(safe_cmd);
    }

    void safetyCheckTimer()
    {
        // If obstacle is close, continuously publish stop command
        if (min_front_distance_ < stop_distance_ && last_cmd_was_forward_)
        {
            auto stop_cmd = geometry_msgs::msg::Twist();
            stop_cmd.linear.x = 0.0;
            stop_cmd.linear.y = 0.0;
            stop_cmd.linear.z = 0.0;
            stop_cmd.angular.x = 0.0;
            stop_cmd.angular.y = 0.0;
            stop_cmd.angular.z = last_angular_z_;  // Keep last rotation
            
            for(int i = 0; i < 3; i++){
                cmd_vel_pub_->publish(stop_cmd);
            }

            if (warning_count_ % 10 == 0) {
                RCLCPP_WARN(this->get_logger(), 
                    "🛑 ENFORCING STOP - Obstacle at %.2fm", min_front_distance_);
            }
            warning_count_++;
        }
    }
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr obstacle_pub_;
    
    double stop_distance_;
    double slow_distance_;
    double sector_angle_;
    double min_front_distance_ = std::numeric_limits<double>::infinity();
    int warning_count_ = 0;
    rclcpp::TimerBase::SharedPtr safety_timer_;
    bool last_cmd_was_forward_ = false;
    double last_angular_z_ = 0.0;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SafetyController>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
