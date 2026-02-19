#include "rclcpp/rclcpp.hpp"
#include "ros2_custom_msgs/msg/robot_status.hpp"

using namespace std::chrono_literals;

class StatusPublisher : public rclcpp::Node
{
public:
    StatusPublisher() : Node("status_publisher"), battery_(100.0), missions_(0)
    {
        publisher_ = this->create_publisher<ros2_custom_msgs::msg::RobotStatus>(
            "robot_status", 10);

        timer_ = this->create_wall_timer(
            1s, std::bind(&StatusPublisher::publish_status, this));
    }

private:
    void publish_status()
    {
        auto message = ros2_custom_msgs::msg::RobotStatus();
        message.robot_name = "Explorer1";
        message.battery_level = battery_;
        message.is_active = true;
        message.mission_count = missions_;

        RCLCPP_INFO(this->get_logger(),
                    "Publishing: robot=%s, battery=%.1f, active=%s, missions=%d",
                    message.robot_name.c_str(),
                    message.battery_level,
                    message.is_active ? "true" : "false",
                    message.mission_count);

        publisher_->publish(message);

        battery_ -= 0.5;
        missions_++;
    }

    rclcpp::Publisher<ros2_custom_msgs::msg::RobotStatus>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    float battery_;
    int missions_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<StatusPublisher>());
    rclcpp::shutdown();
    return 0;
}

