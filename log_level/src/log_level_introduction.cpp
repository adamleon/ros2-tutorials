#include "rclcpp/rclcpp.hpp"

namespace log_level
{
class LogLevelNode : public rclcpp::Node
{
public:
    explicit LogLevelNode(const rclcpp::NodeOptions & options)
    : Node("log_level_introduction", options)
    {
        RCLCPP_DEBUG(this->get_logger(), "for details.");
        std::cin.get();

        RCLCPP_INFO(this->get_logger(), "for general info.");
        std::cin.get();

        RCLCPP_WARN(this->get_logger(), "for unexpected behavior.");
        std::cin.get();

        RCLCPP_ERROR(this->get_logger(), "for when problems occur.");
        std::cin.get();

        RCLCPP_FATAL(this->get_logger(), "for when the node fails.");
        std::cin.get();
    }
};
}

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<log_level::LogLevelNode>(rclcpp::NodeOptions()));
    rclcpp::shutdown();
    return 0;
}