#include "rclcpp/rclcpp.hpp"

namespace log_level
{
class LogLevelNode : public rclcpp::Node
{
public:
    explicit LogLevelNode(const rclcpp::NodeOptions & options)
    : Node("log_level_introduction", options)
    {
        RCLCPP_DEBUG(this->get_logger(), "DEBUG: Used for detailed debugging information.");
        std::cin.get();

        RCLCPP_INFO(this->get_logger(), "INFO: Used for general information about node operation.");
        std::cin.get();

        RCLCPP_WARN(this->get_logger(), "WARN: Used when something unexpected happened, but the node can continue.");
        std::cin.get();

        RCLCPP_ERROR(this->get_logger(), "ERROR: Used when a significant problem occurred, but the node is still running.");
        std::cin.get();

        RCLCPP_FATAL(this->get_logger(), "FATAL: Used when a critical error occurred and the node may not be able to continue.");
        std::cin.get();
    }
};
}

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(log_level::LogLevelNode)