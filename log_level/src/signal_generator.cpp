#include <rclcpp/rclcpp.hpp>
#include <log_level/msg/float64_stamped.hpp>
#include <cmath>
#include "log_level/msg/float64_stamped.hpp"

namespace log_level
{
    
    /**
    * @brief A simple ROS 2 node that publishes a constant setpoint and a sine wave signal.
    *
    * The setpoint is always 1, and the signal is a sine wave that varies over time.
    */
    class SignalGenerator : public rclcpp::Node
    {
        public:
        explicit SignalGenerator(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
        : Node("signal_generator", options)
        {
            // Initialize publishers for setpoint and signal
            setpoint_pub_ = this->create_publisher<log_level::msg::Float64Stamped>("setpoint", 10);
            signal_pub_ = this->create_publisher<log_level::msg::Float64Stamped>("signal", 10);
            
            // Create a timer to call the timer_callback at regular intervals
            timer_ = this->create_wall_timer(
                std::chrono::milliseconds(100),
                std::bind(&SignalGenerator::timer_callback, this));
            }
            
            private:
            void timer_callback()
            {
                auto now = this->now();
                
                // Setpoint message (always 1)
                log_level::msg::Float64Stamped setpoint_msg;
                setpoint_msg.header.stamp = now;
                setpoint_msg.data = 1.0;
                setpoint_pub_->publish(setpoint_msg);
                
                // Signal message (sine wave)
                log_level::msg::Float64Stamped signal_msg;
                signal_msg.header.stamp = now;
                signal_msg.data = std::sin(phase_);
                signal_pub_->publish(signal_msg);
                
                phase_ += 0.1; // Increment phase for sine wave
            }
            
            rclcpp::Publisher<log_level::msg::Float64Stamped>::SharedPtr setpoint_pub_;
            rclcpp::Publisher<log_level::msg::Float64Stamped>::SharedPtr signal_pub_;
            rclcpp::TimerBase::SharedPtr timer_;
            double phase_ = 0.0;
        };
        
    } // namespace log_level
    
    #include <rclcpp_components/register_node_macro.hpp>
    RCLCPP_COMPONENTS_REGISTER_NODE(log_level::SignalGenerator)