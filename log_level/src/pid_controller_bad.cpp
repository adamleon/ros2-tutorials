#include "rclcpp/rclcpp.hpp"
#include "rcutils/error_handling.h"
#include "log_level/msg/float64_stamped.hpp"

namespace log_level
{
namespace bad
{

/**
 * @brief A simple PID controller node that subscribes to a signal and a setpoint,
 * computes the PID output, and publishes it.
 *
 * The node listens for messages on the "signal" and "setpoint" topics, computes the PID
 * control output based on the received values, and publishes the result on the "output" topic.
 * 
 * This implemmentation shows how to utilize log levels effectively in a ROS 2 node.
 */
class PIDController : public rclcpp::Node
{
public:
    explicit PIDController(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
    : Node("pid_controller", options),
      kp_(1.0), ki_(0.5), kd_(0.0), anti_windup_limit_(10.0),
      prev_error_(0.0), integral_(0.0), prev_time_sec_(0.0),
      setpoint_(0.0)
    {
        std::cout << "Initializing PID Controller Node" << std::endl;
        // Initialize subscriptions and publisher
        sub_signal_ = this->create_subscription<log_level::msg::Float64Stamped>(
            "signal", 10,
            std::bind(&PIDController::pid_updater, this, std::placeholders::_1));
        std::cout << "Subscribed to 'signal' topic" << std::endl;
        
        sub_setpoint_ = this->create_subscription<log_level::msg::Float64Stamped>(
            "setpoint", 10,
            std::bind(&PIDController::setpoint_callback, this, std::placeholders::_1));
        std::cout << "Subscribed to 'setpoint' topic" << std::endl;
        
        pub_ = this->create_publisher<log_level::msg::Float64Stamped>("output", 10);
        std::cout << "Publisher for 'output' topic created" << std::endl;

        // Log the initialization
        std::cout << "PID Controller Node initialized with parameters:" << std::endl;
        std::cout << "  Kp: " << kp_ << ", Ki: " << ki_ << ", Kd: " << kd_ << std::endl;
    }

private:
    /**
     * @brief Callback function for the PID controller.
     * Computes the PID control output based on the received signal and setpoint.
     *
     * @param msg The received Float64Stamped message containing the signal value.
     */
    void pid_updater(const log_level::msg::Float64Stamped::SharedPtr msg)
    {
        if (!signal_received_) {
            std::cout << "PID updater called for the first time" << std::endl;
        }
        std::cout << "Received signal: " << msg->data << std::endl;

        // check if setpoint has been received
        if (!setpoint_received_) {
            std::cout << "Setpoint not received yet, ignoring signal." << std::endl;
            return; // Exit if setpoint has not been received
        }

        double error = setpoint_ - msg->data;

        double now_sec = rclcpp::Time(msg->header.stamp).seconds();
        double dt = now_sec - prev_time_sec_;

        if (dt <= 0.0) {
            std::cout << "Ignoring message with timestamp earlier than the last processed message." << std::endl;
            return; // Ignore messages with timestamps earlier than the last processed message
        }

        if (!signal_received_) {
            std::cout << "First signal received, initializing previous error." << std::endl;
            prev_error_ = error; // Initialize previous error on first signal
        }

        double proportional_gain = kp_ * error;
        integral_ += error * dt;
        double integral_gain = ki_ * std::clamp(integral_, -anti_windup_limit_, anti_windup_limit_); // Clamp integral to prevent windup
        if(integral_ > anti_windup_limit_ || integral_ < -anti_windup_limit_) {
            std::cout << "Integral term exceeded anti-windup limit: " << integral_ << std::endl;
        }
        double derivative_gain = kd_ * (error - prev_error_) / dt;
        double output = proportional_gain + integral_gain + derivative_gain;

        std::cout << "PID computation: P=" << proportional_gain 
                  << ", I=" << integral_gain 
                  << ", D=" << derivative_gain 
                  << ", Output=" << output << std::endl;
        
        // Publish the PID output
        auto output_msg = log_level::msg::Float64Stamped();
        output_msg.header = msg->header;
        output_msg.header.stamp = this->get_clock()->now();
        output_msg.header.frame_id = "pid_output";
        output_msg.data = output;
        pub_->publish(output_msg);

        prev_error_ = error;
        prev_time_sec_ = now_sec;

        signal_received_ = true; // Set the flag to indicate signal has been received
    }

    /**
     * @brief Callback function for the setpoint subscription.
     * Updates the setpoint value and logs the received value.
     *
     * @param msg The received Float64Stamped message containing the setpoint value.
     */
    void setpoint_callback(const log_level::msg::Float64Stamped::SharedPtr msg)
    {   
        std::cout << "Received setpoint: " << msg->data << std::endl;
        
        // Check if the setpoint is valid (not NaN or Inf)
        if (std::isnan(msg->data) || std::isinf(msg->data)) {
            std::cout << "Invalid setpoint received, ignoring." << std::endl;
            return; // Ignore invalid setpoints
        }

        setpoint_ = msg->data;
        setpoint_received_ = true; // Set the flag to indicate setpoint has been received
    }

    // PID parameters
    double kp_, ki_, kd_;
    double anti_windup_limit_; // Limit for integral term to prevent windup
    double prev_error_, integral_, prev_time_sec_;
    double setpoint_;

    bool signal_received_ = false; // Flag to check if signal has been received
    bool setpoint_received_ = false; // Flag to check if setpoint has been received

    // Subscriptions and publisher
    rclcpp::Subscription<log_level::msg::Float64Stamped>::SharedPtr sub_signal_;
    rclcpp::Subscription<log_level::msg::Float64Stamped>::SharedPtr sub_setpoint_;
    rclcpp::Publisher<log_level::msg::Float64Stamped>::SharedPtr pub_;
};

} // namespace bad
} // namespace log_level

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(log_level::bad::PIDController)
