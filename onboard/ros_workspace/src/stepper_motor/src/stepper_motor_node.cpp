#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>
#include "stepper_motor_controller.h"

class StepperMotorNode : public rclcpp::Node {
public:
    StepperMotorNode() : Node("stepper_motor_node") {
        // Declare parameters
        this->declare_parameter<std::string>("serial_port", "/dev/esp32");
        this->declare_parameter<int>("baudrate", 115200);

        // Get parameters
        std::string serial_port = this->get_parameter("serial_port").as_string();
        int baudrate = this->get_parameter("baudrate").as_int();

        // Initialize controller
        controller_ = std::make_shared<StepperMotorController>();
        
        if (!controller_->init(serial_port, baudrate)) {
            RCLCPP_ERROR(this->get_logger(), "Failed to initialize stepper motor controller");
            throw std::runtime_error("Controller initialization failed");
        }

        // Create subscriber for motor commands (steps:direction format)
        command_subscriber_ = this->create_subscription<std_msgs::msg::String>(
            "motor_command", 10, 
            std::bind(&StepperMotorNode::motorCommandCallback, this, std::placeholders::_1));

        // Create publishers for feedback
        position_publisher_ = this->create_publisher<std_msgs::msg::Int32>(
            "current_position", 10);
        
        status_publisher_ = this->create_publisher<std_msgs::msg::Bool>(
            "motor_moving", 10);

        // Create timer for publishing feedback
        feedback_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&StepperMotorNode::publishFeedback, this));

        RCLCPP_INFO(this->get_logger(), "Stepper Motor Node Initialized");
        RCLCPP_INFO(this->get_logger(), "Serial Port: %s, Baudrate: %d", 
                    serial_port.c_str(), baudrate);
    }

private:
    void motorCommandCallback(const std_msgs::msg::String::SharedPtr msg) {
        // Expected format: "direction steps\r" e.g., "p 10\r" or "n 10\r"
        char direction;
        int steps;
        
        if (sscanf(msg->data.c_str(), "%c %d", &direction, &steps) == 2) {
            controller_->moveSteps(steps, direction);
            RCLCPP_INFO(this->get_logger(), "Moving %d steps in direction '%c'", steps, direction);
        } else if (msg->data.find("stop") != std::string::npos) {
            controller_->stop();
            RCLCPP_INFO(this->get_logger(), "Motor stopped");
        } else {
            RCLCPP_WARN(this->get_logger(), "Invalid command format. Use 'direction steps\\r' or 'stop\\r'");
        }
    }

    void publishFeedback() {
        auto position_msg = std_msgs::msg::Int32();
        position_msg.data = controller_->getPosition();
        position_publisher_->publish(position_msg);

        auto status_msg = std_msgs::msg::Bool();
        status_msg.data = controller_->isMoving();
        status_publisher_->publish(status_msg);
    }

    std::shared_ptr<StepperMotorController> controller_;
    
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr command_subscriber_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr position_publisher_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr status_publisher_;
    
    rclcpp::TimerBase::SharedPtr feedback_timer_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    try {
        rclcpp::spin(std::make_shared<StepperMotorNode>());
    } catch (const std::exception& e) {
        RCLCPP_FATAL(rclcpp::get_logger("rclcpp"), "Fatal error: %s", e.what());
    }
    rclcpp::shutdown();
    return 0;
}