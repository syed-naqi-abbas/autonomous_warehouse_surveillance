#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <termios.h>
#include <unistd.h>
#include <iostream>
#include <sstream>

class StepperMotorTeleop : public rclcpp::Node {
public:
    StepperMotorTeleop() : Node("stepper_motor_teleop") {
        command_publisher_ = this->create_publisher<std_msgs::msg::String>("motor_command", 10);
        
        setupTerminal();
        
        RCLCPP_INFO(this->get_logger(), "Stepper Motor Teleop Started");
        RCLCPP_INFO(this->get_logger(), "Controls:");
        RCLCPP_INFO(this->get_logger(), "  P: Move 10 steps in +ve direction");
        RCLCPP_INFO(this->get_logger(), "  N: Move 10 steps in -ve direction");
        RCLCPP_INFO(this->get_logger(), "  S: Stop motor");
        RCLCPP_INFO(this->get_logger(), "  Q: Quit");
        
        input_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&StepperMotorTeleop::readKeyboard, this));
    }

    ~StepperMotorTeleop() {
        restoreTerminal();
    }

private:
    struct TerminalSettings {
        termios original;
        termios raw;
    } terminal_settings_;

    void setupTerminal() {
        tcgetattr(STDIN_FILENO, &terminal_settings_.original);
        terminal_settings_.raw = terminal_settings_.original;
        terminal_settings_.raw.c_lflag &= ~(ICANON | ECHO);
        terminal_settings_.raw.c_cc[VMIN] = 0;
        terminal_settings_.raw.c_cc[VTIME] = 0;
        tcsetattr(STDIN_FILENO, TCSANOW, &terminal_settings_.raw);
    }

    void restoreTerminal() {
        tcsetattr(STDIN_FILENO, TCSANOW, &terminal_settings_.original);
    }

    void readKeyboard() {
        char key;
        if (read(STDIN_FILENO, &key, 1) == 1) {
            auto command_msg = std_msgs::msg::String();
            
            switch (key) {
                case 'p':
                case 'P':
                    command_msg.data = "p 10\r";
                    command_publisher_->publish(command_msg);
                    RCLCPP_INFO(this->get_logger(), "Moving 10 steps in +ve direction");
                    break;
                    
                case 'n':
                case 'N':
                    command_msg.data = "n 10\r";
                    command_publisher_->publish(command_msg);
                    RCLCPP_INFO(this->get_logger(), "Moving 10 steps in -ve direction");
                    break;
                    
                case 's':
                case 'S':
                    command_msg.data = "stop\r";
                    command_publisher_->publish(command_msg);
                    RCLCPP_INFO(this->get_logger(), "Motor stopped");
                    break;
                    
                case 'q':
                case 'Q':
                    RCLCPP_INFO(this->get_logger(), "Exiting...");
                    rclcpp::shutdown();
                    return;
            }
        }
    }

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr command_publisher_;
    rclcpp::TimerBase::SharedPtr input_timer_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<StepperMotorTeleop>());
    rclcpp::shutdown();
    return 0;
}