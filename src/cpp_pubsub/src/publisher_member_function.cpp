#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"
#include <sensor_msgs/Join


using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
* member function as a callback from the timer. */

class MinimalPublisher : public rclcpp::Node
{
public:
    MinimalPublisher()
            : Node("minimal_publisher"), count_(0)
    {
        pub_joint_target = this->create_publisher<sensor_msgs::msg::JointState>("/joint_target", 1);
        subscription_ = this->create_subscription<sensor_msgs::msg::JointState>(
                "/joint_state", 1, std::bind(&MinimalSubscriber::sub_joint_state, this, _1));
        timer_ = this->create_wall_timer(
                2ms, std::bind(&MinimalPublisher::timer_callback, this));
    }

private:
//    sensor::msg::JointState joint_target;
    auto joint_target = std_msgs::msg::String();
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr subscription_;

    void sub_joint_state(const sensor_msgs::msg::JointState & msg) const
    {

    }

    void timer_callback()
    {
//        auto message = std_msgs::msg::String();
//        message.data = "Hello, world! " + std::to_string(count_++);
//        RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
//        publisher_->publish(message);

         for(int i = 0 ; i < joint_num ; i++){
             joint_target.effort[i] = kp[i] * (target_position - joint_state.position[i]) + kd[i] * (0 - joint_state.velocity[i])
         }

         pub_joint_target->publish(joint_target)
    }
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    size_t count_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MinimalPublisher>());
    rclcpp::shutdown();
    return 0;
}