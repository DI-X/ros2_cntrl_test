#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"
#include "sensor_msgs/msg/joint_state.hpp"


using std::placeholders::_1;
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
        pub_js_sub_time = this->create_publisher<std_msgs::msg::Float32>("/js_sub_time", 1);
        pub_js_target_pub_time = this->create_publisher<std_msgs::msg::Float32>("/js_target_pub_time", 1);

        subscription_ = this->create_subscription<sensor_msgs::msg::JointState>(
                "/joint_state", 1, std::bind(&MinimalPublisher::sub_joint_state, this, _1));
        timer_ = this->create_wall_timer(
                2ms, std::bind(&MinimalPublisher::timer_callback, this));

        joint_states_.name.resize(joint_num+1);
        joint_states_.position.resize(joint_num+1);
        joint_states_.velocity.resize(joint_num+1);
        joint_states_.effort.resize(joint_num+1);

        joint_target.name.resize(joint_num+1);
        joint_target.position.resize(joint_num+1);
        joint_target.velocity.resize(joint_num+1);
        joint_target.effort.resize(joint_num);
    }

private:
//    sensor::msg::JointState joint_target;
//    auto joint_target = std_msgs::msg::String();
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr subscription_;

    void sub_joint_state(const sensor_msgs::msg::JointState & msg)
    {
        sub_time =
        for(int i = 0; i<joint_num+1 ; i++){
            joint_states_.name[i] = msg.name[i];
            joint_states_.position[i] = msg.position[i];
            joint_states_.velocity[i] = msg.velocity[i];
        }
    }

    void timer_callback()
    {
//        auto message = std_msgs::msg::String();
//        message.data = "Hello, world! " + std::to_string(count_++);
//        RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
//        publisher_->publish(message);

         for(int i = 0 ; i < joint_num ; i++){
             joint_target.effort[i] = kp[i] * (target_position[i] - joint_states_.position[i]) + kd[i] * (0 - joint_states_.velocity[i]);
             joint_target.position[i] = target_position[i];
             joint_target.name[i] = joint_states_.name[i];
         }
         joint_target.position[joint_num] = joint_states_.position[joint_num];
//        joint_target.effort[joint_num] = 0;

         pub_joint_target->publish(joint_target);
    }
    double kp[5], kd[5], target_position[5], sub_time;
    int joint_num = 5;
    sensor_msgs::msg::JointState joint_states_, joint_target;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr pub_joint_target;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_js_sub_time;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_js_target_pub_time;
    size_t count_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MinimalPublisher>());
    rclcpp::shutdown();
    return 0;
}