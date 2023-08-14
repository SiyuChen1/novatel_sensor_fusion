//
// Created by siyuchen on 11.08.23.
//

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/header.hpp"

using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
* member function as a callback from the timer. */

class DemoMessageFilterPub : public rclcpp::Node
{
public:
    DemoMessageFilterPub()
            : Node("demo_message_filter_publisher"), count_1(0), count_2(0)
    {
        pub_1 = this->create_publisher<std_msgs::msg::Header>("/topic_1", 10);
        pub_2 = this->create_publisher<std_msgs::msg::Header>("/topic_2", 10);
        timer_1 = this->create_wall_timer(
                500ms, std::bind(&DemoMessageFilterPub::timer_1_callback, this));
        timer_2 = this->create_wall_timer(
                1000ms, std::bind(&DemoMessageFilterPub::timer_2_callback, this));
    }

private:
    void timer_1_callback()
    {
        auto now = this->get_clock()->now();
        auto message = std_msgs::msg::Header();
        message.frame_id = "A." + std::to_string(count_1++);
        message.stamp = now;
//        RCLCPP_INFO_STREAM(this->get_logger(), this->get_clock()->now().get_clock_type());
        pub_1->publish(message);
    }
    void timer_2_callback()
    {
        auto now = this->get_clock()->now();
        auto message = std_msgs::msg::Header();
        message.frame_id = "B." + std::to_string(count_2++);
        message.stamp = now;
//        RCLCPP_INFO_STREAM(this->get_logger(), this->get_clock()->now().get_clock_type());
        pub_2->publish(message);
    }

    rclcpp::TimerBase::SharedPtr timer_1, timer_2;
    uint32_t count_1, count_2;
    rclcpp::Publisher<std_msgs::msg::Header>::SharedPtr pub_1, pub_2;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DemoMessageFilterPub>());
    rclcpp::shutdown();
    return 0;
}