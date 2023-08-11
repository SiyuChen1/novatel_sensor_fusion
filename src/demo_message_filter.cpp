//
// Created by siyuchen on 11.08.23.
//

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "message_filters/subscriber.h"
#include "message_filters/sync_policies/approximate_epsilon_time.h"
#include "message_filters/sync_policies/approximate_time.h"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/header.hpp"

using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
* member function as a callback from the timer. */

class DemoMessageFilter : public rclcpp::Node
{
public:
    DemoMessageFilter(const uint32_t& queue_size, const rclcpp::Duration& epsilon)
            : Node("demo_message_filter")
    {
        rclcpp::QoS qos(10);
        auto rmw_qos_profile = qos.get_rmw_qos_profile();

        sub_1.subscribe(this, "/topic_1", rmw_qos_profile);
        sub_2.subscribe(this, "/topic_2", rmw_qos_profile);

         // Uncomment this to verify that the messages indeed reach the
         sub_1.registerCallback(
             std::bind(&DemoMessageFilter::Tmp1Callback, this, std::placeholders::_1));
         sub_2.registerCallback(
             std::bind(&DemoMessageFilter::Tmp2Callback, this, std::placeholders::_1));

        sync_ = std::make_shared<Sync>(MySyncPolicy{queue_size}, sub_1, sub_2);
        sync_->registerCallback(&DemoMessageFilter::callback, this);
    }

private:
    // For veryfing the single subscriber instances: Uncomment line 26-28.
    void Tmp1Callback(const std_msgs::msg::Header::ConstSharedPtr& msg) {
        RCLCPP_INFO(this->get_logger(), "Frame '%s', with ts %u.%u sec ",
                    msg->frame_id.c_str(), msg->stamp.sec, msg->stamp.nanosec);
    }

    // For veryfing the single subscriber instances: Uncomment line 29-31.
    void Tmp2Callback(const std_msgs::msg::Header::ConstSharedPtr& msg) {
        RCLCPP_INFO(this->get_logger(), "Frame '%s', with ts %u.%u sec ",
                    msg->frame_id.c_str(), msg->stamp.sec, msg->stamp.nanosec);
    }


    void callback(const std_msgs::msg::Header::ConstSharedPtr& msg_1, const std_msgs::msg::Header::ConstSharedPtr& msg_2){
        RCLCPP_INFO(this->get_logger(), "Frame '%s', with ts %u.%u sec ",
                    msg_1->frame_id.c_str(), msg_1->stamp.sec, msg_1->stamp.nanosec);
        RCLCPP_INFO(this->get_logger(), "Frame '%s', with ts %u.%u sec ",
                    msg_2->frame_id.c_str(), msg_2->stamp.sec, msg_2->stamp.nanosec);
        RCLCPP_INFO(this->get_logger(), "Synchronized messages in sec: %d, %d", msg_1->stamp.sec, msg_2->stamp.sec);
        RCLCPP_INFO(this->get_logger(), "Synchronized messages in nanosec: %d, %d", msg_1->stamp.nanosec, msg_2->stamp.nanosec);
    }
    message_filters::Subscriber<std_msgs::msg::Header> sub_1, sub_2;
    typedef message_filters::sync_policies::ApproximateTime<std_msgs::msg::Header, std_msgs::msg::Header> MySyncPolicy;
    typedef message_filters::Synchronizer<MySyncPolicy> Sync;
    std::shared_ptr<Sync> sync_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    uint32_t queue_size = 10;
    rclcpp::Duration s(1, 1000);
    rclcpp::spin(std::make_shared<DemoMessageFilter>(queue_size, s));
    rclcpp::shutdown();
    return 0;
}