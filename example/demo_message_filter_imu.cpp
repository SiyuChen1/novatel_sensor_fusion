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
#include "sensor_msgs/msg/imu.hpp"
#include "std_msgs/msg/header.hpp"

using namespace std::chrono_literals;

class DemoMessageFilter : public rclcpp::Node
{
public:
    DemoMessageFilter(const uint32_t& queue_size, const rclcpp::Duration& epsilon)
            : Node("demo_message_filter"), count1_(0), count2_(0)
    {
        sync_ = std::make_shared<Sync>(MySyncPolicy{queue_size, epsilon});
        sync_->registerCallback(std::bind(&DemoMessageFilter::callback, this, std::placeholders::_1, std::placeholders::_2));
    }

    void run(){
        rclcpp::Time now = this->get_clock()->now();
        for(int i = 0; i < 5; i++){
            auto p(std::make_shared<sensor_msgs::msg::Imu>());
            auto q(std::make_shared<sensor_msgs::msg::Imu>());
            rclcpp::Duration d(i * 10, 0);
            p->header.stamp = now - d;
            p->header.frame_id = "A" + std::to_string(count1_ ++);
            q->header.stamp = now - d;
            q->header.frame_id = "B" + std::to_string(count2_ ++);
            sync_->add<0>(p);
            sync_->add<1>(q);
        }
    }

private:
    void callback(const sensor_msgs::msg::Imu::ConstSharedPtr& msg_1, const sensor_msgs::msg::Imu::ConstSharedPtr& msg_2){
        RCLCPP_INFO(this->get_logger(), "New syn Frame '%s', with ts %u.%u sec ",
                    msg_1->header.frame_id.c_str(), msg_1->header.stamp.sec, msg_1->header.stamp.nanosec);
        RCLCPP_INFO(this->get_logger(), "Frame '%s', with ts %u.%u sec ",
                    msg_2->header.frame_id.c_str(), msg_2->header.stamp.sec, msg_2->header.stamp.nanosec);
    }

    typedef message_filters::sync_policies::ApproximateEpsilonTime<sensor_msgs::msg::Imu, sensor_msgs::msg::Imu> MySyncPolicy;
    typedef message_filters::Synchronizer<MySyncPolicy> Sync;
    std::shared_ptr<Sync> sync_;
    uint32_t count1_, count2_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    uint32_t queue_size = 10;
    rclcpp::Duration s(0, 1000);
    auto mf = std::make_shared<DemoMessageFilter>(queue_size, s);
    mf->run();
    rclcpp::shutdown();
    return 0;
}