//
// Created by siyuchen on 10.08.23.
//

#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "message_filters/subscriber.h"
#include "message_filters/sync_policies/approximate_epsilon_time.h"
#include "novatel_sensor_fusion/msg/nav_sat_extended.hpp"

typedef novatel_sensor_fusion::msg::NavSatExtended GNSSMsg;
typedef message_filters::Subscriber<GNSSMsg> GNSSSubscriber;
typedef message_filters::sync_policies::ApproximateEpsilonTime<GNSSMsg, GNSSMsg> MySyncPolicy;
typedef message_filters::Synchronizer<MySyncPolicy> Sync;


class GNSSMessageFilterNode : public rclcpp::Node
{
public:
    GNSSMessageFilterNode(uint32_t queue_size, rclcpp::Duration epsilon) : Node("gnss_message_filter_node")
    {
        sub1_.subscribe(this, "/best");
        sub2_.subscribe(this, "/bestgnss");
        sync_ = std::make_shared<Sync>(MySyncPolicy{queue_size, epsilon}, sub1_, sub2_);
        sync_->registerCallback(&GNSSMessageFilterNode::callback, this);
        pub_ = this->create_publisher<GNSSMsg>("/diff_bestgnss_best", 10);
    };

private:
    void callback(const GNSSMsg::SharedPtr msg1,
                  const GNSSMsg::SharedPtr msg2)
    {
        RCLCPP_INFO(this->get_logger(), "Synchronized messages in sec: %d, %d", msg1->header.stamp.sec, msg2->header.stamp.sec);
        RCLCPP_INFO(this->get_logger(), "Synchronized messages in nanosec: %d, %d", msg1->header.stamp.nanosec, msg2->header.stamp.nanosec);
        pub_->publish(*msg1);
    }

    GNSSSubscriber sub1_, sub2_;
    std::shared_ptr<Sync> sync_;
    rclcpp::Publisher<GNSSMsg>::SharedPtr pub_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::Duration s(0, 1e8);
    auto node = std::make_shared<GNSSMessageFilterNode>(10, s);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
