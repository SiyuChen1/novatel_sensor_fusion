//
// Created by siyuchen on 11.08.23.
//
#include <chrono>
#include <memory>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_epsilon_time.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/temperature.hpp>

using namespace std::chrono_literals;

class SyncerNode : public rclcpp::Node {
public:
    SyncerNode() : Node("syncer"), count1_(0), count2_(0)
    {
        rclcpp::QoS qos(10);
        auto rmw_qos_profile = qos.get_rmw_qos_profile();

        publisher_temp1_ =
                this->create_publisher<sensor_msgs::msg::Temperature>("temp_1", qos);
        publisher_temp2_ =
                this->create_publisher<sensor_msgs::msg::Temperature>("temp_2", qos);

        timer1_ = this->create_wall_timer(
                500ms, std::bind(&SyncerNode::TimerCallback1, this));

        timer2_ = this->create_wall_timer(
                1000ms, std::bind(&SyncerNode::TimerCallback2, this));

        subscriber_temp1_.subscribe(this, "temp_1", rmw_qos_profile);
        subscriber_temp2_.subscribe(this, "temp_2", rmw_qos_profile);

        uint32_t queue_size = 10;
        rclcpp::Duration epsilon(0, 5e4);
        sync = std::make_shared<Sync>(MySyncPolicy{queue_size, epsilon}, subscriber_temp1_, subscriber_temp2_);
        sync->registerCallback(&SyncerNode::TempSyncCallback, this);
    }

private:
    void TimerCallback1() {
        rclcpp::Time now = this->get_clock()->now();

        auto msg_tmp1 = sensor_msgs::msg::Temperature();
        msg_tmp1.header.stamp = now;
        msg_tmp1.header.frame_id = "A" + std::to_string(count1_ ++);
        msg_tmp1.temperature = 1.0;

        publisher_temp1_->publish(msg_tmp1);

        RCLCPP_INFO(this->get_logger(), "Published first temperature.");
    }

    void TimerCallback2() {
        rclcpp::Time now = this->get_clock()->now();

        auto msg_tmp2 = sensor_msgs::msg::Temperature();
        msg_tmp2.header.stamp = now + offset;
        msg_tmp2.header.frame_id = "B" + std::to_string(count2_ ++);
        msg_tmp2.temperature = 2.0;

        publisher_temp2_->publish(msg_tmp2);

        RCLCPP_INFO(this->get_logger(), "Published second temperature.");
    }

    // For veryfing the single subscriber instances: Uncomment line 26-28.
    void Tmp1Callback(const sensor_msgs::msg::Temperature::ConstSharedPtr& msg) {
        RCLCPP_INFO(this->get_logger(), "Frame '%s', temp %f with ts %u.%u sec ",
                    msg->header.frame_id.c_str(), msg->temperature,
                    msg->header.stamp.sec, msg->header.stamp.nanosec);
    }

    // For veryfing the single subscriber instances: Uncomment line 29-31.
    void Tmp2Callback(const sensor_msgs::msg::Temperature::ConstSharedPtr& msg) {
        RCLCPP_INFO(this->get_logger(), "Frame '%s', temp %f with ts %u.%u sec ",
                    msg->header.frame_id.c_str(), msg->temperature,
                    msg->header.stamp.sec, msg->header.stamp.nanosec);
    }

    // This callback is never being called.
    void TempSyncCallback(
            const sensor_msgs::msg::Temperature::ConstSharedPtr& msg_1,
            const sensor_msgs::msg::Temperature::ConstSharedPtr& msg_2) {
        RCLCPP_INFO(this->get_logger(),
                    "I heard and synchronized the following timestamps in sec: %u, %u",
                    msg_1->header.stamp.sec, msg_2->header.stamp.sec);
        RCLCPP_INFO(this->get_logger(),
                    "I heard and synchronized the following timestamps in nanosec: %u, %u",
                    msg_1->header.stamp.nanosec, msg_2->header.stamp.nanosec);
        RCLCPP_INFO(this->get_logger(), "I heard and synchronized messages, %s, %s",
                    msg_1->header.frame_id.c_str(), msg_2->header.frame_id.c_str());
    }

    rclcpp::Publisher<sensor_msgs::msg::Temperature>::SharedPtr publisher_temp1_;
    rclcpp::Publisher<sensor_msgs::msg::Temperature>::SharedPtr publisher_temp2_;
    message_filters::Subscriber<sensor_msgs::msg::Temperature> subscriber_temp1_;
    message_filters::Subscriber<sensor_msgs::msg::Temperature> subscriber_temp2_;

    rclcpp::TimerBase::SharedPtr timer1_;
    rclcpp::TimerBase::SharedPtr timer2_;
    uint32_t count1_, count2_;
    rclcpp::Duration offset = rclcpp::Duration(1, 0);

    typedef message_filters::sync_policies::ApproximateEpsilonTime<sensor_msgs::msg::Temperature, sensor_msgs::msg::Temperature> MySyncPolicy;
    typedef message_filters::Synchronizer<MySyncPolicy> Sync;
    std::shared_ptr<Sync> sync;
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SyncerNode>());
    rclcpp::shutdown();
    return 0;
}