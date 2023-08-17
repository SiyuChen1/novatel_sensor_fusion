//
// Created by siyuchen on 10.08.23.
//

#include <memory>

#include "novatel_sensor_fusion/lla2enu.h"
#include "novatel_sensor_fusion/msg/nav_sat_extended.hpp"

#include "rclcpp/rclcpp.hpp"
#include "message_filters/subscriber.h"
#include "message_filters/sync_policies/approximate_epsilon_time.h"
#include "geometry_msgs/msg/vector3_stamped.hpp"

typedef novatel_sensor_fusion::msg::NavSatExtended GNSSMsg;
typedef message_filters::Subscriber<GNSSMsg> GNSSSubscriber;
typedef message_filters::sync_policies::ApproximateEpsilonTime<GNSSMsg, GNSSMsg> MySyncPolicy;
typedef message_filters::Synchronizer<MySyncPolicy> Sync;

struct LLA{
    double lat, lon, alt;
};

class GNSSMessageFilterNode : public rclcpp::Node
{
public:
    GNSSMessageFilterNode(uint32_t queue_size) : Node("gnss_message_filter_node"), lla_ref_ready(false)
    {
        std::string bestgnss_topic_name_value = "/bestgnss";
        std::string best_topic_name_value = "/best";
        std::string diff_best_bestgnss_value = "/diff_best_bestgnss";
        std::string diff_best_fused_value = "/diff_best_fused";
        int32_t sec = 0;
        int32_t nanosec= 1e3;

        this->declare_parameter<std::string>("best_topic_name", best_topic_name_value);
        this->declare_parameter<std::string>("bestgnss_topic_name", bestgnss_topic_name_value);
        this->declare_parameter<std::string>("difference_best_bestgnss", diff_best_bestgnss_value);
        this->declare_parameter<std::string>("difference_best_fused", diff_best_fused_value);
        this->declare_parameter<int32_t>("epsilon_sec", sec);
        this->declare_parameter<int32_t>("epsilon_nanosec", nanosec);

        if (this->get_parameter("best_topic_name").get_type() != rclcpp::ParameterType::PARAMETER_NOT_SET){
            best_topic_name_value = this->get_parameter("best_topic_name").as_string();
            this->set_parameter(rclcpp::Parameter("best_topic_name", best_topic_name_value));
        }

        if (this->get_parameter("bestgnss_topic_name").get_type() != rclcpp::ParameterType::PARAMETER_NOT_SET){
            bestgnss_topic_name_value = this->get_parameter("bestgnss_topic_name").as_string();
            this->set_parameter(rclcpp::Parameter("bestgnss_topic_name", bestgnss_topic_name_value));
        }

        if (this->get_parameter("difference_best_bestgnss").get_type() != rclcpp::ParameterType::PARAMETER_NOT_SET){
            diff_best_bestgnss_value = this->get_parameter("difference_best_bestgnss").as_string();
            RCLCPP_INFO(this->get_logger(), "%s", diff_best_bestgnss_value.c_str());
            this->set_parameter(rclcpp::Parameter("difference_best_bestgnss", diff_best_bestgnss_value));
        }

        if (this->get_parameter("difference_best_fused").get_type() != rclcpp::ParameterType::PARAMETER_NOT_SET){
            diff_best_fused_value = this->get_parameter("difference_best_fused").as_string();
            this->set_parameter(rclcpp::Parameter("difference_best_fused", diff_best_fused_value));
        }

        if (this->get_parameter("epsilon_sec").get_type() != rclcpp::ParameterType::PARAMETER_NOT_SET){
            sec = this->get_parameter("epsilon_sec").as_int();
            this->set_parameter(rclcpp::Parameter("epsilon_sec", sec));
        }

        if (this->get_parameter("epsilon_nanosec").get_type() != rclcpp::ParameterType::PARAMETER_NOT_SET){
            nanosec = this->get_parameter("epsilon_nanosec").as_int();
            this->set_parameter(rclcpp::Parameter("epsilon_nanosec", nanosec));
        }

        sub1_.subscribe(this, best_topic_name_value);
        sub2_.subscribe(this, bestgnss_topic_name_value);

        sub1_.registerCallback(std::bind(&GNSSMessageFilterNode::lla_ref_callback, this, std::placeholders::_1));

        rclcpp::Duration epsilon(sec, nanosec);
        sync_ = std::make_shared<Sync>(MySyncPolicy{queue_size, epsilon}, sub1_, sub2_);
        sync_->registerCallback(&GNSSMessageFilterNode::callback, this);

        pub1_ = this->create_publisher<geometry_msgs::msg::Vector3Stamped>(diff_best_bestgnss_value, 10);
//        pub2_ = this->create_publisher<GNSSMsg>(diff_best_fused_value, 10);
    };

private:
    void callback(const GNSSMsg::ConstSharedPtr& msg1,
                  const GNSSMsg::ConstSharedPtr& msg2)
    {
        if(this->lla_ref_ready){
            double north1, north2, east1, east2, up1, up2;
            lla2enu(msg1->latitude, msg1->longitude, msg1->altitude,
                    this->lla.lat, this->lla.lon, this->lla.alt,
                    east1, north1, up1,true);
            lla2enu(msg2->latitude, msg2->longitude, msg2->altitude,
                    this->lla.lat, this->lla.lon, this->lla.alt,
                    east2, north2, up2,true);
//            RCLCPP_INFO(this->get_logger(), "Synchronized messages in sec: %d, %d", msg1->header.stamp.sec, msg2->header.stamp.sec);
//            RCLCPP_INFO(this->get_logger(), "in nanosec: %d, %d", msg1->header.stamp.nanosec, msg2->header.stamp.nanosec);
            auto msg = geometry_msgs::msg::Vector3Stamped();
            msg.header = msg2->header;
            msg.vector.x = east2 - east1;
            msg.vector.y = north2 - north1;
            msg.vector.z = up2 - up1;
            pub1_->publish(msg);
//            RCLCPP_INFO(this->get_logger(), "%d, %d", msg.header.stamp.sec, msg.header.stamp.nanosec);
        }
    };

    void lla_ref_callback(const GNSSMsg::ConstSharedPtr& msg){
        if (! this->lla_ref_ready){
//            this->lla.lat = msg->latitude / 180 * M_PI;
//            this->lla.lon = msg->longitude / 180 * M_PI;
            this->lla.lat = msg->latitude;
            this->lla.lon = msg->longitude;
            this->lla.alt = msg->altitude;
            this->lla_ref_ready = true;
        }
    };

    GNSSSubscriber sub1_, sub2_;
    std::shared_ptr<Sync> sync_;
    rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr pub1_, pub2_;
    bool lla_ref_ready;
    LLA lla;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<GNSSMessageFilterNode>(10);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
