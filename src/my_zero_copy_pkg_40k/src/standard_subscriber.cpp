/**
 * @file standard_subscriber.cpp
 * @brief Standard Subscriber Node - Riceve array di 40.000 interi
 * @brief Standard Subscriber Node - Receives 40,000 integer array
 * 
 * IT: Subscriber STANDARD senza zero-copy. Usa std_msgs con 40.000 interi.
 * EN: STANDARD subscriber without zero-copy. Uses std_msgs with 40,000 integers.
 * 
 * @author oe
 * @date 2025
 */

#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32_multi_array.hpp"

/**
 * @class StandardSubscriber
 * @brief Subscriber standard per 40.000 interi
 * @brief Standard subscriber for 40,000 integers
 */
class StandardSubscriber : public rclcpp::Node
{
public:
    StandardSubscriber()
    : Node("standard_subscriber_40k"), msg_count_(0)
    {
        subscription_ = this->create_subscription<std_msgs::msg::Int32MultiArray>(
            "standard_array_40k", 
            10,
            std::bind(&StandardSubscriber::message_callback, this, std::placeholders::_1));
        
        RCLCPP_INFO(this->get_logger(), 
            "[IT] Subscriber STANDARD 40K avviato. In ascolto su 'standard_array_40k'.");
        RCLCPP_INFO(this->get_logger(), 
            "[EN] STANDARD 40K Subscriber started. Listening on 'standard_array_40k'.");
    }

private:
    void message_callback(const std_msgs::msg::Int32MultiArray::SharedPtr msg)
    {
        ++msg_count_;
        
        size_t size = msg->data.size();
        int32_t first = size > 0 ? msg->data[0] : 0;
        int32_t last = size > 0 ? msg->data[size - 1] : 0;
        
        RCLCPP_INFO(this->get_logger(),
            "[IT] Msg #%zu | Primo: %d | Ultimo: %d | Dim: %zu",
            msg_count_, first, last, size);
    }
    
    rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr subscription_;
    size_t msg_count_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<StandardSubscriber>());
    rclcpp::shutdown();
    return 0;
}
