/**
 * @file standard_subscriber.cpp
 * @brief Standard Subscriber Node - Riceve array usando ROS2 standard
 * @brief Standard Subscriber Node - Receives array using standard ROS2
 * 
 * IT: Subscriber STANDARD senza zero-copy. Usa le librerie ROS2 normali
 *     con il middleware di default. Questo dovrebbe funzionare sempre.
 * 
 * EN: STANDARD subscriber without zero-copy. Uses normal ROS2 libraries
 *     with default middleware. This should always work.
 * 
 * @author oe
 * @date 2025
 */

#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32_multi_array.hpp"

/**
 * @class StandardSubscriber
 * @brief Subscriber ROS2 standard senza zero-copy
 * @brief Standard ROS2 subscriber without zero-copy
 */
class StandardSubscriber : public rclcpp::Node
{
public:
    StandardSubscriber()
    : Node("standard_subscriber"), msg_count_(0)
    {
        // Crea subscriber con messaggio standard ROS2
        // Create subscriber with standard ROS2 message
        subscription_ = this->create_subscription<std_msgs::msg::Int32MultiArray>(
            "standard_array", 
            10,
            std::bind(&StandardSubscriber::message_callback, this, std::placeholders::_1));
        
        RCLCPP_INFO(this->get_logger(), 
            "[IT] Subscriber STANDARD avviato. In ascolto su 'standard_array'.");
        RCLCPP_INFO(this->get_logger(), 
            "[EN] STANDARD Subscriber started. Listening on 'standard_array'.");
    }

private:
    void message_callback(const std_msgs::msg::Int32MultiArray::SharedPtr msg)
    {
        ++msg_count_;
        
        size_t size = msg->data.size();
        int32_t first = size > 0 ? msg->data[0] : 0;
        int32_t last = size > 0 ? msg->data[size - 1] : 0;
        
        RCLCPP_INFO(this->get_logger(),
            "[IT] Msg #%zu ricevuto | Primo: %d | Ultimo: %d | Dim: %zu",
            msg_count_, first, last, size);
        RCLCPP_INFO(this->get_logger(),
            "[EN] Msg #%zu received | First: %d | Last: %d | Size: %zu",
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
