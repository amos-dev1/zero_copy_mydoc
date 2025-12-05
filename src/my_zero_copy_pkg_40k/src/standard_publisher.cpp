/**
 * @file standard_publisher.cpp
 * @brief Standard Publisher Node - Pubblica array di 40.000 interi
 * @brief Standard Publisher Node - Publishes 40,000 integer array
 * 
 * IT: Publisher STANDARD senza zero-copy. Usa std_msgs con 40.000 interi.
 * EN: STANDARD publisher without zero-copy. Uses std_msgs with 40,000 integers.
 * 
 * @author oe
 * @date 2025
 */

#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32_multi_array.hpp"

using namespace std::chrono_literals;

// MODIFICA: Array di 40.000 interi invece di 125
// CHANGE: Array of 40,000 integers instead of 125
constexpr size_t ARRAY_SIZE = 40000;

/**
 * @class StandardPublisher
 * @brief Publisher standard per 40.000 interi
 * @brief Standard publisher for 40,000 integers
 */
class StandardPublisher : public rclcpp::Node
{
public:
    StandardPublisher()
    : Node("standard_publisher_40k"), count_(0)
    {
        publisher_ = this->create_publisher<std_msgs::msg::Int32MultiArray>(
            "standard_array_40k", 10);
        
        timer_ = this->create_wall_timer(
            1s, std::bind(&StandardPublisher::publish_message, this));
        
        RCLCPP_INFO(this->get_logger(), 
            "[IT] Publisher STANDARD 40K avviato. %zu interi ogni secondo.", ARRAY_SIZE);
        RCLCPP_INFO(this->get_logger(), 
            "[EN] STANDARD 40K Publisher started. %zu integers every second.", ARRAY_SIZE);
    }

private:
    void publish_message()
    {
        auto msg = std_msgs::msg::Int32MultiArray();
        msg.data.resize(ARRAY_SIZE);
        
        for (size_t i = 0; i < ARRAY_SIZE; ++i) {
            msg.data[i] = static_cast<int32_t>(i + count_);
        }
        
        RCLCPP_INFO(this->get_logger(),
            "[IT] Msg #%zu | Primo: %d | Ultimo: %d | 40K",
            count_, msg.data[0], msg.data[ARRAY_SIZE - 1]);
        
        publisher_->publish(msg);
        ++count_;
    }
    
    rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    size_t count_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<StandardPublisher>());
    rclcpp::shutdown();
    return 0;
}
