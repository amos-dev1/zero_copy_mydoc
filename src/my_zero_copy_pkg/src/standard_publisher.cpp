/**
 * @file standard_publisher.cpp
 * @brief Standard Publisher Node - Pubblica array usando ROS2 standard
 * @brief Standard Publisher Node - Publishes array using standard ROS2
 * 
 * IT: Publisher STANDARD senza zero-copy. Usa le librerie ROS2 normali
 *     con il middleware di default. Questo dovrebbe funzionare sempre.
 * 
 * EN: STANDARD publisher without zero-copy. Uses normal ROS2 libraries
 *     with default middleware. This should always work.
 * 
 * @author oe
 * @date 2025
 */

#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32_multi_array.hpp"

using namespace std::chrono_literals;

// Dimensione array / Array size
constexpr size_t ARRAY_SIZE = 125;

/**
 * @class StandardPublisher
 * @brief Publisher ROS2 standard senza zero-copy
 * @brief Standard ROS2 publisher without zero-copy
 */
class StandardPublisher : public rclcpp::Node
{
public:
    StandardPublisher()
    : Node("standard_publisher"), count_(0)
    {
        // Crea publisher con messaggio standard ROS2
        // Create publisher with standard ROS2 message
        publisher_ = this->create_publisher<std_msgs::msg::Int32MultiArray>(
            "standard_array", 10);
        
        // Timer ogni secondo
        // Timer every second
        timer_ = this->create_wall_timer(
            1s, std::bind(&StandardPublisher::publish_message, this));
        
        RCLCPP_INFO(this->get_logger(), 
            "[IT] Publisher STANDARD avviato. Pubblicazione di %zu interi ogni secondo.", ARRAY_SIZE);
        RCLCPP_INFO(this->get_logger(), 
            "[EN] STANDARD Publisher started. Publishing %zu integers every second.", ARRAY_SIZE);
    }

private:
    void publish_message()
    {
        // Crea messaggio standard / Create standard message
        auto msg = std_msgs::msg::Int32MultiArray();
        
        // Ridimensiona l'array / Resize array
        msg.data.resize(ARRAY_SIZE);
        
        // Riempie l'array / Fill array
        for (size_t i = 0; i < ARRAY_SIZE; ++i) {
            msg.data[i] = static_cast<int32_t>(i + count_);
        }
        
        // Log
        RCLCPP_INFO(this->get_logger(),
            "[IT] Msg #%zu | Primo: %d | Ultimo: %d | Dim: %zu",
            count_, msg.data[0], msg.data[ARRAY_SIZE - 1], ARRAY_SIZE);
        
        // Pubblica / Publish
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
