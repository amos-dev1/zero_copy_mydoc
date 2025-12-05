/**
 * @file loaned_subscriber.cpp
 * @brief Zero-Copy Subscriber Node - Riceve array di 40.000 interi
 * @brief Zero-Copy Subscriber Node - Receives 40,000 integer arrays
 * 
 * IT: Subscriber per messaggi zero-copy con 40.000 interi (160 KB).
 * EN: Subscriber for zero-copy messages with 40,000 integers (160 KB).
 * 
 * @author oe
 * @date 2025
 */

#include <memory>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "my_zero_copy_pkg_40k/msg/fixed_array40k.hpp"

// MODIFICA: Array di 40.000 interi invece di 125
// CHANGE: Array of 40,000 integers instead of 125
constexpr size_t ARRAY_SIZE = 40000;

/**
 * @class LoanedSubscriber
 * @brief Subscriber per 40.000 interi via zero-copy
 * @brief Subscriber for 40,000 integers via zero-copy
 */
class LoanedSubscriber : public rclcpp::Node
{
public:
    LoanedSubscriber()
    : Node("loaned_subscriber_40k"), msg_count_(0)
    {
        setvbuf(stdout, NULL, _IONBF, BUFSIZ);
        
        rclcpp::QoS qos(rclcpp::KeepLast(7));
        
        subscription_ = this->create_subscription<my_zero_copy_pkg_40k::msg::FixedArray40k>(
            "fixed_array_40k", 
            qos,
            std::bind(&LoanedSubscriber::message_callback, this, std::placeholders::_1));
        
        RCLCPP_INFO(this->get_logger(), 
            "[IT] Subscriber LOANED 40K avviato. In ascolto su 'fixed_array_40k'.");
        RCLCPP_INFO(this->get_logger(), 
            "[EN] LOANED 40K Subscriber started. Listening on 'fixed_array_40k'.");
    }

private:
    void message_callback(const my_zero_copy_pkg_40k::msg::FixedArray40k::SharedPtr msg)
    {
        auto now = this->now();
        int64_t msg_ns = static_cast<int64_t>(msg->timestamp_sec) * 1000000000LL + 
                         static_cast<int64_t>(msg->timestamp_nanosec);
        int64_t now_ns = now.nanoseconds();
        double latency_ms = static_cast<double>(now_ns - msg_ns) / 1e6;
        
        ++msg_count_;
        
        int32_t first_val = msg->data[0];
        int32_t last_val = msg->data[ARRAY_SIZE - 1];
        int32_t expected_diff = static_cast<int32_t>(ARRAY_SIZE - 1);
        bool integrity_ok = (last_val - first_val) == expected_diff;
        
        RCLCPP_INFO(this->get_logger(),
            "[IT] Msg #%zu | Primo: %d | Ultimo: %d | Latenza: %.3f ms | OK: %s",
            msg_count_, first_val, last_val, latency_ms, 
            integrity_ok ? "SI" : "NO");
    }
    
    rclcpp::Subscription<my_zero_copy_pkg_40k::msg::FixedArray40k>::SharedPtr subscription_;
    size_t msg_count_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LoanedSubscriber>());
    rclcpp::shutdown();
    return 0;
}
