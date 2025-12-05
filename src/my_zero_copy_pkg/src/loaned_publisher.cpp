/**
 * @file loaned_publisher.cpp
 * @brief Zero-Copy Publisher Node - Pubblica array usando loaned messages
 * @brief Zero-Copy Publisher Node - Publishes array using loaned messages
 * 
 * IT: Publisher con ZERO-COPY usando borrow_loaned_message() API.
 *     NOTA: Questo causa NotEnoughMemoryException in alcune configurazioni.
 * 
 * EN: Publisher with ZERO-COPY using borrow_loaned_message() API.
 *     NOTE: This causes NotEnoughMemoryException in some configurations.
 * 
 * @author oe
 * @date 2025
 */

#include <chrono>
#include <memory>
#include <utility>

#include "rclcpp/rclcpp.hpp"
#include "my_zero_copy_pkg/msg/fixed_array40k.hpp"

using namespace std::chrono_literals;

constexpr size_t ARRAY_SIZE = 125;

/**
 * @class LoanedPublisher
 * @brief Publisher con zero-copy usando loaned messages
 * @brief Publisher with zero-copy using loaned messages
 */
class LoanedPublisher : public rclcpp::Node
{
public:
    LoanedPublisher()
    : Node("loaned_publisher"), count_(0)
    {
        setvbuf(stdout, NULL, _IONBF, BUFSIZ);
        
        rclcpp::QoS qos(rclcpp::KeepLast(7));
        
        publisher_ = this->create_publisher<my_zero_copy_pkg::msg::FixedArray40k>(
            "fixed_array_125", qos);
        
        timer_ = this->create_wall_timer(
            1s, std::bind(&LoanedPublisher::publish_message, this));
        
        // Verifica supporto loaned messages
        if (publisher_->can_loan_messages()) {
            RCLCPP_INFO(this->get_logger(), 
                "[IT] Zero-copy ABILITATO - Il middleware supporta loaned messages.");
            RCLCPP_INFO(this->get_logger(), 
                "[EN] Zero-copy ENABLED - Middleware supports loaned messages.");
        } else {
            RCLCPP_WARN(this->get_logger(),
                "[IT] Zero-copy NON DISPONIBILE - Usando pubblicazione standard.");
            RCLCPP_WARN(this->get_logger(),
                "[EN] Zero-copy NOT AVAILABLE - Using standard publishing.");
        }
        
        RCLCPP_INFO(this->get_logger(), 
            "[IT] Publisher LOANED avviato. Pubblicazione di %zu interi ogni secondo.", ARRAY_SIZE);
        RCLCPP_INFO(this->get_logger(), 
            "[EN] LOANED Publisher started. Publishing %zu integers every second.", ARRAY_SIZE);
    }

private:
    void publish_message()
    {
        if (!publisher_->can_loan_messages()) {
            // Fallback a pubblicazione standard
            auto msg = my_zero_copy_pkg::msg::FixedArray40k();
            
            auto now = this->now();
            msg.timestamp_sec = static_cast<int32_t>(now.seconds());
            msg.timestamp_nanosec = static_cast<uint32_t>(now.nanoseconds() % 1000000000);
            
            for (size_t i = 0; i < ARRAY_SIZE; ++i) {
                msg.data[i] = static_cast<int32_t>(i + count_);
            }
            
            RCLCPP_INFO(this->get_logger(),
                "[IT] Msg #%zu (standard) | Primo: %d | Ultimo: %d",
                count_, msg.data[0], msg.data[ARRAY_SIZE - 1]);
            
            publisher_->publish(msg);
        } else {
            // Pubblicazione zero-copy con loaned message
            auto loaned_msg = publisher_->borrow_loaned_message();
            
            auto now = this->now();
            loaned_msg.get().timestamp_sec = static_cast<int32_t>(now.seconds());
            loaned_msg.get().timestamp_nanosec = static_cast<uint32_t>(now.nanoseconds() % 1000000000);
            
            for (size_t i = 0; i < ARRAY_SIZE; ++i) {
                loaned_msg.get().data[i] = static_cast<int32_t>(i + count_);
            }
            
            RCLCPP_INFO(this->get_logger(),
                "[IT] Msg #%zu (zero-copy) | Primo: %d | Ultimo: %d",
                count_, loaned_msg.get().data[0], loaned_msg.get().data[ARRAY_SIZE - 1]);
            
            publisher_->publish(std::move(loaned_msg));
        }
        
        ++count_;
    }
    
    rclcpp::Publisher<my_zero_copy_pkg::msg::FixedArray40k>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    size_t count_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LoanedPublisher>());
    rclcpp::shutdown();
    return 0;
}
