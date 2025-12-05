/**
 * @file loaned_subscriber.cpp
 * @brief Zero-Copy Subscriber Node - Riceve array di 125 interi
 * @brief Zero-Copy Subscriber Node - Receives 125 integer arrays
 * 
 * IT: Questo nodo riceve messaggi pubblicati dal publisher zero-copy.
 *     IMPORTANTE: Il messaggio deve essere PURO POD (nessuna stringa/Header)!
 * 
 * EN: This node receives messages published by the zero-copy publisher.
 *     IMPORTANT: Message must be PURE POD (no strings/Header)!
 * 
 * @author oe
 * @date 2025
 */

#include <memory>    // Per smart pointers / For smart pointers
#include <chrono>    // Per calcolo latenza / For latency calculation

#include "rclcpp/rclcpp.hpp"
#include "my_zero_copy_pkg/msg/fixed_array40k.hpp"

// Costante per la dimensione dell'array / Constant for array size
constexpr size_t ARRAY_SIZE = 125;

/**
 * @class LoanedSubscriber
 * @brief Nodo ROS2 che riceve array di 125 interi via zero-copy
 * @brief ROS2 node that receives 125 integer arrays via zero-copy
 */
class LoanedSubscriber : public rclcpp::Node
{
public:
    /**
     * @brief Costruttore del nodo subscriber
     * @brief Subscriber node constructor
     */
    LoanedSubscriber()
    : Node("loaned_subscriber"), msg_count_(0)
    {
        // Disabilita il buffering dello stdout per log immediati
        // Disable stdout buffering for immediate logs
        setvbuf(stdout, NULL, _IONBF, BUFSIZ);
        
        // Configura QoS con storico degli ultimi 7 messaggi
        // Configure QoS with history of last 7 messages
        rclcpp::QoS qos(rclcpp::KeepLast(7));
        
        // Crea il subscriber per il tipo FixedArray40k
        // Create the subscriber for FixedArray40k type
        subscription_ = this->create_subscription<my_zero_copy_pkg::msg::FixedArray40k>(
            "fixed_array_125", 
            qos,
            std::bind(&LoanedSubscriber::message_callback, this, std::placeholders::_1));
        
        // Log di inizializzazione
        // Initialization log
        RCLCPP_INFO(this->get_logger(), 
            "[IT] Subscriber Zero-Copy avviato. In ascolto su 'fixed_array_125'.");
        RCLCPP_INFO(this->get_logger(), 
            "[EN] Zero-Copy Subscriber started. Listening on 'fixed_array_125'.");
    }

private:
    /**
     * @brief Callback per la ricezione dei messaggi
     * @brief Callback for message reception
     */
    void message_callback(const my_zero_copy_pkg::msg::FixedArray40k::SharedPtr msg)
    {
        // Calcola la latenza del messaggio usando timestamp POD
        // Calculate message latency using POD timestamp
        auto now = this->now();
        int64_t msg_ns = static_cast<int64_t>(msg->timestamp_sec) * 1000000000LL + 
                         static_cast<int64_t>(msg->timestamp_nanosec);
        int64_t now_ns = now.nanoseconds();
        double latency_ms = static_cast<double>(now_ns - msg_ns) / 1e6;
        
        // Incrementa il contatore di messaggi ricevuti
        // Increment received message counter
        ++msg_count_;
        
        // Verifica l'integrità dei dati
        // Verify data integrity
        int32_t first_val = msg->data[0];
        int32_t last_val = msg->data[ARRAY_SIZE - 1];
        int32_t expected_diff = static_cast<int32_t>(ARRAY_SIZE - 1);
        bool integrity_ok = (last_val - first_val) == expected_diff;
        
        // Log del messaggio ricevuto con dettagli
        // Log received message with details
        RCLCPP_INFO(this->get_logger(),
            "[IT] Msg #%zu ricevuto | Primo: %d | Ultimo: %d | Latenza: %.3f ms | Integrità: %s",
            msg_count_, first_val, last_val, latency_ms, 
            integrity_ok ? "OK" : "ERRORE");
        RCLCPP_INFO(this->get_logger(),
            "[EN] Msg #%zu received | First: %d | Last: %d | Latency: %.3f ms | Integrity: %s",
            msg_count_, first_val, last_val, latency_ms,
            integrity_ok ? "OK" : "ERROR");
    }
    
    // Membri privati / Private members
    rclcpp::Subscription<my_zero_copy_pkg::msg::FixedArray40k>::SharedPtr subscription_;
    size_t msg_count_;  // Contatore messaggi ricevuti / Received message counter
};

/**
 * @brief Funzione principale
 * @brief Main function
 */
int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LoanedSubscriber>());
    rclcpp::shutdown();
    return 0;
}
