# 7. Subscriber Loaned (Zero-Copy) - Dettaglio

Questo capitolo spiega in dettaglio il subscriber che riceve messaggi zero-copy.

---

## Panoramica

Il **subscriber loaned** riceve messaggi dal publisher zero-copy senza copie aggiuntive, leggendo direttamente dalla memoria condivisa.

**File**: `src/loaned_subscriber.cpp`

---

## Nota importante

Il subscriber NON ha bisogno di codice speciale per zero-copy! La "magia" avviene automaticamente nel middleware. Il subscriber usa una normale callback e riceve un puntatore alla memoria condivisa.

---

## Codice Completo

```cpp
/**
 * @file loaned_subscriber.cpp
 * @brief Subscriber Zero-Copy - Riceve array di 40.000 interi
 * 
 * Subscriber per messaggi zero-copy con 40.000 interi (160 KB).
 */

#include <memory>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "my_zero_copy_pkg_40k/msg/fixed_array40k.hpp"

// Dimensione array: 40.000 interi
constexpr size_t ARRAY_SIZE = 40000;

/**
 * @class LoanedSubscriber
 * @brief Subscriber per 40.000 interi via zero-copy
 */
class LoanedSubscriber : public rclcpp::Node
{
public:
    LoanedSubscriber()
    : Node("loaned_subscriber_40k"), msg_count_(0)
    {
        // Disabilita buffering stdout
        setvbuf(stdout, NULL, _IONBF, BUFSIZ);
        
        // QoS con history depth 7
        rclcpp::QoS qos(rclcpp::KeepLast(7));
        
        // Crea subscription
        subscription_ = this->create_subscription<my_zero_copy_pkg_40k::msg::FixedArray40k>(
            "fixed_array_40k", 
            qos,
            std::bind(&LoanedSubscriber::message_callback, this, std::placeholders::_1));
        
        RCLCPP_INFO(this->get_logger(), 
            "Subscriber LOANED 40K avviato. In ascolto su 'fixed_array_40k'.");
    }

private:
    void message_callback(const my_zero_copy_pkg_40k::msg::FixedArray40k::SharedPtr msg)
    {
        // Calcola timestamp attuale
        auto now = this->now();
        
        // Ricostruisci timestamp del messaggio
        int64_t msg_ns = static_cast<int64_t>(msg->timestamp_sec) * 1000000000LL + 
                         static_cast<int64_t>(msg->timestamp_nanosec);
        int64_t now_ns = now.nanoseconds();
        
        // Calcola latenza in millisecondi
        double latency_ms = static_cast<double>(now_ns - msg_ns) / 1e6;
        
        ++msg_count_;
        
        // Verifica integrità dati
        int32_t first_val = msg->data[0];
        int32_t last_val = msg->data[ARRAY_SIZE - 1];
        int32_t expected_diff = static_cast<int32_t>(ARRAY_SIZE - 1);
        bool integrity_ok = (last_val - first_val) == expected_diff;
        
        RCLCPP_INFO(this->get_logger(),
            "Msg #%zu | Primo: %d | Ultimo: %d | Latenza: %.3f ms | OK: %s",
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
```

---

## Spiegazione Dettagliata

### Include necessari

```cpp
#include "my_zero_copy_pkg_40k/msg/fixed_array40k.hpp"  // Messaggio custom POD
```

Usa lo stesso tipo di messaggio del publisher.

### Creazione Subscription

```cpp
subscription_ = this->create_subscription<my_zero_copy_pkg_40k::msg::FixedArray40k>(
    "fixed_array_40k",  // Stesso topic del publisher
    qos,                // Stessa QoS
    std::bind(&LoanedSubscriber::message_callback, this, std::placeholders::_1));
```

### Callback con SharedPtr

```cpp
void message_callback(const my_zero_copy_pkg_40k::msg::FixedArray40k::SharedPtr msg)
```

Con zero-copy attivo, `msg` punta **direttamente alla memoria condivisa** senza copie!

### Calcolo Latenza

```cpp
// Timestamp del messaggio (dal publisher)
int64_t msg_ns = static_cast<int64_t>(msg->timestamp_sec) * 1000000000LL + 
                 static_cast<int64_t>(msg->timestamp_nanosec);

// Timestamp attuale
int64_t now_ns = now.nanoseconds();

// Differenza = latenza
double latency_ms = static_cast<double>(now_ns - msg_ns) / 1e6;
```

La latenza include:
- Tempo di serializzazione (quasi zero con zero-copy)
- Tempo di trasferimento (quasi zero con shared memory)
- Overhead del middleware

### Verifica Integrità Dati

```cpp
bool integrity_ok = (last_val - first_val) == expected_diff;
```

Verifica che i dati siano corretti:
- Con 40.000 elementi, l'ultimo dovrebbe essere sempre 39.999 più del primo
- Se `integrity_ok` è `false`, qualcosa è andato storto

---

## Output Console

```
[INFO] [loaned_subscriber_40k]: Subscriber LOANED 40K avviato. In ascolto su 'fixed_array_40k'.
[INFO] [loaned_subscriber_40k]: Msg #1 | Primo: 0 | Ultimo: 39999 | Latenza: 0.234 ms | OK: SI
[INFO] [loaned_subscriber_40k]: Msg #2 | Primo: 1 | Ultimo: 40000 | Latenza: 0.198 ms | OK: SI
[INFO] [loaned_subscriber_40k]: Msg #3 | Primo: 2 | Ultimo: 40001 | Latenza: 0.215 ms | OK: SI
```

Nota la latenza molto bassa (< 1 ms) per 160 KB di dati!

---

## Zero-Copy: Come Funziona nel Subscriber

```
                    MEMORIA CONDIVISA (/dev/shm)
                    ┌─────────────────────────────┐
                    │  fastrtps_xxxxx             │
                    │  ┌─────────────────────┐    │
Publisher ──────────│──│ timestamp_sec       │────│───────── Subscriber
(scrive)            │  │ timestamp_nanosec   │    │          (legge)
                    │  │ data[0]             │    │
                    │  │ data[1]             │    │
                    │  │ ...                 │    │
                    │  │ data[39999]         │    │
                    │  └─────────────────────┘    │
                    └─────────────────────────────┘
                    
                    NESSUNA COPIA! Stesso indirizzo di memoria!
```

---

## Confronto Latenze

| Metodo | Latenza tipica (160 KB) | Note |
|--------|------------------------|------|
| UDP | 5-50 ms | Dipende dalla rete |
| Standard (copie) | 1-5 ms | 2 copie memoria |
| Zero-copy (SHM) | 0.1-0.5 ms | Nessuna copia |

---

## Best Practices

### 1. Usa ConstSharedPtr quando possibile

```cpp
// Meglio: sola lettura, nessuna copia
void callback(const FixedArray40k::ConstSharedPtr msg)

// Okay: ma potrebbe forzare una copia se modifichi
void callback(const FixedArray40k::SharedPtr msg)
```

### 2. Non copiare il messaggio

```cpp
// MALE: copia i dati!
auto copy = *msg;

// BENE: usa direttamente
int32_t val = msg->data[0];
```

### 3. Processa velocemente

Il buffer condiviso è limitato. Se il subscriber è lento, il publisher potrebbe bloccarsi.

---

## Prossimo Passo

Vai al capitolo [8. Compilazione e Test](08_compilazione_test.md) per compilare ed eseguire il sistema.
