# 6. Publisher Loaned (Zero-Copy) - Dettaglio

Questo capitolo spiega in dettaglio il publisher zero-copy con messaggi loaned.

---

## Panoramica

Il **publisher loaned** usa l'API `borrow_loaned_message()` per scrivere direttamente nella memoria condivisa, eliminando le copie.

**File**: `src/loaned_publisher.cpp`

---

## Concetto chiave: Messaggi "in prestito"

Invece di creare un messaggio locale e copiarlo, il publisher "prende in prestito" un messaggio direttamente dal middleware:

```
Metodo tradizionale:
Publisher crea messaggio → Copia nel middleware → Copia al subscriber

Metodo zero-copy:
Publisher SCRIVE DIRETTAMENTE nel buffer condiviso → Subscriber LEGGE DIRETTAMENTE
```

---

## Codice Completo

```cpp
/**
 * @file loaned_publisher.cpp
 * @brief Publisher Zero-Copy - Pubblica array di 40.000 interi ogni secondo
 * 
 * Publisher con ZERO-COPY usando borrow_loaned_message() API.
 * Array di 40.000 interi (160 KB per messaggio).
 */

#include <chrono>
#include <memory>
#include <utility>

#include "rclcpp/rclcpp.hpp"
#include "my_zero_copy_pkg_40k/msg/fixed_array40k.hpp"

using namespace std::chrono_literals;

// Dimensione array: 40.000 interi
constexpr size_t ARRAY_SIZE = 40000;

/**
 * @class LoanedPublisher
 * @brief Publisher con zero-copy per 40.000 interi
 */
class LoanedPublisher : public rclcpp::Node
{
public:
    LoanedPublisher()
    : Node("loaned_publisher_40k"), count_(0)
    {
        // Disabilita buffering stdout per output immediato
        setvbuf(stdout, NULL, _IONBF, BUFSIZ);
        
        // QoS con history depth 7
        rclcpp::QoS qos(rclcpp::KeepLast(7));
        
        // Crea publisher con messaggio custom
        publisher_ = this->create_publisher<my_zero_copy_pkg_40k::msg::FixedArray40k>(
            "fixed_array_40k", qos);
        
        // Timer per pubblicare ogni secondo
        timer_ = this->create_wall_timer(
            1s, std::bind(&LoanedPublisher::publish_message, this));
        
        // Verifica se zero-copy è disponibile
        if (publisher_->can_loan_messages()) {
            RCLCPP_INFO(this->get_logger(), 
                "Zero-copy ABILITATO per 40.000 interi (160 KB).");
        } else {
            RCLCPP_WARN(this->get_logger(),
                "Zero-copy NON DISPONIBILE - Usando pubblicazione standard.");
        }
        
        RCLCPP_INFO(this->get_logger(), 
            "Publisher LOANED 40K avviato. %zu interi ogni secondo.", ARRAY_SIZE);
    }

private:
    void publish_message()
    {
        // Controlla se possiamo usare loaned messages
        if (!publisher_->can_loan_messages()) {
            // FALLBACK: pubblicazione standard
            auto msg = my_zero_copy_pkg_40k::msg::FixedArray40k();
            
            auto now = this->now();
            msg.timestamp_sec = static_cast<int32_t>(now.seconds());
            msg.timestamp_nanosec = static_cast<uint32_t>(now.nanoseconds() % 1000000000);
            
            for (size_t i = 0; i < ARRAY_SIZE; ++i) {
                msg.data[i] = static_cast<int32_t>(i + count_);
            }
            
            RCLCPP_INFO(this->get_logger(),
                "Msg #%zu (std) | Primo: %d | Ultimo: %d",
                count_, msg.data[0], msg.data[ARRAY_SIZE - 1]);
            
            publisher_->publish(msg);
        } else {
            // ZERO-COPY: prendi messaggio in prestito
            auto loaned_msg = publisher_->borrow_loaned_message();
            
            // Scrivi timestamp
            auto now = this->now();
            loaned_msg.get().timestamp_sec = static_cast<int32_t>(now.seconds());
            loaned_msg.get().timestamp_nanosec = static_cast<uint32_t>(now.nanoseconds() % 1000000000);
            
            // Scrivi dati DIRETTAMENTE nella memoria condivisa
            for (size_t i = 0; i < ARRAY_SIZE; ++i) {
                loaned_msg.get().data[i] = static_cast<int32_t>(i + count_);
            }
            
            RCLCPP_INFO(this->get_logger(),
                "Msg #%zu (zc) | Primo: %d | Ultimo: %d",
                count_, loaned_msg.get().data[0], loaned_msg.get().data[ARRAY_SIZE - 1]);
            
            // Pubblica e restituisci il messaggio
            publisher_->publish(std::move(loaned_msg));
        }
        
        ++count_;
    }
    
    rclcpp::Publisher<my_zero_copy_pkg_40k::msg::FixedArray40k>::SharedPtr publisher_;
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
```

---

## Spiegazione Dettagliata

### Include aggiuntivi

```cpp
#include <utility>  // Per std::move
#include "my_zero_copy_pkg_40k/msg/fixed_array40k.hpp"  // Nostro messaggio custom
```

### Verifica disponibilità zero-copy

```cpp
if (publisher_->can_loan_messages()) {
    RCLCPP_INFO(this->get_logger(), "Zero-copy ABILITATO...");
} else {
    RCLCPP_WARN(this->get_logger(), "Zero-copy NON DISPONIBILE...");
}
```

`can_loan_messages()` ritorna:
- `true` se il middleware supporta messaggi loaned E il messaggio è compatibile
- `false` altrimenti

### Prendere in prestito un messaggio

```cpp
auto loaned_msg = publisher_->borrow_loaned_message();
```

Questa è la **funzione chiave**! Ritorna un `LoanedMessage<FixedArray40k>` che:
- Punta direttamente alla memoria condivisa
- Non crea copie
- Viene "restituito" automaticamente alla pubblicazione

### Accedere ai dati

```cpp
loaned_msg.get().timestamp_sec = 123;
loaned_msg.get().data[0] = 42;
```

Usa `.get()` per accedere al messaggio sottostante. Stai scrivendo **direttamente in memoria condivisa**!

### Pubblicare e restituire

```cpp
publisher_->publish(std::move(loaned_msg));
```

**IMPORTANTE**: Usa `std::move()` per trasferire la proprietà del messaggio al middleware!

Dopo `publish()`:
- Il messaggio è pubblicato
- Il publisher non possiede più il messaggio
- Il subscriber può leggerlo dalla stessa memoria

---

## Differenze con Publisher Standard

| Aspetto | Standard | Loaned (Zero-Copy) |
|---------|----------|-------------------|
| Creazione messaggio | `auto msg = Type()` | `borrow_loaned_message()` |
| Accesso dati | `msg.field` | `loaned_msg.get().field` |
| Pubblicazione | `publish(msg)` | `publish(std::move(loaned_msg))` |
| Copie memoria | 2 (locale → middleware → subscriber) | 0 |
| Tipo messaggio | Qualsiasi | Solo POD a dimensione fissa |

---

## Meccanismo di Fallback

Il codice include un fallback per quando zero-copy non è disponibile:

```cpp
if (!publisher_->can_loan_messages()) {
    // Usa pubblicazione standard
} else {
    // Usa zero-copy
}
```

Questo rende il codice robusto: funziona sempre, ma usa zero-copy quando possibile.

---

## Output Console

### Zero-copy attivo
```
[INFO] [loaned_publisher_40k]: Zero-copy ABILITATO per 40.000 interi (160 KB).
[INFO] [loaned_publisher_40k]: Publisher LOANED 40K avviato. 40000 interi ogni secondo.
[INFO] [loaned_publisher_40k]: Msg #0 (zc) | Primo: 0 | Ultimo: 39999
[INFO] [loaned_publisher_40k]: Msg #1 (zc) | Primo: 1 | Ultimo: 40000
                                      ^^
                                      "zc" = zero-copy!
```

### Zero-copy non disponibile
```
[WARN] [loaned_publisher_40k]: Zero-copy NON DISPONIBILE - Usando pubblicazione standard.
[INFO] [loaned_publisher_40k]: Msg #0 (std) | Primo: 0 | Ultimo: 39999
                                      ^^^
                                      "std" = standard
```

---

## Prossimo Passo

Vai al capitolo [7. Subscriber Loaned (Zero-Copy)](07_subscriber_loaned.md) per completare il sistema zero-copy.
