# 5. Subscriber Standard (Dettaglio)

Questo capitolo spiega in dettaglio il subscriber standard senza zero-copy.

---

## Panoramica

Il **subscriber standard** riceve messaggi `std_msgs::msg::Int32MultiArray` e li processa. Questo è il metodo tradizionale.

**File**: `src/standard_subscriber.cpp`

---

## Codice Completo

```cpp
/**
 * @file standard_subscriber.cpp
 * @brief Subscriber Standard - Riceve array di 40.000 interi
 * 
 * Subscriber STANDARD senza zero-copy. Usa std_msgs con 40.000 interi.
 */

#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32_multi_array.hpp"

/**
 * @class StandardSubscriber
 * @brief Subscriber standard per 40.000 interi
 */
class StandardSubscriber : public rclcpp::Node
{
public:
    StandardSubscriber()
    : Node("standard_subscriber_40k"), msg_count_(0)
    {
        // Crea la subscription
        subscription_ = this->create_subscription<std_msgs::msg::Int32MultiArray>(
            "standard_array_40k",  // Nome topic (deve corrispondere al publisher)
            10,                    // QoS depth
            std::bind(&StandardSubscriber::message_callback, this, std::placeholders::_1));
        
        RCLCPP_INFO(this->get_logger(), 
            "Subscriber STANDARD 40K avviato. In ascolto su 'standard_array_40k'.");
    }

private:
    void message_callback(const std_msgs::msg::Int32MultiArray::SharedPtr msg)
    {
        ++msg_count_;
        
        // Ottieni dimensione e valori
        size_t size = msg->data.size();
        int32_t first = size > 0 ? msg->data[0] : 0;
        int32_t last = size > 0 ? msg->data[size - 1] : 0;
        
        RCLCPP_INFO(this->get_logger(),
            "Msg #%zu | Primo: %d | Ultimo: %d | Dimensione: %zu",
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
```

---

## Spiegazione Riga per Riga

### Include necessari

```cpp
#include <memory>    // Per std::shared_ptr

#include "rclcpp/rclcpp.hpp"              // Libreria ROS2 C++
#include "std_msgs/msg/int32_multi_array.hpp"  // Tipo messaggio standard
```

### Classe StandardSubscriber

```cpp
class StandardSubscriber : public rclcpp::Node
```
Eredita da `rclcpp::Node`.

### Costruttore

```cpp
StandardSubscriber()
: Node("standard_subscriber_40k"), msg_count_(0)
{
```
Inizializza il nodo con nome `"standard_subscriber_40k"` e contatore a 0.

### Creazione Subscription

```cpp
subscription_ = this->create_subscription<std_msgs::msg::Int32MultiArray>(
    "standard_array_40k",  // Nome topic
    10,                    // QoS depth
    std::bind(&StandardSubscriber::message_callback, this, std::placeholders::_1));
```

| Parametro | Valore | Descrizione |
|-----------|--------|-------------|
| Tipo messaggio | `Int32MultiArray` | Deve corrispondere al publisher |
| Nome topic | `"standard_array_40k"` | Deve corrispondere al publisher |
| QoS depth | `10` | Massimo 10 messaggi in coda |
| Callback | `message_callback` | Funzione chiamata per ogni messaggio |

**Nota**: `std::placeholders::_1` indica che la callback riceverà un parametro (il messaggio).

### Callback function

```cpp
void message_callback(const std_msgs::msg::Int32MultiArray::SharedPtr msg)
```

La callback riceve un `SharedPtr` al messaggio. Questo è un puntatore intelligente che gestisce automaticamente la memoria.

```cpp
    ++msg_count_;
```
Incrementa il contatore dei messaggi ricevuti.

```cpp
    size_t size = msg->data.size();
    int32_t first = size > 0 ? msg->data[0] : 0;
    int32_t last = size > 0 ? msg->data[size - 1] : 0;
```
Accede ai dati del messaggio:
- `msg->data.size()` - Numero di elementi nell'array
- `msg->data[0]` - Primo elemento
- `msg->data[size - 1]` - Ultimo elemento

### Logging

```cpp
    RCLCPP_INFO(this->get_logger(),
        "Msg #%zu | Primo: %d | Ultimo: %d | Dimensione: %zu",
        msg_count_, first, last, size);
```

Stampa informazioni sul messaggio ricevuto.

---

## Flusso di esecuzione

```
1. main() chiama rclcpp::init()
2. Crea StandardSubscriber
3. Costruttore:
   - Crea subscription
   - Si registra al topic "standard_array_40k"
4. spin() entra nel loop degli eventi
5. Quando arriva un messaggio:
   - middleware COPIA i dati nel buffer locale
   - message_callback() viene chiamata
   - Callback processa il messaggio
6. Ctrl+C → shutdown()
```

---

## SharedPtr vs ConstSharedPtr

La callback può ricevere il messaggio in due modi:

### Modo 1: SharedPtr (modificabile)
```cpp
void callback(const std_msgs::msg::Int32MultiArray::SharedPtr msg)
{
    msg->data[0] = 999;  // Posso modificare!
}
```

### Modo 2: ConstSharedPtr (sola lettura)
```cpp
void callback(const std_msgs::msg::Int32MultiArray::ConstSharedPtr msg)
{
    // msg->data[0] = 999;  // ERRORE! Non posso modificare
    int x = msg->data[0];  // Posso solo leggere
}
```

Per zero-copy, usa `ConstSharedPtr` quando possibile per evitare copie non necessarie.

---

## Problema: Copie multiple

Anche il subscriber riceve una copia dei dati:

```
Middleware buffer → Copia → Messaggio locale nella callback
                    ^^^^^^
                    160 KB copiati!
```

---

## Test Publisher + Subscriber

Per testare:

```bash
# Terminale 1
ros2 run my_zero_copy_pkg_40k standard_subscriber

# Terminale 2
ros2 run my_zero_copy_pkg_40k standard_publisher
```

Output atteso (subscriber):
```
[INFO] [standard_subscriber_40k]: Subscriber STANDARD 40K avviato...
[INFO] [standard_subscriber_40k]: Msg #1 | Primo: 0 | Ultimo: 39999 | Dimensione: 40000
[INFO] [standard_subscriber_40k]: Msg #2 | Primo: 1 | Ultimo: 40000 | Dimensione: 40000
```

---

## Prossimo Passo

Vai al capitolo [6. Publisher Loaned (Zero-Copy)](06_publisher_loaned.md) per vedere come eliminare le copie!
