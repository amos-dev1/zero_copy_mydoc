# 4. Publisher Standard (Dettaglio)

Questo capitolo spiega in dettaglio il publisher standard senza zero-copy.

---

## Panoramica

Il **publisher standard** usa `std_msgs::msg::Int32MultiArray` per inviare un array di interi. Questo è il metodo tradizionale che copia i dati.

**File**: `src/standard_publisher.cpp`

---

## Codice Completo

```cpp
/**
 * @file standard_publisher.cpp
 * @brief Publisher Standard - Pubblica array di 40.000 interi
 * 
 * Publisher STANDARD senza zero-copy. Usa std_msgs con 40.000 interi.
 */

#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32_multi_array.hpp"

using namespace std::chrono_literals;

// Dimensione dell'array: 40.000 interi
constexpr size_t ARRAY_SIZE = 40000;

/**
 * @class StandardPublisher
 * @brief Publisher standard per 40.000 interi
 */
class StandardPublisher : public rclcpp::Node
{
public:
    StandardPublisher()
    : Node("standard_publisher_40k"), count_(0)
    {
        // Crea il publisher
        publisher_ = this->create_publisher<std_msgs::msg::Int32MultiArray>(
            "standard_array_40k", 10);
        
        // Crea il timer per pubblicare ogni secondo
        timer_ = this->create_wall_timer(
            1s, std::bind(&StandardPublisher::publish_message, this));
        
        RCLCPP_INFO(this->get_logger(), 
            "Publisher STANDARD 40K avviato. %zu interi ogni secondo.", ARRAY_SIZE);
    }

private:
    void publish_message()
    {
        // Crea il messaggio
        auto msg = std_msgs::msg::Int32MultiArray();
        
        // Ridimensiona l'array
        msg.data.resize(ARRAY_SIZE);
        
        // Riempi l'array con dati
        for (size_t i = 0; i < ARRAY_SIZE; ++i) {
            msg.data[i] = static_cast<int32_t>(i + count_);
        }
        
        RCLCPP_INFO(this->get_logger(),
            "Msg #%zu | Primo: %d | Ultimo: %d",
            count_, msg.data[0], msg.data[ARRAY_SIZE - 1]);
        
        // Pubblica il messaggio (COPIA i dati)
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
```

---

## Spiegazione Riga per Riga

### Include necessari

```cpp
#include <chrono>    // Per std::chrono_literals (1s, 500ms, etc.)
#include <memory>    // Per std::shared_ptr

#include "rclcpp/rclcpp.hpp"              // Libreria ROS2 C++
#include "std_msgs/msg/int32_multi_array.hpp"  // Tipo messaggio standard
```

### Costanti

```cpp
using namespace std::chrono_literals;  // Permette di scrivere 1s invece di std::chrono::seconds(1)

constexpr size_t ARRAY_SIZE = 40000;   // Dimensione array: 40.000 elementi
```

### Classe StandardPublisher

```cpp
class StandardPublisher : public rclcpp::Node
```
Eredita da `rclcpp::Node`, la classe base per tutti i nodi ROS2.

### Costruttore

```cpp
StandardPublisher()
: Node("standard_publisher_40k"), count_(0)  // Nome nodo e inizializzazione contatore
{
```

### Creazione Publisher

```cpp
publisher_ = this->create_publisher<std_msgs::msg::Int32MultiArray>(
    "standard_array_40k",  // Nome del topic
    10);                   // Dimensione coda
```

| Parametro | Valore | Descrizione |
|-----------|--------|-------------|
| Tipo messaggio | `Int32MultiArray` | Array di interi a dimensione variabile |
| Nome topic | `"standard_array_40k"` | Nome univoco per questo topic |
| QoS depth | `10` | Massimo 10 messaggi in coda |

### Creazione Timer

```cpp
timer_ = this->create_wall_timer(
    1s,  // Intervallo: 1 secondo
    std::bind(&StandardPublisher::publish_message, this));  // Callback
```

`std::bind` collega la funzione `publish_message` a questo oggetto.

### Funzione di pubblicazione

```cpp
void publish_message()
{
    auto msg = std_msgs::msg::Int32MultiArray();
```
Crea un nuovo messaggio. Questo alloca memoria sullo stack.

```cpp
    msg.data.resize(ARRAY_SIZE);
```
Ridimensiona il vettore interno. Questa operazione alloca 160 KB di memoria heap.

```cpp
    for (size_t i = 0; i < ARRAY_SIZE; ++i) {
        msg.data[i] = static_cast<int32_t>(i + count_);
    }
```
Riempie l'array con valori. Ogni messaggio ha valori diversi grazie a `count_`.

```cpp
    publisher_->publish(msg);
```
Pubblica il messaggio. **Questo COPIA i 160 KB nel buffer del middleware!**

### Main function

```cpp
int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);        // Inizializza ROS2
    rclcpp::spin(std::make_shared<StandardPublisher>());  // Esegui il nodo
    rclcpp::shutdown();              // Chiudi ROS2
    return 0;
}
```

---

## Flusso di esecuzione

```
1. main() chiama rclcpp::init()
2. Crea StandardPublisher
3. Costruttore:
   - Crea publisher
   - Crea timer (1 secondo)
4. spin() entra nel loop degli eventi
5. Ogni secondo:
   - Timer scatta
   - publish_message() chiamata
   - Messaggio creato e riempito
   - Messaggio COPIATO nel middleware
   - Middleware invia al subscriber
6. Ctrl+C → shutdown()
```

---

## Problema: Copie multiple

Con il publisher standard, i dati vengono copiati più volte:

```
Array locale → Copia 1 → Buffer middleware → Copia 2 → Buffer subscriber
              ^^^^^^^^                      ^^^^^^^^
              160 KB                        160 KB
```

Per messaggi grandi come il nostro (160 KB), questo spreco di memoria e CPU diventa significativo!

---

## Prossimo Passo

Vai al capitolo [5. Subscriber Standard](05_subscriber_standard.md) per creare il subscriber corrispondente.
