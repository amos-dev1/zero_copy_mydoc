# Publisher Zero-Copy Documentation
# Documentazione Publisher Zero-Copy

---

## English

### Overview
The `loaned_publisher` node publishes an array of **125 integers** every second using ROS2's zero-copy loaned messages mechanism.

> **Note**: Array size reduced from 40,000 → 10,000 → 125 due to FastCDR buffer memory constraints.

### Key Features
- Uses `borrow_loaned_message()` API for zero-copy allocation
- Publishes to topic: `fixed_array_125`
- Publishing rate: 1 Hz (every second)
- QoS: KeepLast(7)
- Uses `ARRAY_SIZE` constant for easy modification

### How It Works
1. The timer callback fires every 1 second
2. `borrow_loaned_message()` allocates memory in shared memory (if supported by middleware)
3. The array is filled with incremental values: `data[i] = i + message_count`
4. `publish(std::move(loaned_msg))` transfers ownership to FastDDS
5. FastDDS shares the memory buffer directly with subscribers (zero-copy)

### API Reference
```cpp
// Array size constant
constexpr size_t ARRAY_SIZE = 125;

// Borrow a message from middleware
auto loaned_msg = publisher_->borrow_loaned_message();

// Access message data
loaned_msg.get().data[i] = value;

// Publish (transfers ownership)
publisher_->publish(std::move(loaned_msg));
```

### Execution
```bash
ros2 run my_zero_copy_pkg loaned_publisher
```

---

## Italiano

### Panoramica
Il nodo `loaned_publisher` pubblica un array di **125 interi** ogni secondo usando il meccanismo di loaned messages zero-copy di ROS2.

> **Nota**: Dimensione array ridotta da 40.000 → 10.000 → 125 per limiti del buffer FastCDR.

### Caratteristiche Principali
- Usa l'API `borrow_loaned_message()` per allocazione zero-copy
- Pubblica sul topic: `fixed_array_125`
- Frequenza di pubblicazione: 1 Hz (ogni secondo)
- QoS: KeepLast(7)
- Usa costante `ARRAY_SIZE` per facile modifica

### Come Funziona
1. Il callback del timer si attiva ogni secondo
2. `borrow_loaned_message()` alloca memoria nella memoria condivisa (se supportato dal middleware)
3. L'array viene riempito con valori incrementali: `data[i] = i + contatore_messaggi`
4. `publish(std::move(loaned_msg))` trasferisce la proprietà a FastDDS
5. FastDDS condivide il buffer di memoria direttamente con i subscriber (zero-copy)

### Esecuzione
```bash
ros2 run my_zero_copy_pkg loaned_publisher
```
