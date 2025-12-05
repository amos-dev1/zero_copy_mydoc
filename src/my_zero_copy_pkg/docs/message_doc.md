# FixedArray40k Message Documentation
# Documentazione Messaggio FixedArray40k

---

## English

### Overview
`FixedArray40k.msg` is a custom ROS2 message designed for **zero-copy** data transfer of integer arrays.

> **Note**: Array size reduced from 40,000 → 10,000 → 125 due to FastCDR buffer memory constraints (NotEnoughMemoryException).

### Message Definition
```
std_msgs/Header header
int32[125] data
```

### Fields
| Field | Type | Description |
|-------|------|-------------|
| `header` | std_msgs/Header | Timestamp and frame ID |
| `data` | int32[125] | Fixed-size array of 125 integers |

### Why Fixed Size?
**Zero-copy requires fixed-size (POD) types.**

- Dynamic arrays (like `int32[]`) require heap allocation
- Fixed arrays are allocated in contiguous memory
- FastDDS can share the buffer directly via shared memory

### Memory Size
- Array: 125 × 4 bytes = **500 bytes**
- With header: ~500 bytes total per message

### Usage
```cpp
#include "my_zero_copy_pkg/msg/fixed_array40k.hpp"

// Access data
msg->data[0] = 42;
msg->data[124] = 100;
```

### How to Change Size
1. Edit `msg/FixedArray40k.msg`: change `int32[125]` to desired size
2. Edit `src/loaned_publisher.cpp`: update `ARRAY_SIZE` constant
3. Edit `src/loaned_subscriber.cpp`: update `ARRAY_SIZE` constant
4. Rebuild package

---

## Italiano

### Panoramica
`FixedArray40k.msg` è un messaggio ROS2 personalizzato progettato per il trasferimento **zero-copy** di array di interi.

> **Nota**: Dimensione array ridotta da 40.000 → 10.000 → 125 per limiti del buffer FastCDR (NotEnoughMemoryException).

### Definizione del Messaggio
```
std_msgs/Header header
int32[125] data
```

### Campi
| Campo | Tipo | Descrizione |
|-------|------|-------------|
| `header` | std_msgs/Header | Timestamp e frame ID |
| `data` | int32[125] | Array a dimensione fissa di 125 interi |

### Dimensione in Memoria
- Array: 125 × 4 byte = **500 byte**
- Con header: ~500 byte totali per messaggio

### Come Modificare la Dimensione
1. Modifica `msg/FixedArray40k.msg`: cambia `int32[125]` alla dimensione desiderata
2. Modifica `src/loaned_publisher.cpp`: aggiorna costante `ARRAY_SIZE`
3. Modifica `src/loaned_subscriber.cpp`: aggiorna costante `ARRAY_SIZE`
4. Ricompila il package
