# Troubleshooting and Known Errors
# Risoluzione Problemi ed Errori Noti

---

## Test Environment / Ambiente di Test

| Specification | Value |
|---------------|-------|
| **OS** | Ubuntu 22.04 |
| **RAM** | 16 GB |
| **Device** | Laptop |
| **ROS2 Version** | Humble (compiled from source) |
| **Middleware** | FastDDS |

---

## ✅ SOLVED: NotEnoughMemoryException

### Root Cause Found / Causa Trovata

> [!IMPORTANT]
> **IT**: Il problema NON era nel codice! Era un problema del terminale. La seconda scheda del terminale non funzionava correttamente.
> 
> **EN**: The problem was NOT in the code! It was a terminal issue. The second terminal tab wasn't working properly.

### How We Discovered It / Come L'abbiamo Scoperto

1. Prima il subscriber funzionava ma il publisher (nella seconda scheda) dava errore
2. Poi abbiamo invertito: il publisher funzionava ma il subscriber (nella seconda scheda) dava errore
3. Questo ha rivelato che il problema era la seconda scheda del terminale, non il codice

### Solution / Soluzione

**IT**: Aprire terminali separati invece di usare schede multiple nello stesso terminale, oppure verificare che tutte le schede funzionino correttamente.

**EN**: Open separate terminals instead of using multiple tabs in the same terminal, or verify that all tabs are working correctly.

---

## Previous Debug Attempts (Not the Real Issue)

During debugging, we tried many things that weren't actually needed:

| Attempt | Description | Necessary? |
|---------|-------------|------------|
| Reduce array size | 40k → 10k → 125 | ❌ No |
| Change data_sharing | AUTOMATIC → OFF | ❌ No |
| Remove std_msgs/Header | Replace with POD fields | ❌ No |
| Remove loaned API | Use standard publishing | ❌ No |

---

## Working Commands / Comandi Funzionanti

```bash
# Build
colcon build --packages-select my_zero_copy_pkg

# IN SEPARATE TERMINALS (not tabs!)
# Terminal 1 - Subscriber
ros2 run my_zero_copy_pkg standard_subscriber

# Terminal 2 - Publisher  
ros2 run my_zero_copy_pkg standard_publisher
```

---

## Files in Package / File nel Pacchetto

| File | Description |
|------|-------------|
| `src/standard_publisher.cpp` | Publisher standard con std_msgs (WORKING ✅) |
| `src/standard_subscriber.cpp` | Subscriber standard con std_msgs (WORKING ✅) |
| `src/loaned_publisher.cpp` | Publisher con borrow_loaned_message() |
| `src/loaned_subscriber.cpp` | Subscriber per messaggi loaned |
| `msg/FixedArray40k.msg` | Messaggio custom POD |
| `config/fastdds_setup.xml` | Configurazione FastDDS |
