# 2. Messaggio Custom POD

Questo capitolo spiega come creare un messaggio compatibile con zero-copy.

---

## Cos'è un messaggio POD?

**POD** significa **Plain Old Data** - dati semplici senza costruttori, distruttori o puntatori.

Per il zero-copy, il messaggio DEVE essere POD perché:
- La memoria condivisa richiede dati a dimensione fissa
- Non si possono copiare puntatori tra processi diversi
- FastDDS deve conoscere la dimensione esatta in anticipo

---

## Cosa NON usare

### ❌ std_msgs/Header
```msg
# NON USARE!
std_msgs/Header header
```

Il tipo `Header` contiene un campo `string frame_id` che è a dimensione variabile!

### ❌ Array dinamici
```msg
# NON USARE!
int32[] data    # Lunghezza variabile!
```

### ❌ Stringhe
```msg
# NON USARE!
string nome     # Lunghezza variabile!
```

---

## Cosa usare

### ✅ Tipi numerici
```msg
int32 valore
float64 temperatura
uint8 flag
```

### ✅ Array a lunghezza fissa
```msg
int32[100] dati    # Esattamente 100 interi
float32[50] valori # Esattamente 50 float
```

---

## Il nostro messaggio: FixedArray40k.msg

Crea il file `msg/FixedArray40k.msg`:

```msg
# Messaggio personalizzato per trasferimento dati zero-copy
#
# Array a lunghezza fissa di 40.000 int32 per supporto zero-copy.
# Questo è un messaggio PURO POD (nessuna stringa, nessun tipo dinamico).
#
# Dimensione totale: ~160 KB
#   - timestamp_sec: 4 byte
#   - timestamp_nanosec: 4 byte
#   - data: 40.000 × 4 byte = 160.000 byte

# Timestamp come interi POD (nessun Header per mantenerlo POD puro)
int32 timestamp_sec
uint32 timestamp_nanosec

# Array a lunghezza fissa di 40.000 interi
int32[40000] data
```

### Spiegazione dei campi

| Campo | Tipo | Dimensione | Scopo |
|-------|------|------------|-------|
| `timestamp_sec` | int32 | 4 byte | Secondi del timestamp |
| `timestamp_nanosec` | uint32 | 4 byte | Nanosecondi del timestamp |
| `data` | int32[40000] | 160.000 byte | Array di dati |

**Dimensione totale**: 160.008 byte (~156 KB)

---

## Perché non usiamo std_msgs/Header?

Il messaggio standard `std_msgs/Header` è definito così:

```msg
# std_msgs/Header.msg
builtin_interfaces/Time stamp
string frame_id    # <-- PROBLEMA! Lunghezza variabile!
```

Il campo `frame_id` è una stringa che può avere qualsiasi lunghezza, quindi il messaggio non ha una dimensione fissa e **non è compatibile con zero-copy**.

### Soluzione: Timestamp POD

Invece di usare `Header`, creiamo i nostri campi timestamp:

```msg
# Invece di:
# std_msgs/Header header

# Usiamo:
int32 timestamp_sec
uint32 timestamp_nanosec
```

Nel codice C++, impostiamo questi valori manualmente:

```cpp
auto now = this->now();
msg.timestamp_sec = static_cast<int32_t>(now.seconds());
msg.timestamp_nanosec = static_cast<uint32_t>(now.nanoseconds() % 1000000000);
```

---

## Generazione del codice C++

Quando compili il package, ROS2 genera automaticamente:

1. **Header file**: `my_zero_copy_pkg_40k/msg/fixed_array40k.hpp`
2. **Classe C++**: `my_zero_copy_pkg_40k::msg::FixedArray40k`

### Uso nel codice

```cpp
#include "my_zero_copy_pkg_40k/msg/fixed_array40k.hpp"

// Crea messaggio
auto msg = my_zero_copy_pkg_40k::msg::FixedArray40k();

// Accedi ai campi
msg.timestamp_sec = 123;
msg.timestamp_nanosec = 456789;
msg.data[0] = 42;
msg.data[39999] = 100;
```

---

## Verifica compatibilità zero-copy

Dopo la compilazione, puoi verificare se il messaggio è compatibile:

```cpp
// Nel costruttore del publisher
if (publisher_->can_loan_messages()) {
    std::cout << "Zero-copy disponibile!" << std::endl;
} else {
    std::cout << "Zero-copy NON disponibile" << std::endl;
}
```

Se `can_loan_messages()` ritorna `false`, probabilmente il messaggio non è POD.

---

## Prossimo Passo

Vai al capitolo [3. Configurazione FastDDS](03_configurazione_fastdds.md) per configurare il middleware.
