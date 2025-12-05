# Zero-Copy ROS2 con FastDDS

Documentazione completa per tenere traccia di come ho implementato la comunicazione zero-copy in ROS2 usando FastDDS e messaggi loaned.

---

## Indice

1. [Creazione Package](01_creazione_package.md)
2. [Messaggio Custom POD](02_messaggio_custom.md)
3. [Configurazione FastDDS](03_configurazione_fastdds.md)
4. [Publisher Standard](04_publisher_standard.md)
5. [Subscriber Standard](05_subscriber_standard.md)
6. [Publisher Loaned (Zero-Copy)](06_publisher_loaned.md)
7. [Subscriber Loaned (Zero-Copy)](07_subscriber_loaned.md)
8. [Compilazione e Test](08_compilazione_test.md)
9. [Verifica Funzionamento SHM](09_verifica_shm.md)

---

## Cos'è Zero-Copy?

La comunicazione **zero-copy** elimina le copie di dati tra publisher e subscriber. Invece di copiare i dati nella memoria del middleware e poi nella memoria del subscriber, i dati vengono scritti direttamente in una zona di memoria condivisa (Shared Memory - SHM).

### Comunicazione Tradizionale (con copie)
```
Publisher → Copia in buffer DDS → Rete/IPC → Copia in buffer Subscriber → Subscriber
            ^^^^                              ^^^^
            2 copie dei dati (160 KB ciascuna per array 40K!)
```

### Comunicazione Zero-Copy
```
Publisher → Memoria Condivisa ← Subscriber
            ^^^^^^^^^^^^^^^^
            Nessuna copia! Accesso diretto alla stessa memoria
```

---

## Prerequisiti

| Requisito | Versione |
|-----------|----------|
| **Sistema Operativo** | Ubuntu 22.04 |
| **ROS2** | Humble Hawksbill |
| **Middleware** | FastDDS (rmw_fastrtps_cpp) |
| **Compilatore** | GCC con supporto C++17 |

### Verifica installazione

```bash
# Verifica ROS2
ros2 --version

# Verifica middleware
echo $RMW_IMPLEMENTATION
# Deve mostrare: rmw_fastrtps_cpp
```

---

## Panoramica del Package

Questo tutorial crea un package chiamato `my_zero_copy_pkg_40k` che contiene:

| File | Descrizione |
|------|-------------|
| `msg/FixedArray40k.msg` | Messaggio con 40.000 interi (160 KB) |
| `src/standard_publisher.cpp` | Publisher tradizionale |
| `src/standard_subscriber.cpp` | Subscriber tradizionale |
| `src/loaned_publisher.cpp` | Publisher zero-copy |
| `src/loaned_subscriber.cpp` | Subscriber zero-copy |
| `config/fastdds_setup.xml` | Configurazione FastDDS |

---

## Comandi Rapidi

Per chi vuole solo eseguire senza leggere tutti i dettagli:

```bash
# 1. Compila
cd ~/ros2_ws
colcon build --packages-select my_zero_copy_pkg_40k
source install/setup.bash

# 2. Imposta variabili ambiente (aggiungi a ~/.bashrc)
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export RMW_FASTRTPS_USE_QOS_FROM_XML=1
export ROS_DISABLE_LOANED_MESSAGE=0
export ROS_DOMAIN_ID=0
export FASTRTPS_DEFAULT_PROFILES_FILE=$(ros2 pkg prefix my_zero_copy_pkg_40k)/share/my_zero_copy_pkg_40k/config/fastdds_setup.xml

# 3. Esegui (in terminali SEPARATI!)
ros2 run my_zero_copy_pkg_40k loaned_subscriber
ros2 run my_zero_copy_pkg_40k loaned_publisher

# 4. Verifica SHM funzionante
ls -la /dev/shm | grep fastrtps
```

---

## Prossimo Passo

Inizia dal capitolo [1. Creazione Package](01_creazione_package.md).
