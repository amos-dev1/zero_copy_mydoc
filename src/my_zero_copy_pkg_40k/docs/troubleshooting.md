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

## Package Info / Info Pacchetto

| Property | Value |
|----------|-------|
| **Package Name** | my_zero_copy_pkg_40k |
| **Array Size** | 40,000 integers |
| **Message Size** | ~160 KB |
| **Base Package** | my_zero_copy_pkg |

---

## ✅ SOLVED: NotEnoughMemoryException (Terminal Issue)

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

**IT**: Aprire terminali separati invece di usare schede multiple nello stesso terminale.

**EN**: Open separate terminals instead of using multiple tabs in the same terminal.

---

## ✅ SOLVED: SHM IS Working! (FastDDS Monitor Shows UDPv4 for Discovery)

### What We Found / Cosa Abbiamo Trovato

> [!IMPORTANT]
> **SHM sta funzionando!** I file in `/dev/shm` lo confermano!
> 
> FastDDS Monitor mostra UDPv4 perché quello è usato per la **discovery** (trovare altri participant).
> I **dati effettivi** vengono trasferiti via **Shared Memory** (SHM).

### Evidence / Evidenza

```bash
ls -la /dev/shm | grep fastrtps
# Shows many files like:
# fastrtps_361c755af764764b (data segments)
# fastrtps_port7411 (communication ports)
# sem.fastrtps_port7411_mutex (synchronization)
```

### Explanation / Spiegazione

| Component | Transport | Purpose |
|-----------|-----------|---------|
| **Discovery** | UDPv4 multicast | Finding other DDS participants |
| **Data Transfer** | SHM (Shared Memory) | Actual message data |

FastDDS Monitor shows the discovery locators (UDPv4), but data goes through shared memory segments in `/dev/shm`.

---

| # | Attempt | XML Config | Result |
|---|---------|------------|--------|
| 1 | Simple data_sharing | `<data_writer>` with `AUTOMATIC` | ❌ Still UDPv4 |
| 2 | Add transport_descriptors | `<transport_descriptors>` outside profiles | ❌ XML parser error |
| 3 | Move transport_descriptors | Inside `<profiles>` | ❌ Still error |
| 4 | Add shared_memory_directory | In `<data_sharing>` | ❌ Invalid element error |
| 5 | LARGE_DATA builtin | `<builtinTransports>LARGE_DATA` | ❌ Still UDPv4 |
| 6 | Copy from reference repo | Use `<publisher>`/`<subscriber>` tags | ❌ Still UDPv4 |
| 7 | **Force data_sharing ON** | `<kind>ON</kind>` instead of AUTOMATIC | ⏳ Testing... |

> **Note**: User has duplicate `FASTRTPS_DEFAULT_PROFILES_FILE` in `.bashrc` - second value overrides first.

### Reference Comparison / Confronto con Riferimento

**Reference repo**: [leonardonels/zero_copy](https://github.com/leonardonels/zero_copy/blob/main/loaned_messages/fastDDS_setup.xml)

| Element | Our Early Code | Reference Code |
|---------|----------------|----------------|
| Tag names | `<data_writer>`, `<data_reader>` | `<publisher>`, `<subscriber>` |
| Profile names | `"default_publisher"` | `"default publisher profile"` |
| Participant | Complex config | None |
| transport_descriptors | Various attempts | None |

---

## ❌ FIXED: Previous XML Parser Errors

### Error 1: transport_descriptors
```
Not expected tag: 'transport_descriptors' -> Function parseXML
```
**Fix**: Removed transport_descriptors entirely

### Error 2: shared_memory_directory
```
Invalid element found in 'data_sharing'. Name: shared_memory_directory
```
**Fix**: Removed shared_memory_directory from data_sharing

### Error 3: STATISTICS_DOMAIN_PARTICIPANT (Harmless)
```
Topic MONITOR_SERVICE_TOPIC is not a valid statistics topic name/alias
```
**Note**: This is a harmless warning about FastDDS Monitor, does not affect functionality

---

## Previous Debug Attempts (For NotEnoughMemoryException)

During debugging terminal issue, we tried things that weren't the problem:

| Attempt | Description | Necessary? |
|---------|-------------|------------|
| Reduce array size | 40k → 10k → 125 | ❌ No |
| Change data_sharing | AUTOMATIC → OFF | ❌ No |
| Remove std_msgs/Header | Replace with POD fields | ✅ Yes (for zero-copy) |
| Remove loaned API | Use standard publishing | ❌ No |

---

## Modifications from Base Package / Modifiche dal Pacchetto Base

| File | Line | Change |
|------|------|--------|
| `package.xml` | 3 | `name` → `my_zero_copy_pkg_40k` |
| `CMakeLists.txt` | 2 | `project(my_zero_copy_pkg_40k)` |
| `msg/FixedArray40k.msg` | 17 | `int32[125]` → `int32[40000]` |
| `src/loaned_publisher.cpp` | 21 | include → `my_zero_copy_pkg_40k` |
| `src/loaned_publisher.cpp` | 27 | `ARRAY_SIZE = 40000` |
| `src/loaned_publisher.cpp` | 45 | topic → `fixed_array_40k` |
| `src/loaned_subscriber.cpp` | 17 | include → `my_zero_copy_pkg_40k` |
| `src/loaned_subscriber.cpp` | 21 | `ARRAY_SIZE = 40000` |
| `src/loaned_subscriber.cpp` | 39 | topic → `fixed_array_40k` |
| `src/standard_publisher.cpp` | 23 | `ARRAY_SIZE = 40000` |
| `src/standard_publisher.cpp` | 37 | topic → `standard_array_40k` |
| `src/standard_subscriber.cpp` | 30 | topic → `standard_array_40k` |

---

## Build & Test Commands / Comandi Build e Test

```bash
# Build 40K package
colcon build --packages-select my_zero_copy_pkg_40k

# Source workspace
source install/setup.bash

# Environment variables (REQUIRED for zero-copy!)
export RMW_FASTRTPS_USE_QOS_FROM_XML=1
export ROS_DISABLE_LOANED_MESSAGE=0
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_DOMAIN_ID=0
export FASTRTPS_DEFAULT_PROFILES_FILE=$(ros2 pkg prefix my_zero_copy_pkg_40k)/share/my_zero_copy_pkg_40k/config/fastdds_setup.xml

# Test STANDARD (no zero-copy) - IN SEPARATE TERMINALS!
ros2 run my_zero_copy_pkg_40k standard_subscriber  # Terminal 1
ros2 run my_zero_copy_pkg_40k standard_publisher   # Terminal 2

# Test LOANED (zero-copy) - IN SEPARATE TERMINALS!
ros2 run my_zero_copy_pkg_40k loaned_subscriber    # Terminal 1
ros2 run my_zero_copy_pkg_40k loaned_publisher     # Terminal 2
```

---

## Files in Package / File nel Pacchetto

| File | Description |
|------|-------------|
| `src/standard_publisher.cpp` | Publisher standard con std_msgs (WORKING ✅) |
| `src/standard_subscriber.cpp` | Subscriber standard con std_msgs (WORKING ✅) |
| `src/loaned_publisher.cpp` | Publisher con borrow_loaned_message() |
| `src/loaned_subscriber.cpp` | Subscriber per messaggi loaned |
| `msg/FixedArray40k.msg` | Messaggio custom POD (40K interi) |
| `config/fastdds_setup.xml` | Configurazione FastDDS |

---

## Verification Checklist / Lista di Verifica

- [x] No XML parser errors at startup
- [x] Publisher shows "(zc)" in messages
- [ ] FastDDS Monitor shows **SHM** locators
- [ ] Performance improvement visible

---

## How to Verify SHM is Working / Come Verificare SHM

### Method 1: Check /dev/shm
FastDDS creates shared memory segments in `/dev/shm`. Run while nodes are active:
```bash
ls -la /dev/shm | grep fastrtps
```
**If SHM is working**: You should see files like `fastrtps_*` and `fastrtps_port*`
**If SHM is NOT working**: No fastrtps files appear

### Method 2: FastDDS Monitor
- Check "Physical Data" tab
- Look for **SHM** locators (not just UDPv4)

---

## Known Issues / Problemi Noti

1. **Terminal tabs** - Use separate terminal windows, not tabs!
2. **Environment variables** - Must be set in EACH terminal before running
3. **SHM not showing** - Currently investigating...
