# FastDDS Zero-Copy Configuration Documentation
# Documentazione Configurazione Zero-Copy FastDDS

---

## Overview / Panoramica

**IT**: Questa documentazione descrive la configurazione FastDDS per abilitare il trasferimento zero-copy di messaggi ROS2 con array di 40.000 interi.

**EN**: This documentation describes the FastDDS configuration to enable zero-copy transfer of ROS2 messages with 40,000 integer arrays.

---

## How Zero-Copy Works / Come Funziona Zero-Copy

### Traditional Publishing (Copy)
```
Publisher → Copy to DDS buffer → Network/IPC → Copy to Subscriber buffer → Subscriber
            ^^^^                              ^^^^
            2 copies of 160KB data!
```

### Zero-Copy Publishing (Data Sharing)
```
Publisher → Shared Memory ← Subscriber
            ^^^^^^^^^^^^
            No copies! Direct memory access
```

---

## Configuration Files / File di Configurazione

### 1. fastdds_setup.xml

**Location**: `config/fastdds_setup.xml`

```xml
<?xml version="1.0" encoding="UTF-8"?>
<dds xmlns="http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles">
    <profiles>
        <!-- Publisher profile with data sharing -->
        <data_writer profile_name="default_publisher" is_default_profile="true">
            <qos>
                <publishMode><kind>SYNCHRONOUS</kind></publishMode>
                <reliability><kind>RELIABLE</kind></reliability>
                <durability><kind>VOLATILE</kind></durability>
                <data_sharing><kind>AUTOMATIC</kind></data_sharing>
            </qos>
            <historyMemoryPolicy>PREALLOCATED_WITH_REALLOC</historyMemoryPolicy>
        </data_writer>

        <!-- Subscriber profile with data sharing -->
        <data_reader profile_name="default_subscriber" is_default_profile="true">
            <qos>
                <reliability><kind>RELIABLE</kind></reliability>
                <durability><kind>VOLATILE</kind></durability>
                <data_sharing><kind>AUTOMATIC</kind></data_sharing>
            </qos>
            <historyMemoryPolicy>PREALLOCATED_WITH_REALLOC</historyMemoryPolicy>
        </data_reader>
    </profiles>
</dds>
```

### Key Elements / Elementi Chiave

| Element | Value | Description |
|---------|-------|-------------|
| `data_sharing.kind` | AUTOMATIC | Enables automatic data sharing when possible |
| `historyMemoryPolicy` | PREALLOCATED_WITH_REALLOC | Allows buffer reallocation for large messages |
| `publishMode.kind` | SYNCHRONOUS | Synchronous publishing mode |
| `reliability.kind` | RELIABLE | Reliable message delivery |

---

## Environment Variables / Variabili Ambiente

### Required / Obbligatorie

```bash
# Enable XML QoS configuration
export RMW_FASTRTPS_USE_QOS_FROM_XML=1

# Path to FastDDS configuration file
export FASTRTPS_DEFAULT_PROFILES_FILE=/path/to/fastdds_setup.xml
```

### Optional / Opzionali

```bash
# Explicitly use FastRTPS middleware
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

# Ensure loaned messages are not disabled
export ROS_DISABLE_LOANED_MESSAGE=0
```

---

## API Usage / Uso API

### Publisher with borrow_loaned_message()

```cpp
// Check if zero-copy is available
if (publisher_->can_loan_messages()) {
    // Zero-copy: borrow message from middleware
    auto loaned_msg = publisher_->borrow_loaned_message();
    
    // Fill data directly in shared memory
    loaned_msg.get().data[0] = value;
    
    // Publish (transfers ownership)
    publisher_->publish(std::move(loaned_msg));
} else {
    // Fallback: standard publishing
    auto msg = MessageType();
    publisher_->publish(msg);
}
```

---

## Message Requirements / Requisiti Messaggio

For zero-copy to work, messages must be:

1. **Fixed-size (bounded)** - No dynamic arrays or strings
2. **POD (Plain Old Data)** - No std_msgs/Header (contains string)
3. **Pure numeric types** - int32, float64, etc.

### Valid Zero-Copy Message
```msg
int32 timestamp_sec
uint32 timestamp_nanosec
int32[40000] data
```

### Invalid (Non-POD)
```msg
std_msgs/Header header  # Contains string frame_id!
int32[] data            # Unbounded array!
```

---

## Verification / Verifica

### Publisher Output Check
```
[IT] Zero-copy ABILITATO ...     Working
[IT] Msg #0 (zc) | ...           Using zero-copy

[IT] Zero-copy NON DISPONIBILE   Not working
[IT] Msg #0 (std) | ...          Standard publishing
```

---

## Common Issues / Problemi Comuni

See `troubleshooting.md` for detailed error solutions.
