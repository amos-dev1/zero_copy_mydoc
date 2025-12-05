# Subscriber Zero-Copy Documentation
# Documentazione Subscriber Zero-Copy

---

## English

### Overview
The `loaned_subscriber` node receives arrays of **125 integers** published by the `loaned_publisher` using zero-copy shared memory transport.

> **Note**: Array size reduced from 40,000 → 10,000 → 125 due to FastCDR buffer memory constraints.

### Key Features
- Subscribes to topic: `fixed_array_125`
- Calculates and displays message latency
- Verifies data integrity on each received message
- QoS: KeepLast(7)
- Uses `ARRAY_SIZE` constant for easy modification

### How It Works
1. FastDDS receives pointer to shared memory (no copy)
2. Callback receives `SharedPtr` to message data
3. Latency is calculated from header timestamp
4. Data integrity is verified: `last_element - first_element == ARRAY_SIZE - 1`

### Callback Function
```cpp
constexpr size_t ARRAY_SIZE = 125;

void message_callback(const FixedArray40k::SharedPtr msg)
{
    // Calculate latency
    auto latency = (now() - msg->header.stamp).nanoseconds() / 1e6;
    
    // Verify integrity
    bool ok = (msg->data[ARRAY_SIZE - 1] - msg->data[0]) == (ARRAY_SIZE - 1);
    
    // Process data...
}
```

### Output Example
```
[IT] Msg #42 ricevuto | Primo: 42 | Ultimo: 166 | Latenza: 0.123 ms | Integrità: OK
[EN] Msg #42 received | First: 42 | Last: 166 | Latency: 0.123 ms | Integrity: OK
```

### Execution
```bash
ros2 run my_zero_copy_pkg loaned_subscriber
```

---

## Italiano

### Panoramica
Il nodo `loaned_subscriber` riceve array di **125 interi** pubblicati dal `loaned_publisher` usando il trasporto zero-copy in memoria condivisa.

> **Nota**: Dimensione array ridotta da 40.000 → 10.000 → 125 per limiti del buffer FastCDR.

### Caratteristiche Principali
- Si sottoscrive al topic: `fixed_array_125`
- Calcola e visualizza la latenza dei messaggi
- Verifica l'integrità dei dati su ogni messaggio ricevuto
- QoS: KeepLast(7)
- Usa costante `ARRAY_SIZE` per facile modifica

### Metriche Monitorate
- **Latenza**: tempo tra pubblicazione e ricezione
- **Integrità**: verifica che i dati siano arrivati correttamente
- **Conteggio**: numero totale di messaggi ricevuti

### Esecuzione
```bash
ros2 run my_zero_copy_pkg loaned_subscriber
```
