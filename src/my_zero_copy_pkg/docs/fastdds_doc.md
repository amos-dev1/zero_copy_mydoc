# FastDDS Configuration Documentation
# Documentazione Configurazione FastDDS

---

## English

### Overview
`fastdds_setup.xml` configures FastDDS middleware to enable **zero-copy data sharing** using shared memory transport.

### Key Configuration Elements

#### Data Sharing
```xml
<data_sharing>
    <kind>AUTOMATIC</kind>
</data_sharing>
```
- `AUTOMATIC`: FastDDS decides when to use shared memory
- `ON`: Always use data sharing (fails if not possible)
- `OFF`: Disable data sharing

#### Publish Mode
```xml
<publishMode>
    <kind>SYNCHRONOUS</kind>
</publishMode>
```
- `SYNCHRONOUS`: Message sent before `publish()` returns
- `ASYNCHRONOUS`: Message queued for later sending

#### History Memory Policy
```xml
<historyMemoryPolicy>PREALLOCATED_WITH_REALLOC</historyMemoryPolicy>
```
- Pre-allocates memory for message history
- Reallocates if needed for larger messages

### Environment Variables
```bash
# Use QoS from XML configuration
export RMW_FASTRTPS_USE_QOS_FROM_XML=1

# Enable loaned messages
export ROS_DISABLE_LOANED_MESSAGE=0

# Use FastRTPS implementation
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

# Path to XML configuration
export FASTRTPS_DEFAULT_PROFILES_FILE=/full/path/to/fastdds_setup.xml
```

---

## Italiano

### Panoramica
`fastdds_setup.xml` configura il middleware FastDDS per abilitare il **data sharing zero-copy** usando il trasporto in memoria condivisa.

### Elementi Chiave di Configurazione

#### Data Sharing
```xml
<data_sharing>
    <kind>AUTOMATIC</kind>
</data_sharing>
```
- `AUTOMATIC`: FastDDS decide quando usare la memoria condivisa
- `ON`: Usa sempre il data sharing (fallisce se non possibile)
- `OFF`: Disabilita il data sharing

#### Modalità di Pubblicazione
- `SYNCHRONOUS`: Il messaggio viene inviato prima che `publish()` ritorni
- `ASYNCHRONOUS`: Il messaggio viene accodato per l'invio successivo

#### Politica Memoria Storico
- Pre-alloca la memoria per lo storico dei messaggi
- Rialloca se necessario per messaggi più grandi

### Come Funziona lo Zero-Copy
1. Publisher alloca il messaggio in memoria condivisa
2. FastDDS passa solo il puntatore al subscriber
3. Subscriber legge direttamente dalla memoria condivisa
4. **Nessuna copia** dei 160KB di dati!

### Verifica Zero-Copy Attivo
Se il log mostra:
```
[WARN] Middleware cannot loan messages. Local allocator will be used.
```
Lo zero-copy **non** è attivo. Verificare le variabili d'ambiente.

### Percorso Installato
```
~ros2_ws/install/my_zero_copy_pkg/share/my_zero_copy_pkg/config/fastdds_setup.xml
```
