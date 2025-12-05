# 3. Configurazione FastDDS

Questo capitolo spiega come configurare FastDDS per abilitare il data sharing (zero-copy).

---

## Cos'è FastDDS?

**FastDDS** (precedentemente Fast-RTPS) è il middleware DDS predefinito di ROS2. Gestisce la comunicazione tra nodi, inclusa la possibilità di usare memoria condivisa.

---

## File di configurazione XML

FastDDS si configura tramite un file XML. Crea il file `config/fastdds_setup.xml`:

```xml
<?xml version="1.0" encoding="UTF-8"?>
<!--
  Configurazione FastDDS per Zero-Copy / Data Sharing
  
  Questo file abilita il data sharing automatico tra publisher e subscriber.
-->
<dds xmlns="http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles">
    <profiles>
        <!-- Profilo publisher di default -->
        <publisher profile_name="default publisher profile" is_default_profile="true">
            <qos>
                <publishMode>
                    <kind>SYNCHRONOUS</kind>
                </publishMode>
                <!-- Abilita data sharing (zero-copy) -->
                <data_sharing>
                    <kind>ON</kind>
                </data_sharing>
            </qos>
            <historyMemoryPolicy>PREALLOCATED_WITH_REALLOC</historyMemoryPolicy>
        </publisher>

        <!-- Profilo subscriber di default -->
        <subscriber profile_name="default subscriber profile" is_default_profile="true">
            <qos>
                <!-- Abilita data sharing (zero-copy) -->
                <data_sharing>
                    <kind>ON</kind>
                </data_sharing>
            </qos>
            <historyMemoryPolicy>PREALLOCATED_WITH_REALLOC</historyMemoryPolicy>
        </subscriber>

        <!-- Profilo specifico per il topic fixed_array_40k -->
        <publisher profile_name="fixed_array_40k">
            <qos>
                <publishMode>
                    <kind>SYNCHRONOUS</kind>
                </publishMode>
                <data_sharing>
                    <kind>ON</kind>
                </data_sharing>
            </qos>
            <historyMemoryPolicy>PREALLOCATED_WITH_REALLOC</historyMemoryPolicy>
        </publisher>

        <subscriber profile_name="fixed_array_40k">
            <qos>
                <data_sharing>
                    <kind>ON</kind>
                </data_sharing>
            </qos>
            <historyMemoryPolicy>PREALLOCATED_WITH_REALLOC</historyMemoryPolicy>
        </subscriber>
    </profiles>
</dds>
```

---

## Spiegazione degli elementi XML

### Elemento radice `<dds>`
```xml
<dds xmlns="http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles">
```
Definisce il namespace XML per FastDDS.

### Elemento `<profiles>`
Contiene tutti i profili di configurazione.

### Elemento `<publisher>` / `<subscriber>`

| Attributo | Descrizione |
|-----------|-------------|
| `profile_name` | Nome del profilo |
| `is_default_profile` | Se `true`, usato come default |

### Elemento `<data_sharing>`

| Valore | Descrizione |
|--------|-------------|
| `ON` | Forza l'uso del data sharing |
| `OFF` | Disabilita il data sharing |
| `AUTOMATIC` | FastDDS decide automaticamente |

### Elemento `<publishMode>`

| Valore | Descrizione |
|--------|-------------|
| `SYNCHRONOUS` | Pubblicazione sincrona (più prevedibile) |
| `ASYNCHRONOUS` | Pubblicazione asincrona (più veloce) |

### Elemento `<historyMemoryPolicy>`

| Valore | Descrizione |
|--------|-------------|
| `PREALLOCATED_WITH_REALLOC` | Prealloca memoria, rialloca se necessario |
| `PREALLOCATED` | Solo memoria preallocata |
| `DYNAMIC` | Allocazione dinamica |

---

## Variabili d'ambiente

Per far funzionare la configurazione, devi impostare queste variabili d'ambiente.

### Aggiungi a `~/.bashrc`:

```bash
# Usa FastDDS come middleware
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

# Abilita lettura della configurazione XML
export RMW_FASTRTPS_USE_QOS_FROM_XML=1

# NON disabilitare i messaggi loaned
export ROS_DISABLE_LOANED_MESSAGE=0

# Domain ID (tutti i nodi devono usare lo stesso)
export ROS_DOMAIN_ID=0

# Percorso al file di configurazione
export FASTRTPS_DEFAULT_PROFILES_FILE=$(ros2 pkg prefix my_zero_copy_pkg_40k)/share/my_zero_copy_pkg_40k/config/fastdds_setup.xml
```

### Applica le modifiche:

```bash
source ~/.bashrc
```

---

## Spiegazione delle variabili

### RMW_IMPLEMENTATION
```bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
```
Indica a ROS2 di usare FastDDS come middleware.

### RMW_FASTRTPS_USE_QOS_FROM_XML
```bash
export RMW_FASTRTPS_USE_QOS_FROM_XML=1
```
Abilita la lettura delle impostazioni QoS dal file XML.

### ROS_DISABLE_LOANED_MESSAGE
```bash
export ROS_DISABLE_LOANED_MESSAGE=0
```
- `0` = Abilita i messaggi loaned (zero-copy)
- `1` = Disabilita i messaggi loaned

### ROS_DOMAIN_ID
```bash
export ROS_DOMAIN_ID=0
```
Tutti i nodi devono usare lo stesso Domain ID per comunicare.

### FASTRTPS_DEFAULT_PROFILES_FILE
```bash
export FASTRTPS_DEFAULT_PROFILES_FILE=/path/to/fastdds_setup.xml
```
Percorso al file di configurazione XML.

---

## Verifica della configurazione

### Verifica che le variabili siano impostate:

```bash
echo $RMW_IMPLEMENTATION
# Output atteso: rmw_fastrtps_cpp

echo $RMW_FASTRTPS_USE_QOS_FROM_XML
# Output atteso: 1

echo $FASTRTPS_DEFAULT_PROFILES_FILE
# Output atteso: /home/oe/ros2_ws/install/.../fastdds_setup.xml
```

### Verifica che il file XML esista:

```bash
cat $FASTRTPS_DEFAULT_PROFILES_FILE
```

---

## Errori comuni

### Errore: "Not expected tag"
```
[XMLPARSER Error] Not expected tag: 'transport_descriptors'
```
**Causa**: Struttura XML non valida.
**Soluzione**: Usa solo gli elementi supportati (vedi sopra).

### Errore: "Invalid element"
```
[XMLPARSER Error] Invalid element found in 'data_sharing'. Name: shared_memory_directory
```
**Causa**: Elemento `shared_memory_directory` non supportato in questa versione.
**Soluzione**: Rimuovi l'elemento.

---

## Prossimo Passo

Vai al capitolo [4. Publisher Standard](04_publisher_standard.md) per creare il publisher tradizionale.
