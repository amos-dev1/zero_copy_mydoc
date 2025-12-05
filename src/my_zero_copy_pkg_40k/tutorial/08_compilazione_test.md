# 8. Compilazione e Test

Questo capitolo spiega come compilare il package e testare i nodi.

---

## Passo 1: Verifica struttura file

Prima di compilare, verifica che tutti i file siano presenti:

```bash
cd ~/ros2_ws/src/my_zero_copy_pkg_40k
ls -la
```

Output atteso:
```
CMakeLists.txt
package.xml
config/
├── fastdds_setup.xml
docs/
msg/
├── FixedArray40k.msg
src/
├── loaned_publisher.cpp
├── loaned_subscriber.cpp
├── standard_publisher.cpp
├── standard_subscriber.cpp
tutorial/
```

---

## Passo 2: Compilazione

### Compila solo il nostro package

```bash
cd ~/ros2_ws
colcon build --packages-select my_zero_copy_pkg_40k
```

Output atteso:
```
Starting >>> my_zero_copy_pkg_40k
Finished <<< my_zero_copy_pkg_40k [X.XXs]

Summary: 1 package finished [X.XXs]
```

### Compila tutto il workspace

```bash
colcon build
```

### Gestione errori comuni

**Errore: "package not found"**
```bash
# Assicurati che il package sia in src/
ls ~/ros2_ws/src/
```

**Errore: "rosidl_generate_interfaces"**
```bash
# Verifica che rosidl_default_generators sia in package.xml
grep rosidl package.xml
```

**Errore: "cannot find -lmy_zero_copy_pkg_40k__rosidl..."**
```bash
# Il messaggio non è stato generato, ricompila da zero
rm -rf build/ install/ log/
colcon build
```

---

## Passo 3: Source del workspace

Dopo ogni compilazione, carica l'ambiente:

```bash
source ~/ros2_ws/install/setup.bash
```

Oppure aggiungi a `~/.bashrc`:
```bash
echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

---

## Passo 4: Impostare variabili ambiente

### Opzione A: Temporanea (per questa sessione)

```bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export RMW_FASTRTPS_USE_QOS_FROM_XML=1
export ROS_DISABLE_LOANED_MESSAGE=0
export ROS_DOMAIN_ID=0
export FASTRTPS_DEFAULT_PROFILES_FILE=$(ros2 pkg prefix my_zero_copy_pkg_40k)/share/my_zero_copy_pkg_40k/config/fastdds_setup.xml
```

### Opzione B: Permanente (aggiungi a ~/.bashrc)

```bash
cat >> ~/.bashrc << 'EOF'

# ROS2 FastDDS Zero-Copy Configuration
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export RMW_FASTRTPS_USE_QOS_FROM_XML=1
export ROS_DISABLE_LOANED_MESSAGE=0
export ROS_DOMAIN_ID=0
export FASTRTPS_DEFAULT_PROFILES_FILE=$(ros2 pkg prefix my_zero_copy_pkg_40k)/share/my_zero_copy_pkg_40k/config/fastdds_setup.xml
EOF

source ~/.bashrc
```

---

## Passo 5: Test Publisher e Subscriber Standard

### Terminale 1 - Subscriber

```bash
ros2 run my_zero_copy_pkg_40k standard_subscriber
```

Output:
```
[INFO] [standard_subscriber_40k]: Subscriber STANDARD 40K avviato. In ascolto su 'standard_array_40k'.
```

### Terminale 2 - Publisher

```bash
ros2 run my_zero_copy_pkg_40k standard_publisher
```

Output:
```
[INFO] [standard_publisher_40k]: Publisher STANDARD 40K avviato. 40000 interi ogni secondo.
[INFO] [standard_publisher_40k]: Msg #0 | Primo: 0 | Ultimo: 39999 | 40K
```

### Terminale 1 (subscriber riceve):
```
[INFO] [standard_subscriber_40k]: Msg #1 | Primo: 0 | Ultimo: 39999 | Dimensione: 40000
```

---

## Passo 6: Test Publisher e Subscriber Loaned (Zero-Copy)

### Terminale 1 - Subscriber

```bash
ros2 run my_zero_copy_pkg_40k loaned_subscriber
```

Output:
```
[INFO] [loaned_subscriber_40k]: Subscriber LOANED 40K avviato. In ascolto su 'fixed_array_40k'.
```

### Terminale 2 - Publisher

```bash
ros2 run my_zero_copy_pkg_40k loaned_publisher
```

Output (zero-copy attivo):
```
[INFO] [loaned_publisher_40k]: Zero-copy ABILITATO per 40.000 interi (160 KB).
[INFO] [loaned_publisher_40k]: Publisher LOANED 40K avviato. 40000 interi ogni secondo.
[INFO] [loaned_publisher_40k]: Msg #0 (zc) | Primo: 0 | Ultimo: 39999 | 40K
```

### Terminale 1 (subscriber riceve):
```
[INFO] [loaned_subscriber_40k]: Msg #1 | Primo: 0 | Ultimo: 39999 | Latenza: 0.234 ms | OK: SI
```

---

## Passo 7: Verifica topic attivi

In un terzo terminale:

```bash
ros2 topic list
```

Output:
```
/fixed_array_40k     # Loaned (zero-copy)
/standard_array_40k  # Standard
```

### Visualizza info topic

```bash
ros2 topic info /fixed_array_40k
```

Output:
```
Type: my_zero_copy_pkg_40k/msg/FixedArray40k
Publisher count: 1
Subscription count: 1
```

---

## Passo 8: Terminare i nodi

Premi `Ctrl+C` in ogni terminale per terminare i nodi.

---

## Comandi utili

### Visualizza nodi attivi
```bash
ros2 node list
```

### Visualizza frequenza messaggi
```bash
ros2 topic hz /fixed_array_40k
```

### Visualizza un messaggio (attenzione: molti dati!)
```bash
ros2 topic echo /fixed_array_40k --once
```

---

## Problemi comuni

### "Zero-copy NON DISPONIBILE"

1. Verifica variabili ambiente:
   ```bash
   echo $RMW_FASTRTPS_USE_QOS_FROM_XML
   echo $FASTRTPS_DEFAULT_PROFILES_FILE
   ```

2. Verifica che il file XML esista:
   ```bash
   cat $FASTRTPS_DEFAULT_PROFILES_FILE
   ```

3. Ricompila dopo modifiche all'XML:
   ```bash
   colcon build --packages-select my_zero_copy_pkg_40k
   source install/setup.bash
   ```

### "Publisher/Subscriber non comunicano"

1. Verifica che usino lo stesso topic name
2. Verifica che usino lo stesso `ROS_DOMAIN_ID`
3. Usa terminali separati, non tab!

---

## Prossimo Passo

Vai al capitolo [9. Verifica Funzionamento SHM](09_verifica_shm.md) per confermare che zero-copy sia attivo.
