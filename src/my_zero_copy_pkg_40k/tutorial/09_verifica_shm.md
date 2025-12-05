# 9. Verifica Funzionamento SHM (Shared Memory)

Questo capitolo spiega come verificare che zero-copy e shared memory siano effettivamente attivi.

---

## Metodo 1: Controllo /dev/shm

La shared memory di FastDDS crea file in `/dev/shm`. Verifica mentre i nodi sono in esecuzione:

```bash
ls -la /dev/shm | grep fastrtps
```

### Output atteso (SHM funzionante)

```
-rw-r--r--  1 user user    549408 Dec  5 15:49 fastrtps_361c755af764764b
-rw-rw-r--  1 user user         0 Dec  5 15:49 fastrtps_361c755af764764b_el
-rw-r--r--  1 user user     52400 Dec  5 15:49 fastrtps_port7411
-rw-rw-r--  1 user user         0 Dec  5 15:49 fastrtps_port7411_el
-rw-r--r--  1 user user        32 Dec  5 15:49 sem.fastrtps_port7411_mutex
```

### Spiegazione file

| File | Descrizione |
|------|-------------|
| `fastrtps_XXXX` | Segmento di memoria condivisa per i dati |
| `fastrtps_XXXX_el` | Event lock per sincronizzazione |
| `fastrtps_portXXXX` | Porta di comunicazione SHM |
| `sem.fastrtps_*` | Semaforo per sincronizzazione |

### Se NON vedi file fastrtps_*

SHM non è attivo. Verifica:
1. Variabili ambiente impostate
2. File XML caricato correttamente
3. Publisher e subscriber in esecuzione

---

## Metodo 2: FastDDS Monitor

FastDDS Monitor è un tool grafico per monitorare il middleware.

### Installazione

```bash
sudo apt install fastdds-tools
```

### Esecuzione

```bash
fastdds_monitor
```

### Cosa cercare

1. Apri la tab "Physical Data"
2. Cerca i locator dei participant
3. Verifica i tipi di locator:

| Tipo Locator | Significato |
|--------------|-------------|
| `SHM` | Shared Memory (zero-copy attivo) |
| `UDPv4` | UDP multicast (usato per discovery) |

### Nota importante

Vedere solo `UDPv4` NON significa che SHM non funzioni!

FastDDS usa:
- **UDPv4** per la **discovery** (trovare altri nodi)
- **SHM** per il **trasferimento dati** effettivo

I locator mostrati sono spesso quelli di discovery. Controlla `/dev/shm` per conferma definitiva.

---

## Metodo 3: Output del Publisher

Il nostro publisher loaned indica chiaramente se zero-copy è attivo:

### Zero-copy ATTIVO

```
[INFO] [loaned_publisher_40k]: Zero-copy ABILITATO per 40.000 interi (160 KB).
[INFO] [loaned_publisher_40k]: Msg #0 (zc) | Primo: 0 | Ultimo: 39999
                                      ^^^^
                                      "zc" = zero-copy
```

### Zero-copy NON ATTIVO

```
[WARN] [loaned_publisher_40k]: Zero-copy NON DISPONIBILE - Usando pubblicazione standard.
[INFO] [loaned_publisher_40k]: Msg #0 (std) | Primo: 0 | Ultimo: 39999
                                      ^^^^^
                                      "std" = standard (con copie)
```

---

## Metodo 4: Confronto Latenze

Confronta la latenza tra standard e loaned:

### Test Standard

```bash
# Terminale 1
ros2 run my_zero_copy_pkg_40k standard_subscriber

# Terminale 2
ros2 run my_zero_copy_pkg_40k standard_publisher
```

Nota: non mostra latenza, ma puoi usare `ros2 topic delay`.

### Test Loaned

```bash
# Terminale 1
ros2 run my_zero_copy_pkg_40k loaned_subscriber

# Terminale 2
ros2 run my_zero_copy_pkg_40k loaned_publisher
```

Il subscriber loaned mostra la latenza:
```
Latenza: 0.234 ms
```

### Latenze tipiche

| Metodo | Latenza (160 KB) |
|--------|-----------------|
| Standard | 1-5 ms |
| Zero-copy (SHM) | 0.1-0.5 ms |

Se la latenza loaned è molto più bassa, SHM sta funzionando!

---

## Metodo 5: Monitoraggio Memoria

Usa `htop` o simili per verificare l'uso di memoria:

```bash
htop
```

Con zero-copy:
- La memoria usata dovrebbe essere minore
- Meno copie = meno allocazioni

---

## Checklist Finale

Prima di considerare zero-copy funzionante, verifica:

- [ ] File `fastrtps_*` presenti in `/dev/shm` durante esecuzione
- [ ] Publisher stampa "Zero-copy ABILITATO"
- [ ] Publisher stampa "(zc)" nei messaggi
- [ ] Subscriber riceve messaggi con integrità OK
- [ ] Latenza bassa (< 1 ms per 160 KB)

---

## Riepilogo Architettura

```
                    ┌──────────────────────────────────────┐
                    │         /dev/shm (Linux)              │
                    │                                       │
   ┌────────────┐   │  ┌─────────────────────────────────┐ │   ┌────────────┐
   │ Publisher  │───│──│  fastrtps_xxxxx                 │─│───│ Subscriber │
   │ (loaned)   │   │  │  ┌────────────────────────────┐ │ │   │ (callback) │
   │            │   │  │  │ timestamp_sec: 1234567     │ │ │   │            │
   │ borrow()   │   │  │  │ timestamp_nanosec: 890123  │ │ │   │ msg->data  │
   │ get()      │   │  │  │ data[0]: 0                 │ │ │   │            │
   │ publish()  │   │  │  │ data[1]: 1                 │ │ │   │            │
   └────────────┘   │  │  │ ...                        │ │ │   └────────────┘
                    │  │  │ data[39999]: 39999         │ │ │
                    │  │  └────────────────────────────┘ │ │
                    │  └─────────────────────────────────┘ │
                    │                                       │
                    │  Stessa memoria fisica!               │
                    │  Nessuna copia!                       │
                    └──────────────────────────────────────┘
```

---

## Conclusione

Se hai seguito tutti i passi del tutorial, ora hai:

1. ✅ Un package ROS2 con messaggi POD
2. ✅ Publisher e subscriber zero-copy funzionanti
3. ✅ Configurazione FastDDS per SHM
4. ✅ Verifica che SHM sia attivo

**Congratulazioni! Il tuo sistema zero-copy è operativo!**

---

## Risorse aggiuntive

- [FastDDS Documentation](https://fast-dds.docs.eprosima.com/)
- [ROS2 Loaned Messages](https://docs.ros.org/en/humble/How-To-Guides/DDS-Loaned-Message-Info.html)
- [Repository di riferimento](https://github.com/leonardonels/zero_copy)
