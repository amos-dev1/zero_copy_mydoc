# Custom message for zero-copy data transfer - 40K VERSION
# Messaggio personalizzato per trasferimento dati zero-copy - VERSIONE 40K
#
# EN: Fixed-length array of 40000 int32 for zero-copy support.
#     This is a PURE POD message (no strings, no dynamic types).
#
# IT: Array a lunghezza fissa di 40000 int32 per supporto zero-copy.
#     Questo è un messaggio PURO POD (nessuna stringa, nessun tipo dinamico).

# Timestamp as POD integers (no Header to keep it pure POD)
# Timestamp come interi POD (nessun Header per mantenerlo POD puro)
int32 timestamp_sec
uint32 timestamp_nanosec

# Fixed-length array of 40000 integers (160 KB)
# Array a lunghezza fissa di 40000 interi (160 KB)
int32[40000] data
