# Custom message for zero-copy data transfer
# Messaggio personalizzato per trasferimento dati zero-copy
#
# EN: Fixed-length array of 125 int32 for zero-copy support.
#     This message is PURE POD (Plain Old Data) - NO strings or dynamic types!
#     This is REQUIRED for loaned messages / zero-copy to work.
#
# IT: Array a lunghezza fissa di 125 int32 per supporto zero-copy.
#     Questo messaggio è PURO POD (Plain Old Data) - NESSUNA stringa o tipo dinamico!
#     Questo è NECESSARIO perché loaned messages / zero-copy funzionino.
#
# NOTE: std_msgs/Header was REMOVED because it contains frame_id (string)
#       which is non-POD and causes NotEnoughMemoryException!
# NOTA: std_msgs/Header è stato RIMOSSO perché contiene frame_id (stringa)
#       che è non-POD e causa NotEnoughMemoryException!

# Timestamp as two integers (POD) instead of Header
# Timestamp come due interi (POD) invece di Header
int32 timestamp_sec
uint32 timestamp_nanosec

# Fixed-length array of 125 integers (500 bytes)
# Array a lunghezza fissa di 125 interi (500 byte)
int32[125] data
