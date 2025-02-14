# Script di Generazione Chiavi con Docker

Questo script genera una coppia di chiavi EC utilizzando OpenSSL in un container Docker e salva le chiavi in un file tar.

## Come Funziona

1. **Dockerfile temporaneo**:
   - Crea un `Dockerfile` con due stadi: builder (`alpine/openssl`) per generare le chiavi, e output (`scratch`) per includere solo le chiavi.

2. **Costruzione immagine Docker**:
   - Costruisce l'immagine Docker e salva le chiavi in `keys.tar`.

3. **Pulizia**:
   - Rimuove il `Dockerfile` temporaneo.

## Prerequisiti

- Docker installato.

## Utilizzo

1. Avvia `generate_keys.bat`.
2. Troverai le tue chiavi nel file `keys.tar`.