# Convertitore da M3U8 a MP4

Questi script consentono di effettuare la conversione di scalette M3U8 in MP4 mediante **ffmpeg**. Sono presenti due script:

- **m3u8-to-mp4.sh** (per ambienti Unix-like, Linux): utilizza Docker per eseguire `ffmpeg`.
- **m3u8-to-mp4.bat** (per Windows): utilizza direttamente l'eseguibile `ffmpeg`.

## Funzionamento (Linux)

Esegui lo script con due parametri: la directory di origine e quella di destinazione.

```bash
m3u8-to-mp4.sh /path/to/source/directory /path/to/destination/directory
```

**Dipendenze**:
- Docker

## Funzionamento (Windows)

Esegui lo script passando il percorso del file M3U8 da convertire.

```bat
convert.bat "C:\path\to\file.m3u8"
```

**Dipendenze**:
- ffmpeg (nella cartella corrente o installato sul sistema)
