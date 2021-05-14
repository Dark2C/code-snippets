# Apriporta NFC

## Panoramica
Implementazione di un controllo accessi con modulo PN532 che implementa un sistema anti-clonazione tramite HMAC.

## Hardware
- **Scheda Arduino** (es. Arduino Uno).
- **Modulo PN532** per leggere carte RFID MIFARE Classic.
- **Carte RFID MIFARE Classic** con UID a 4 byte.
- **LEDs** (verde e rosso) per feedback.
- **Resistenze** da 220Ω per i LED.

## Collegamenti
1. **Modulo PN532**: SDA (Pin 4), SCL (Pin 5), VCC (5V), GND (GND).
2. **LEDs**: Verde su Pin 2 (LED_OK), Rosso su Pin 3 (LED_KO).

## Funzionamento
- Il sistema legge l'UID delle carte RFID. Se l'UID è autorizzato, il sistema verifica la chiave tramite HMAC con MD5, basata su UID e contatore.
- Se la chiave è corretta, il contatore viene incrementato e la chiave successiva viene scritta sulla carta, consentendo l'accesso (LED verde).
- Se la chiave è errata o l'UID non è autorizzato, l'accesso viene negato (LED rosso).

## Limitazioni
- **Contatori azzerati**: Dopo ogni riavvio, i contatori ripartono da 0, quindi è necessario ripristinare la carta con **Mifare Classic Tool**.