#define NUMERO_CARTE 2 // Numero di carte abilitate
#define LED_OK 2       // Pin del led verde (accesso consentito)
#define LED_KO 3       // Pin del led rosso (accesso negato)

char secret[64] = "supersecret";           // chiave utilizzata come salt
uint8_t UID_ABILITATI[NUMERO_CARTE][4] = { // Lista delle carte abilitate
    {0xE3, 0xDA, 0x99, 0x2B},
    {0xDA, 0x9D, 0x90, 0xB1}};
uint32_t CONTATORI[NUMERO_CARTE] = {0, 0}; // Contatori delle carte abilitate

uint8_t key[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF}; // chiave per leggere e scrivere sulla carta

#include <stdint.h>
#include <SoftwareSerial.h>
#include <PN532_SWHSU.h>
#include <PN532.h>
#include "MD5.h" // tzikis/ArduinoMD5

SoftwareSerial SWSerial(4, 5);
PN532_SWHSU pn532swhsu(SWSerial);
PN532 nfc(pn532swhsu);

unsigned long ledTurnedOnMillis = 0;
boolean ledOn = false;

void setup(void)
{
    pinMode(LED_OK, OUTPUT);
    pinMode(LED_KO, OUTPUT);
    digitalWrite(LED_OK, LOW);
    digitalWrite(LED_KO, LOW);

    Serial.begin(115200);
    nfc.begin();
    uint32_t versionFata = nfc.getFirmwareVersion();
    if (!versionFata)
    {
        Serial.println("Modulo NFC non trovato!");
        while (1)
            ;
    }
    nfc.SAMConfig();
}

unsigned long currentMillis = 0;
unsigned long prevMillis = 0;
unsigned long lastRead = 0;
uint8_t prevUid[4] = {0, 0, 0, 0};

void loop()
{
    currentMillis = millis();
    if (ledOn && currentMillis - ledTurnedOnMillis >= 1000) // spengo il led dopo 1 secondo
    {
        ledOn = false;
        digitalWrite(LED_OK, LOW);
        digitalWrite(LED_KO, LOW);
    }

    if (currentMillis - prevMillis >= 100) // provo a leggere un tag ogni 100ms
    {
        uint8_t acKey[16]; // Chiave segreta per l'accesso

        boolean success;
        uint8_t uid[] = {0, 0, 0, 0, 0, 0, 0};
        uint8_t uidLength;
        uint32_t now = millis();
        // Provo a leggere un tag
        success = nfc.readPassiveTargetID(PN532_MIFARE_ISO14443A, &uid[0], &uidLength);
        // Supporto solo i tag Mifare Classic con 4 byte UID
        if (success && uidLength == 4)
        {
            // procedo solo se il codice letto è diverso da quello letto precedentemente o se è passato almeno un secondo
            if (memcmp(uid, prevUid, 4) != 0 || now - lastRead > 1000)
            {
                memcpy(prevUid, uid, 4);
                lastRead = now;
                Serial.print("UID: ");
                for (uint8_t i = 0; i < 4; i++)
                {
                    Serial.print(uid[i], HEX);
                }
                Serial.println("");

                int card = -1;
                // itero sulla lista di carte abilitate per trovare la corrispondenza (se esiste)
                for (int i = 0; i < NUMERO_CARTE; i++)
                {
                    if (memcmp(UID_ABILITATI[i], uid, 4) == 0)
                    {
                        card = i;
                        break;
                    }
                }
                success = false;
                // se la carta è abilitata procedo
                if (card != -1)
                {
                    // recupero la chiave salvata sulla carta
                    uint8_t currentKey[16];
                    success = nfc.mifareclassic_AuthenticateBlock(uid, 4, 4, 1, key);
                    if (success)
                        success = nfc.mifareclassic_ReadDataBlock(4, acKey);

                    if (success)
                    {
                        // genero la chiave in base al contatore attuale per verificare che corrisponda a quella salvata sulla carta
                        generaChiave(uid, CONTATORI[card], currentKey);
                        if (memcmp(currentKey, key, 16) == 0)
                        {
                            // la chiave è corretta!
                            uint8_t nextKey[16];
                            generaChiave(uid, CONTATORI[card] + 1, nextKey);

                            // salvo la prossima chiave sulla carta nel settore 4
                            success = nfc.mifareclassic_AuthenticateBlock(uid, 4, 4, 1, key);
                            if (success)
                                success = nfc.mifareclassic_WriteDataBlock(4, nextKey);

                            if (success)
                            {
                                // ho aggiornato la chiave sulla carta, posso aggiornare il contatore e segnalare l'accesso consentito
                                CONTATORI[card]++;
                                digitalWrite(LED_OK, HIGH);
                                digitalWrite(LED_KO, LOW);
                                Serial.println("Accesso consentito!");
                            }
                            else
                                Serial.println("Errore nell'aggiornamento della chiave!");
                        }
                        else
                            Serial.println("Accesso negato!");
                    }
                    else
                        Serial.println("Errore nella lettura della chiave!");
                }
                else
                    Serial.println("Carta non presente in lista!");
            }
            if (!success)
            {
                digitalWrite(LED_OK, LOW);
                digitalWrite(LED_KO, HIGH);
            }
        }
        if (!success)
        {
            prevUid[0] = ~prevUid[0];
        }
    }
}

void generaChiave(uint8_t *uid, uint32_t counter, uint8_t *key)
{
    char buffer[64];
    for (int i = 0; i < 4; i++)
    {
        sprintf(buffer + i * 2, "%02X", uid[i]);
    }
    sprintf(buffer + 8, "%08X", counter);
    char *payload = strcat(buffer, secret);
    unsigned char *hash = MD5::make_hash(payload);
    memcpy(key, hash, 16);
}
