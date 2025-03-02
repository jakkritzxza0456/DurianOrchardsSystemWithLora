#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>

const lmic_pinmap lmic_pins = {
    .nss = 5,
    .rxtx = LMIC_UNUSED_PIN,
    .rst = 27,
    .dio = {26, 33, 32},
};

static const u1_t PROGMEM APPEUI[8] = { 0x98, 0x6f, 0x23, 0xde, 0x5a, 0xd7, 0xeb, 0x5d }; //JoinEUI
static const u1_t PROGMEM DEVEUI[8] = { 0xf6, 0x00, 0x90, 0xe4, 0xb8, 0xe5, 0xce, 0x5c }; //DevEUI
static const u1_t PROGMEM APPKEY[16] = { 0xce, 0xb6, 0x8b, 0x71, 0xdd, 0xe7, 0xff, 0x66, 0x30, 0xf2, 0x87, 0x4b, 0x68, 0xd2, 0x05, 0x43 }; //AppKey

void os_getArtEui(u1_t* buf) { memcpy(buf, APPEUI, 8); }
void os_getDevEui(u1_t* buf) { memcpy(buf, DEVEUI, 8); }
void os_getDevKey(u1_t* buf) { memcpy(buf, APPKEY, 16); }

static osjob_t sendjob;
const unsigned TX_INTERVAL = 20;

void printDataRate() {
    Serial.print(F("Current DR: "));
    Serial.print(LMIC.datarate);
    Serial.print(F(" | SF: "));

    switch (LMIC.datarate) {
        case LORAWAN_DR0: Serial.println(F("SF12")); break;
        case LORAWAN_DR1: Serial.println(F("SF11")); break;
        case LORAWAN_DR2: Serial.println(F("SF10")); break;
        case LORAWAN_DR3: Serial.println(F("SF9")); break;
        case LORAWAN_DR4: Serial.println(F("SF8")); break;
        case LORAWAN_DR5: Serial.println(F("SF7")); break;
        default: Serial.println(F("Unknown"));
    }
}

void onEvent(ev_t ev) {
    Serial.print(os_getTime());
    Serial.print(": ");

    switch(ev) {
        case EV_JOINING:
            Serial.println(F("EV_JOINING"));
            break;
        
        case EV_JOINED:
            Serial.println(F("EV_JOINED"));
            Serial.println(F("RX2 Data Rate set to SF12"));
            printDataRate();
            break;
        
        case EV_TXCOMPLETE:
            Serial.println(F("EV_TXCOMPLETE"));
            printDataRate();
            break;
        
        default:
            Serial.println(F("Unknown event"));
            break;
    }
}

void do_send(osjob_t* j) {
    if (LMIC.opmode & OP_TXRXPEND) {
        Serial.println(F("OP_TXRXPEND, not sending"));
    } else {
        const char* message = "Hello from ESP32!";
        LMIC_setTxData2(1, (uint8_t*)message, strlen(message), 0);
        Serial.print(F("Packet queued: "));
        Serial.println(message);
        printDataRate();
    }

    os_setTimedCallback(&sendjob, os_getTime() + sec2osticks(TX_INTERVAL), do_send);
}

void setup() {
    Serial.begin(115200);
    Serial.println(F("Starting..."));
    os_init();
    LMIC_reset();
    LMIC_setClockError(10 * MAX_CLOCK_ERROR / 100);  // แก้ปัญหา Timing ของ ESP32

    LMIC_startJoining();
    os_setCallback(&sendjob, do_send);
}

void loop() {
    os_runloop_once();
}
