#include <SoftwareSerial.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <Adafruit_INA219.h>
#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>
#include <Arduino.h>
#include <esp_sleep.h>
#include "esp_task_wdt.h"

// ----- การตั้งค่าตัวแปร -----
#define RE 4
#define DE 14
#define BME1_ADDR 0x76
#define BME2_ADDR 0x77
#define SOIL_MOISTURE_ANALOG_PIN 34  
  // แปลงจากค่าที่เห็นเป็นตัวเลขฐาน 16

const unsigned TX_INTERVAL = 30;  // ช่วงเวลาส่งข้อมูล (วินาที)
const unsigned SLEEP_INTERVAL = 900000000; //900000000 // 15 นาที (มิลลิวินาที)
const byte MAX_SEND_COUNT = 2;
byte sendCount = 0;

Adafruit_BME280 bme1;
Adafruit_BME280 bme2;
Adafruit_INA219 ina219;

SoftwareSerial mod(16, 17);

static const u1_t PROGMEM APPEUI[8] = { 0xEE, 0x51, 0x60, 0xA1, 0x76, 0x26, 0x2E, 0xA2 };
static const u1_t PROGMEM DEVEUI[8] = { 0xDD, 0x20, 0xD0, 0x63, 0x2A, 0xCB, 0x78, 0xAF };
static const u1_t PROGMEM APPKEY[16] = { 0xD2, 0x5E, 0xDF, 0xF4, 0xEB, 0xBC, 0xA4, 0xE6, 0x4E, 0xBD, 0x41, 0x64, 0x40, 0x6A, 0xD4, 0xD1 };

void os_getArtEui (u1_t* buf) { memcpy_P(buf, APPEUI, 8); }
void os_getDevEui (u1_t* buf) { memcpy_P(buf, DEVEUI, 8); }
void os_getDevKey (u1_t* buf) { memcpy_P(buf, APPKEY, 16); }

static osjob_t sendjob;

const lmic_pinmap lmic_pins = {
    .nss = 5,
    .rxtx = LMIC_UNUSED_PIN,
    .rst = 27,
    .dio = {26, 33, 32},
};

byte values[11];
const byte nitro[] = {0x01, 0x03, 0x00, 0x1e, 0x00, 0x01, 0xe4, 0x0c};
const byte phos[]  = {0x01, 0x03, 0x00, 0x1f, 0x00, 0x01, 0xb5, 0xcc};
const byte pota[]  = {0x01, 0x03, 0x00, 0x20, 0x00, 0x01, 0x85, 0xc0};

void goToSleep() {
    Serial.println(F("เข้าสู่โหมด Sleep เป็นเวลา 15 นาที"));
    // 💤 ปิด BME280 (เข้าสู่โหมด Sleep)
    bme1.setSampling(Adafruit_BME280::MODE_SLEEP,
                     Adafruit_BME280::SAMPLING_NONE,
                     Adafruit_BME280::SAMPLING_NONE,
                     Adafruit_BME280::SAMPLING_NONE);
                     
    bme2.setSampling(Adafruit_BME280::MODE_SLEEP,
                     Adafruit_BME280::SAMPLING_NONE,
                     Adafruit_BME280::SAMPLING_NONE,
                     Adafruit_BME280::SAMPLING_NONE);

    // 💤 ปิด INA219
    ina219.powerSave(true);

    esp_sleep_disable_wakeup_source(ESP_SLEEP_WAKEUP_ALL); // ปิดทุก Wakeup Source ที่ไม่ได้ใช้
    esp_sleep_enable_timer_wakeup(SLEEP_INTERVAL);
    Serial.println(F("กำลังเข้าสู่ Deep Sleep..."));
    delay(200);  // รอให้ Serial ส่งข้อมูลออกก่อน
    esp_deep_sleep_start();
}

byte nitrogen() {
    memset(values, 0, sizeof(values));  // 🏆 เคลียร์ค่าขยะก่อนอ่าน
    digitalWrite(DE, HIGH);
    digitalWrite(RE, HIGH);
    delay(10);
    if(mod.write(nitro, sizeof(nitro)) == 8) {
        digitalWrite(DE, LOW);
        digitalWrite(RE, LOW);
        for(byte i = 0; i < 7; i++){
            values[i] = mod.read();
        }
    }
    return values[4];
}

byte phosphorous() {
    memset(values, 0, sizeof(values));  // 🏆 เคลียร์ค่าขยะก่อนอ่าน
    digitalWrite(DE, HIGH);
    digitalWrite(RE, HIGH);
    delay(10);
    if(mod.write(phos, sizeof(phos)) == 8) {
        digitalWrite(DE, LOW);
        digitalWrite(RE, LOW);
        for(byte i = 0; i < 7; i++){
            values[i] = mod.read();
        }
    }
    return values[4];
}

byte potassium() {
    memset(values, 0, sizeof(values));  // 🏆 เคลียร์ค่าขยะก่อนอ่าน
    digitalWrite(DE, HIGH);
    digitalWrite(RE, HIGH);
    delay(10);
    if(mod.write(pota, sizeof(pota)) == 8) {
        digitalWrite(DE, LOW);
        digitalWrite(RE, LOW);
        for(byte i = 0; i < 7; i++){
            values[i] = mod.read();
        }
    }
    return values[4];
}


void do_send(osjob_t* j) {
    if (LMIC.opmode & OP_TXRXPEND) {
        Serial.println(F("OP_TXRXPEND, ไม่สามารถส่งข้อมูลได้"));
    } else {
        float temperature = bme1.readTemperature();
        float humidity = bme1.readHumidity();
        float temperatureair = bme2.readTemperature();
        float humidityair = bme2.readHumidity();
        float busVoltage = ina219.getBusVoltage_V();
        float current_mA = ina219.getCurrent_mA();
        float loadVoltage = busVoltage + (ina219.getShuntVoltage_mV() / 1000.0);
        
        int analogValue = analogRead(SOIL_MOISTURE_ANALOG_PIN);
        int soilMoisturePercent = map(analogValue, 4095, 0, 0, 100);
        int busVoltagePercent = (int) ((busVoltage - 1.0) * (100.0 / (4.2 - 1.0)));
        busVoltagePercent = constrain(busVoltagePercent, 0, 100);

        byte n_val = nitrogen();
        delay(250);
        byte p_val = phosphorous();
        delay(250);
        byte k_val = potassium();
        delay(250);
        
        char payload[128];
        snprintf(payload, sizeof(payload), "T:%.2f,H:%.2f,V:%.2d,I:%.2f,S:%d,N:%d,P:%d,K:%d,TA:%.2f,HA:%.2f", 
                 temperature, humidity, busVoltagePercent, current_mA, soilMoisturePercent, 
                 n_val, p_val, k_val, temperatureair, humidityair);
        
        LMIC_setTxData2(1, (uint8_t*)payload, strlen(payload), 0);
        Serial.print(F("Packet ถูกส่ง: "));
        Serial.println(payload);
        Serial.println(busVoltagePercent);
        sendCount++;
        Serial.print(F("จำนวนครั้งที่ส่งข้อมูล: "));
        Serial.println(sendCount);

        if (sendCount >= MAX_SEND_COUNT) {
            goToSleep();
        }
    }
    os_setTimedCallback(&sendjob, os_getTime() + sec2osticks(TX_INTERVAL), do_send);
}

void print_wakeup_reason() {
    esp_sleep_wakeup_cause_t wakeup_reason;
    wakeup_reason = esp_sleep_get_wakeup_cause();

    Serial.print(F("สาเหตุที่ ESP32 ตื่นขึ้นมา: "));
    switch (wakeup_reason) {
        case ESP_SLEEP_WAKEUP_TIMER: Serial.println(F("ตื่นจาก Timer")); break;
        case ESP_SLEEP_WAKEUP_EXT0: Serial.println(F("ตื่นจาก EXT0")); break;
        case ESP_SLEEP_WAKEUP_EXT1: Serial.println(F("ตื่นจาก EXT1")); break;
        case ESP_SLEEP_WAKEUP_TOUCHPAD: Serial.println(F("ตื่นจาก Touchpad")); break;
        case ESP_SLEEP_WAKEUP_ULP: Serial.println(F("ตื่นจาก ULP")); break;
        default: Serial.println(F("ไม่ทราบสาเหตุ (อาจเป็นรีเซ็ต)")); break;
    }
}


void setup() {
    Serial.begin(115200);
    esp_task_wdt_deinit();
    pinMode(RE, OUTPUT);
    pinMode(DE, OUTPUT);
    mod.begin(4800);
    delay(3000);
    print_wakeup_reason();
    Wire.begin();
    if (!bme1.begin(BME1_ADDR)) {
        Serial.println(F("ไม่พบ BME280 1"));
    }
    if (!bme2.begin(BME2_ADDR)) {
        Serial.println(F("ไม่พบ BME280 2"));
    }
    if (!ina219.begin()) {
        Serial.println(F("ไม่พบ INA219! ตรวจสอบการเชื่อมต่อ I2C"));
    } else {
        Serial.println(F("INA219 พร้อมใช้งาน!"));
    }
    ina219.powerSave(false);
    ina219.setCalibration_32V_2A(); 
    

    os_init();
    LMIC_reset();

    LMIC_setDrTxpow(DR_SF9, 20);
    do_send(&sendjob);
}




void loop() {
    os_runloop_once();
}
