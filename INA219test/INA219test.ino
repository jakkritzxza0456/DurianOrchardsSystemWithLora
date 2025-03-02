#include <Wire.h>
#include <Adafruit_INA219.h>

Adafruit_INA219 ina219;

void setup() {
    Serial.begin(115200);
    Serial.println("เริ่มต้นการทดสอบ INA219...");

    if (!ina219.begin()) {
        Serial.println("ไม่พบเซ็นเซอร์ INA219! ตรวจสอบการเชื่อมต่อ I2C");
        while (1);
    }

    // ตั้งค่าการคาลิเบรตให้เหมาะสม
    ina219.setCalibration_32V_2A(); 

    Serial.println("INA219 พร้อมใช้งาน!");
}

void loop() {
    float busVoltage = ina219.getBusVoltage_V(); // อ่านค่าแรงดันบัส (V)

    // แปลงค่าแรงดันเป็นเปอร์เซ็นต์ (1V = 0%, 4.2V = 100%)
    int voltagePercent = (int) ((busVoltage - 1.0) * (100.0 / (4.2 - 1.0)));

    // จำกัดค่าไม่ให้เกิน 100% หรือต่ำกว่า 0%
    voltagePercent = constrain(voltagePercent, 0, 100);

    Serial.print("แรงดันบัส (V): "); Serial.print(busVoltage);
    Serial.print("\tแรงดันเป็นเปอร์เซ็นต์: "); Serial.print(voltagePercent);
    Serial.println("%");

    delay(1000); // หน่วงเวลา 1 วินาที
}
