#include <I2C_Insarianne.h>

MPU6050 mpu;

void Transfert_Info(){
    mpu.read_sensor();

    Serial.print("Accéleration X : ");
    Serial.print(mpu.accX);
    Serial.println(" g,");
    Serial.print("Accéleration Y : ");
    Serial.print(mpu.accY);
    Serial.println(" g,");
    Serial.print("Accéleration Z : ");
    Serial.print(mpu.accZ);
    Serial.println(" g,");
    Serial.print("Rotation X : ");
    Serial.print(mpu.gyroX);
    Serial.println(" rad/s,");
    Serial.print("Rotation Y : ");
    Serial.print(mpu.gyroY);
    Serial.println(" rad/s,");
    Serial.print("Rotation Z : ");
    Serial.print(mpu.gyroZ);
    Serial.println(" rad/s,");
    Serial.print("Température : ");
    Serial.print(mpu.temperature);
    Serial.println(" °C.");
    Serial.println();
}

void setup() {
    Serial.begin(9600);
    Serial.print("Version Librairie : "); Serial.println(VERSION_LIB);
    Wire.begin();

    mpu.begin();
}

void loop() {
    Transfert_Info();
    delay(500);
}