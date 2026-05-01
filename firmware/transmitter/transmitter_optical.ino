/*
 * Tactical HeadTrack FPV - Optical Fiber Transmitter
 *
 * Передатчик с оптоволоконной связью (Toslink/TTL).
 * Защита от РЭБ, EMI/RFI иммунитет.
 *
 * Wiring (Toslink модуль TOTX173):
 *   MPU-6050: VCC -> 3.3V, GND -> GND, SCL -> A5, SDA -> A4
 *   TOTX173: VCC -> 5V через резистор 220Ω, GND -> GND, IN -> TX (D1)
 *   Home Button: D2 -> GND
 *
 * Wiring (Vishay VTLT1A00):
 *   VTLT1A00: VCC -> 5V, GND -> GND, ANODE -> TX (D1)
 *   (Катод LED через резистор к TX)
 */

#include <Wire.h>
#include <SoftwareSerial.h>

// ================= CONFIG =================
const int HOME_PIN = 2;
const int OPTICAL_TX_PIN = 1;  // Hardware TX на Nano
const int BAUD_RATE = 115200;

// Маджвик фильтр
const float beta = 0.1f;
const float sampleFreq = 100.0f;

// Серво пределы
const int SERVO_MIN = 0;
const int SERVO_MAX = 180;
const int SERVO_CENTER = 90;

const int HOME_THRESHOLD = 5;

// ================= VARIABLES =================
struct __attribute__((packed)) {
    uint16_t yaw;   // 0-100
    uint16_t pitch; // 0-100
    uint8_t home;
    uint8_t checksum;  // Простая контрольная сумма
} txData;

// Маджвик
float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;
float roll = 0.0f, pitch = 0.0f, yaw = 0.0f;
float rollOffset = 0.0f, pitchOffset = 0.0f, yawOffset = 0.0f;

bool calibrateRequested = false;

// MPU-6050
const int MPU_ADDR = 0x68;
struct {
    int16_t ax, ay, az;
    int16_t gx, gy, gz;
} mpuData;

// ================= MPU-6050 =================
void initMPU() {
    Wire.begin();
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x6B);
    Wire.write(0);
    Wire.endTransmission(true);

    // DLPF для сглаживания
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x1A);
    Wire.write(0x03);
    Wire.endTransmission(true);

    // Гироскоп ±250 deg/s
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x1B);
    Wire.write(0x00);
    Wire.endTransmission(true);

    // Акселерометр ±2g
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x1C);
    Wire.write(0x00);
    Wire.endTransmission(true);
}

void readMPU() {
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x3B);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU_ADDR, 14, true);

    mpuData.ax = (Wire.read() << 8 | Wire.read());
    mpuData.ay = (Wire.read() << 8 | Wire.read());
    mpuData.az = (Wire.read() << 8 | Wire.read());
    Wire.read(); Wire.read();
    mpuData.gx = (Wire.read() << 8 | Wire.read());
    mpuData.gy = (Wire.read() << 8 | Wire.read());
    mpuData.gz = (Wire.read() << 8 | Wire.read());
}

// ================= MADGWICK =================
void madgwickUpdate(float gx, float gy, float gz, float ax, float ay, float az) {
    float recipNorm, s0, s1, s2, s3;
    float qDot1, qDot2, qDot3, qDot4;
    float _2q0, _2q1, _2q2, _2q3, _4q0, _4q1, _4q2, _8q1, _8q2;

    qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
    qDot2 = 0.5f * (q0 * gx + q2 * gz - q3 * gy);
    qDot3 = 0.5f * (q0 * gy - q1 * gz + q3 * gx);
    qDot4 = 0.5f * (q0 * gz + q1 * gy - q2 * gx);

    recipNorm = sqrt(ax * ax + ay * ay + az * az);
    if (recipNorm > 0.0f) {
        recipNorm = 1.0f / recipNorm;
        ax *= recipNorm; ay *= recipNorm; az *= recipNorm;

        _2q0 = 2.0f * q0; _2q1 = 2.0f * q1; _2q2 = 2.0f * q2; _2q3 = 2.0f * q3;
        _4q0 = 4.0f * q0; _4q1 = 4.0f * q1; _4q2 = 4.0f * q2;
        _8q1 = 8.0f * q1; _8q2 = 8.0f * q2;
        float q0q0 = q0 * q0, q1q1 = q1 * q1, q2q2 = q2 * q2, q3q3 = q3 * q3;

        s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q3q3 - _2q3 * ay;
        s1 = _4q1 * q1q1 + _8q1 * q2q2 + _4q1 * q3q3 - _2q3 * az + _4q0 * q2 * ax - _4q0 * q3 * ay + _4q0 * q1 * az;
        s2 = 4.0f * q0q0 * q2 - _2q0 * ax + 4.0f * q1q1 * q2 - _2q1 * ay;
        s3 = 4.0f * q0q0 * q3 - _2q0 * ay + 4.0f * q1q1 * q3 - _2q1 * az;

        recipNorm = sqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3);
        if (recipNorm > 0.0f) {
            recipNorm = 1.0f / recipNorm;
            s0 *= recipNorm; s1 *= recipNorm; s2 *= recipNorm; s3 *= recipNorm;
            qDot1 -= beta * s0; qDot2 -= beta * s1; qDot3 -= beta * s2; qDot4 -= beta * s3;
        }
    }

    q0 += qDot1 * (1.0f / sampleFreq);
    q1 += qDot2 * (1.0f / sampleFreq);
    q2 += qDot3 * (1.0f / sampleFreq);
    q3 += qDot4 * (1.0f / sampleFreq);

    recipNorm = sqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    if (recipNorm > 0.0f) {
        recipNorm = 1.0f / recipNorm;
        q0 *= recipNorm; q1 *= recipNorm; q2 *= recipNorm; q3 *= recipNorm;
    }

    roll = atan2(2.0f * (q0 * q1 + q2 * q3), 1.0f - 2.0f * (q1 * q1 + q2 * q2));
    pitch = asin(2.0f * (q0 * q2 - q3 * q1));
    yaw = atan2(2.0f * (q0 * q3 + q1 * q2), 1.0f - 2.0f * (q2 * q2 + q3 * q3));
    roll *= 180.0f / PI;
    pitch *= 180.0f / PI;
    yaw *= 180.0f / PI;
}

// ================= CHECKSUM =================
uint8_t calcChecksum() {
    uint8_t sum = 0;
    uint8_t* ptr = (uint8_t*)&txData;
    for (int i = 0; i < sizeof(txData) - 1; i++) {
        sum ^= ptr[i];
    }
    return sum;
}

// ================= CALIBRATION =================
void calibrate() {
    readMPU();
    float ax = mpuData.ax / 16384.0f;
    float ay = mpuData.ay / 16384.0f;
    float az = mpuData.az / 16384.0f;
    float gx = mpuData.gx / 131.0f;
    float gy = mpuData.gy / 131.0f;
    float gz = mpuData.gz / 131.0f;

    for (int i = 0; i < 100; i++) {
        madgwickUpdate(gx, gy, gz, ax, ay, az);
        delay(5);
    }

    rollOffset = roll;
    pitchOffset = pitch;
    yawOffset = yaw;
}

// ================= SETUP =================
void setup() {
    Serial.begin(BAUD_RATE);

    pinMode(HOME_PIN, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(HOME_PIN), []() {
        calibrateRequested = true;
    }, FALLING);

    initMPU();

    // Начальная калибровка
    Serial.println("Калибровка...");
    delay(2000);
    calibrate();

    Serial.println("Готово! Оптоволоконный трекинг.");
}

// ================= LOOP =================
void loop() {
    if (calibrateRequested) {
        calibrateRequested = false;
        calibrate();
        Serial.println("Calibrated");
        delay(500);
    }

    readMPU();

    float ax = mpuData.ax / 16384.0f;
    float ay = mpuData.ay / 16384.0f;
    float az = mpuData.az / 16384.0f;
    float gx = mpuData.gx / 131.0f;
    float gy = mpuData.gy / 131.0f;
    float gz = mpuData.gz / 131.0f;

    madgwickUpdate(gx, gy, gz, ax, ay, az);

    float finalRoll = roll - rollOffset;
    float finalPitch = pitch - pitchOffset;
    float finalYaw = yaw - yawOffset;

    int16_t servoYaw = constrain((int16_t)(finalYaw + 90), 0, 180);
    int16_t servoPitch = constrain((int16_t)(-finalPitch + 90), 0, 180);

    bool inHome = (abs(servoYaw - SERVO_CENTER) < HOME_THRESHOLD) &&
                  (abs(servoPitch - SERVO_CENTER) < HOME_THRESHOLD);

    txData.yaw = (uint16_t)(servoYaw / 1.8);
    txData.pitch = (uint16_t)(servoPitch / 1.8);
    txData.home = inHome ? 1 : 0;
    txData.checksum = calcChecksum();

    // Отправка через оптоволокно
    Serial.write((uint8_t*)&txData, sizeof(txData));

    // Дебаг
    // Serial.print("Y:"); Serial.print(servoYaw);
    // Serial.print(" P:"); Serial.println(servoPitch);

    delay(10);
}