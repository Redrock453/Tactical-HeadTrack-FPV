/*
 * Tactical HeadTrack FPV - Transmitter
 *
 * На голове пилота. Считывает MPU-6050, фильтрует данные,
 * отправляет углы на приемник по NRF24L01+.
 *
 * Wiring:
 *   MPU-6050: VCC -> 3.3V, GND -> GND, SCL -> A5, SDA -> A4
 *   NRF24L01+: VCC -> 3.3V, GND -> GND, CE -> D9, CSN -> D10, SCK -> D13, MOSI -> D11, MISO -> D12
 *   Home Button: D2 -> GND (внутренний pull-up)
 *
 * v2.0 - Security fixes applied
 */

#include <Wire.h>
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <avr/wdt.h>

// ================= CONFIG =================
const int HOME_PIN = 2;
const RF24 radio(9, 10);
const uint8_t address[6] = "HTFPV";

// Маджвик фильтр параметры
const float BETA = 0.1f;           // 0.0-1.0, меньше = медленнее реакция
const float SAMPLE_FREQ = 100.0f;

// MPU-6050 константы (±2g accel, ±250°/s gyro)
const float ACCEL_SCALE = 16384.0f;
const float GYRO_SCALE = 131.0f;
const int MPU_ADDR = 0x68;

// Серво пределы (в градусах)
const int SERVO_MIN = 0;
const int SERVO_MAX = 180;
const int SERVO_CENTER = 90;
const int SERVO_RANGE = 100;     // 0-100 для передачи (вместо 0-180)

// Порог для режима Home (в градусах)
const int HOME_THRESHOLD = 5;

// Валидация данных
const uint8_t MAX_YAW_VALUE = 100;
const uint8_t MAX_PITCH_VALUE = 100;

// ================= VARIABLES =================
struct {
    int16_t ax, ay, az;
    int16_t gx, gy, gz;
} mpuData;

struct __attribute__((packed)) {
    uint16_t yaw;    // 0-100
    uint16_t pitch;  // 0-100
    uint8_t home;    // 1 если в центре
    uint8_t checksum; // XOR чексумма
} txData;

// Маджвик фильтр переменные
float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;
float roll = 0.0f, pitch = 0.0f, yaw = 0.0f;
float rollOffset = 0.0f, pitchOffset = 0.0f, yawOffset = 0.0f;

// volatile для доступа из ISR
volatile bool calibrateRequested = false;

// Debounce таймер для кнопки
uint32_t lastButtonTime = 0;
const uint32_t DEBOUNCE_MS = 50;

// ================= MPU-6050 =================
void initMPU() {
    Wire.begin();
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x6B);  // PWR_MGMT_1
    Wire.write(0);     // Wake up
    Wire.endTransmission(true);

    // Настройка DLPF (фильтр низких частот) для сглаживания
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x1A);  // CONFIG
    Wire.write(0x03);  // DLPF = 42Hz
    Wire.endTransmission(true);

    // Усиление гироскопа: ±250 deg/s
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x1B);  // GYRO_CONFIG
    Wire.write(0x00);
    Wire.endTransmission(true);

    // Усиление акселерометра: ±2g
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x1C);  // ACCEL_CONFIG
    Wire.write(0x00);
    Wire.endTransmission(true);
}

void readMPU() {
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x3B);  // Начало данных
    Wire.endTransmission(false);
    Wire.requestFrom(MPU_ADDR, 14, true);

    mpuData.ax = (Wire.read() << 8 | Wire.read());
    mpuData.ay = (Wire.read() << 8 | Wire.read());
    mpuData.az = (Wire.read() << 8 | Wire.read());
    Wire.read(); Wire.read(); // temp
    mpuData.gx = (Wire.read() << 8 | Wire.read());
    mpuData.gy = (Wire.read() << 8 | Wire.read());
    mpuData.gz = (Wire.read() << 8 | Wire.read());
}

// ================= MADGWICK FILTER =================
void madgwickUpdate(float gx, float gy, float gz, float ax, float ay, float az) {
    float recipNorm;
    float s0, s1, s2, s3;
    float qDot1, qDot2, qDot3, qDot4;
    float _2q0, _2q1, _2q2, _2q3, _4q0, _4q1, _4q2, _8q1, _8q2, q0q0, q1q1, q2q2, q3q3;

    qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
    qDot2 = 0.5f * (q0 * gx + q2 * gz - q3 * gy);
    qDot3 = 0.5f * (q0 * gy - q1 * gz + q3 * gx);
    qDot4 = 0.5f * (q0 * gz + q1 * gy - q2 * gx);

    recipNorm = sqrt(ax * ax + ay * ay + az * az);
    if (recipNorm > 0.0f) {
        recipNorm = 1.0f / recipNorm;
        ax *= recipNorm;
        ay *= recipNorm;
        az *= recipNorm;

        _2q0 = 2.0f * q0; _2q1 = 2.0f * q1; _2q2 = 2.0f * q2; _2q3 = 2.0f * q3;
        _4q0 = 4.0f * q0; _4q1 = 4.0f * q1; _4q2 = 4.0f * q2;
        _8q1 = 8.0f * q1; _8q2 = 8.0f * q2;
        q0q0 = q0 * q0; q1q1 = q1 * q1; q2q2 = q2 * q2; q3q3 = q3 * q3;

        s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q3q3 - _2q3 * ay;
        s1 = _4q1 * q1q1 + _8q1 * q2q2 + _4q1 * q3q3 - _2q3 * az + _4q0 * q2 * ax - _4q0 * q3 * ay + _4q0 * q1 * az;
        s2 = 4.0f * q0q0 * q2 - _2q0 * ax + 4.0f * q1q1 * q2 - _2q1 * ay;
        s3 = 4.0f * q0q0 * q3 - _2q0 * ay + 4.0f * q1q1 * q3 - _2q1 * az;

        recipNorm = sqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3);
        if (recipNorm > 0.0f) {
            recipNorm = 1.0f / recipNorm;
            s0 *= recipNorm; s1 *= recipNorm; s2 *= recipNorm; s3 *= recipNorm;
            qDot1 -= BETA * s0; qDot2 -= BETA * s1; qDot3 -= BETA * s2; qDot4 -= BETA * s3;
        }
    }

    q0 += qDot1 * (1.0f / SAMPLE_FREQ);
    q1 += qDot2 * (1.0f / SAMPLE_FREQ);
    q2 += qDot3 * (1.0f / SAMPLE_FREQ);
    q3 += qDot4 * (1.0f / SAMPLE_FREQ);

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

    float ax = mpuData.ax / ACCEL_SCALE;
    float ay = mpuData.ay / ACCEL_SCALE;
    float az = mpuData.az / ACCEL_SCALE;
    float gx = mpuData.gx / GYRO_SCALE;
    float gy = mpuData.gy / GYRO_SCALE;
    float gz = mpuData.gz / GYRO_SCALE;

    for (int i = 0; i < 100; i++) {
        madgwickUpdate(gx, gy, gz, ax, ay, az);
        delay(5);
        wdt_reset(); // Сброс watchdog при долгой операции
    }

    rollOffset = roll;
    pitchOffset = pitch;
    yawOffset = yaw;
}

// ================= BUTTON ISR =================
void homeButtonISR() {
    uint32_t now = millis();
    if (now - lastButtonTime > DEBOUNCE_MS) {
        calibrateRequested = true;
        lastButtonTime = now;
    }
}

// ================= SETUP =================
void setup() {
    Serial.begin(115200);

    pinMode(HOME_PIN, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(HOME_PIN), homeButtonISR, FALLING);

    initMPU();

    radio.begin();
    radio.setPALevel(RF24_PA_LOW);
    radio.setDataRate(RF24_250KBPS);
    radio.enableDynamicPayloads();
    radio.openWritingPipe(address);
    radio.stopListening();

    // Watchdog 2 секунды
    wdt_enable(WDTO_2S);

    Serial.println("Калибровка... Держи голову ровно!");
    delay(2000);
    calibrate();

    Serial.println("Готово! Трекинг запущен.");
}

// ================= LOOP =================
void loop() {
    wdt_reset(); // Сброс watchdog

    if (calibrateRequested) {
        calibrateRequested = false;
        calibrate();
        Serial.println("Калибровка Home");
        delay(500);
    }

    readMPU();

    float ax = mpuData.ax / ACCEL_SCALE;
    float ay = mpuData.ay / ACCEL_SCALE;
    float az = mpuData.az / ACCEL_SCALE;
    float gx = mpuData.gx / GYRO_SCALE;
    float gy = mpuData.gy / GYRO_SCALE;
    float gz = mpuData.gz / GYRO_SCALE;

    madgwickUpdate(gx, gy, gz, ax, ay, az);

    float finalRoll = roll - rollOffset;
    float finalPitch = pitch - pitchOffset;
    float finalYaw = yaw - yawOffset;

    // Валидация и конвертация
    int16_t servoYaw = constrain((int16_t)(finalYaw + 90), 0, 180);
    int16_t servoPitch = constrain((int16_t)(-finalPitch + 90), 0, 180);

    bool inHome = (abs(servoYaw - SERVO_CENTER) < HOME_THRESHOLD) &&
                  (abs(servoPitch - SERVO_CENTER) < HOME_THRESHOLD);

    // Нормализация в диапазон 0-100
    txData.yaw = (uint16_t)map(servoYaw, 0, 180, 0, SERVO_RANGE);
    txData.pitch = (uint16_t)map(servoPitch, 0, 180, 0, SERVO_RANGE);
    txData.home = inHome ? 1 : 0;
    txData.checksum = calcChecksum();

    radio.write(&txData, sizeof(txData));

    delay(10);  // ~100Hz
}