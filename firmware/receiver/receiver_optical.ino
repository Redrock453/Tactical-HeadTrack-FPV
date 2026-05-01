/*
 * Tactical HeadTrack FPV - Optical Fiber Receiver
 *
 * Приемник с оптоволоконной связью.
 *
 * Wiring (Toslink модуль TORX173):
 *   TORX173: VCC -> 5V, GND -> GND, OUT -> RX (D0)
 *
 * Wiring (Vishay VLR11):
 *   VLR11: VCC -> 5V, GND -> GND, OUT -> RX (D0) через резистор
 */

#include <Servo.h>

// ================= CONFIG =================
const int OPTICAL_RX_PIN = 0;  // Hardware RX на Nano
const int BAUD_RATE = 115200;

const int SERVO_PAN_PIN = 6;
const int SERVO_TILT_PIN = 5;

const int PAN_MIN = 10, PAN_MAX = 170;
const int TILT_MIN = 20, TILT_MAX = 160;
const int SERVO_CENTER = 90;

const uint32_t TIMEOUT_MS = 500;
const int LED_PIN = 13;

// ================= VARIABLES =================
struct __attribute__((packed)) {
    uint16_t yaw;
    uint16_t pitch;
    uint8_t home;
    uint8_t checksum;
} rxData;

Servo servoPan, servoTilt;

int targetPan = SERVO_CENTER, targetTilt = SERVO_CENTER;
int currentPan = SERVO_CENTER, currentTilt = SERVO_CENTER;

uint32_t lastPacketTime = 0;
uint32_t errorCount = 0;

// ================= CHECKSUM =================
uint8_t calcChecksum() {
    uint8_t sum = 0;
    uint8_t* ptr = (uint8_t*)&rxData;
    for (int i = 0; i < sizeof(rxData) - 1; i++) {
        sum ^= ptr[i];
    }
    return sum;
}

// ================= SYNC DETECTION =================
// Простой синхробайт для поиска начала пакета
const uint8_t SYNC_PATTERN = 0xAA;
bool packetValid = false;

// ================= SETUP =================
void setup() {
    Serial.begin(BAUD_RATE);

    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, LOW);

    servoPan.attach(SERVO_PAN_PIN);
    servoTilt.attach(SERVO_TILT_PIN);

    servoPan.write(SERVO_CENTER);
    servoTilt.write(SERVO_CENTER);

    Serial.println("Optical RX ready");
}

// ================= LOOP =================
void loop() {
    if (Serial.available() >= sizeof(rxData)) {
        // Читаем пакет
        Serial.readBytes((char*)&rxData, sizeof(rxData));

        // Проверка контрольной суммы
        if (rxData.checksum == calcChecksum()) {
            lastPacketTime = millis();
            digitalWrite(LED_PIN, LOW);

            targetPan = (int)(rxData.yaw * 1.8f);
            targetTilt = (int)(rxData.pitch * 1.8f);

            targetPan = constrain(targetPan, PAN_MIN, PAN_MAX);
            targetTilt = constrain(targetTilt, TILT_MIN, TILT_MAX);

            errorCount = 0;
        } else {
            errorCount++;
            // Сброс буфера при частых ошибках
            if (errorCount > 10) {
                while (Serial.available()) Serial.read();
                errorCount = 0;
            }
        }
    }

    // Таймаут
    if (millis() - lastPacketTime > TIMEOUT_MS) {
        digitalWrite(LED_PIN, HIGH);
        targetPan = SERVO_CENTER;
        targetTilt = SERVO_CENTER;
    }

    // Плавное движение
    if (currentPan < targetPan) currentPan = min(currentPan + 1, targetPan);
    if (currentPan > targetPan) currentPan = max(currentPan - 1, targetPan);
    if (currentTilt < targetTilt) currentTilt = min(currentTilt + 1, targetTilt);
    if (currentTilt > targetTilt) currentTilt = max(currentTilt - 1, targetTilt);

    servoPan.write(currentPan);
    servoTilt.write(currentTilt);

    delay(10);
}