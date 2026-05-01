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
 *
 * v2.0 - Security fixes applied
 */

#include <Servo.h>
#include <avr/wdt.h>

// ================= CONFIG =================
const int OPTICAL_RX_PIN = 0;  // Hardware RX на Nano
const int BAUD_RATE = 115200;

const int SERVO_PAN_PIN = 6;
const int SERVO_TILT_PIN = 5;

const int PAN_MIN = 10, PAN_MAX = 170;
const int TILT_MIN = 20, TILT_MAX = 160;
const int SERVO_CENTER = 90;
const int SERVO_RANGE = 100;

const uint32_t TIMEOUT_MS = 500;
const int LED_PIN = 13;
const uint8_t MAX_YAW_VALUE = 100;
const uint8_t MAX_PITCH_VALUE = 100;
const uint32_t MAX_ERRORS = 50;

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
uint8_t syncState = 0;  // Для синхронизации пакетов

// ================= CHECKSUM =================
uint8_t calcChecksum() {
    uint8_t sum = 0;
    uint8_t* ptr = (uint8_t*)&rxData;
    for (int i = 0; i < sizeof(rxData) - 1; i++) {
        sum ^= ptr[i];
    }
    return sum;
}

// ================= DATA VALIDATION =================
bool validateData() {
    // Проверка чексуммы
    if (rxData.checksum != calcChecksum()) {
        return false;
    }

    // Проверка диапазона данных
    if (rxData.yaw > MAX_YAW_VALUE || rxData.pitch > MAX_PITCH_VALUE) {
        return false;
    }

    // Проверка флага home
    if (rxData.home > 1) {
        return false;
    }

    return true;
}

// ================= SAFE SERVO WRITE =================
void safeServoWrite(Servo& servo, int value, int minVal, int maxVal) {
    value = constrain(value, minVal, maxVal);
    servo.write(value);
}

// ================= SETUP =================
void setup() {
    Serial.begin(BAUD_RATE);

    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, LOW);

    servoPan.attach(SERVO_PAN_PIN);
    servoTilt.attach(SERVO_TILT_PIN);

    safeServoWrite(servoPan, SERVO_CENTER, PAN_MIN, PAN_MAX);
    safeServoWrite(servoTilt, SERVO_CENTER, TILT_MIN, TILT_MAX);

    wdt_enable(WDTO_2S);

    Serial.println("Optical RX ready");
}

// ================= LOOP =================
void loop() {
    wdt_reset();

    // Читаем доступные байты
    if (Serial.available() >= sizeof(rxData)) {
        // Пытаемся прочитать пакет
        size_t bytesRead = Serial.readBytes((char*)&rxData, sizeof(rxData));

        if (bytesRead == sizeof(rxData)) {
            if (validateData()) {
                lastPacketTime = millis();
                digitalWrite(LED_PIN, LOW);
                errorCount = 0;

                targetPan = map(rxData.yaw, 0, SERVO_RANGE, PAN_MIN, PAN_MAX);
                targetTilt = map(rxData.pitch, 0, SERVO_RANGE, TILT_MIN, TILT_MAX);
            } else {
                errorCount++;

                // Слишком много ошибок — очищаем буфер
                if (errorCount > MAX_ERRORS) {
                    while (Serial.available()) Serial.read();
                    errorCount = 0;
                }
            }
        } else {
            // Неверный размер пакета — сброс
            while (Serial.available()) Serial.read();
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

    safeServoWrite(servoPan, currentPan, PAN_MIN, PAN_MAX);
    safeServoWrite(servoTilt, currentTilt, TILT_MIN, TILT_MAX);

    delay(10);
}