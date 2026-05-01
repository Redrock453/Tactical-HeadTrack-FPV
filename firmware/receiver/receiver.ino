/*
 * Tactical HeadTrack FPV - Receiver
 *
 * На дроне. Получает углы с передатчика,
 * управляет двумя сервоприводами (pan + tilt).
 *
 * Wiring:
 *   NRF24L01+: VCC -> 3.3V, GND -> GND, CE -> D9, CSN -> D10, SCK -> D13, MOSI -> D11, MISO -> D12
 *   Servo Pan: D6 -> Сигнал (красный -> 5V, коричневый -> GND)
 *   Servo Tilt: D5 -> Сигнал
 *
 * v2.0 - Security fixes applied
 */

#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <Servo.h>
#include <avr/wdt.h>

// ================= CONFIG =================
const int SERVO_PAN_PIN = 6;   // Горизонтальное вращение (yaw)
const int SERVO_TILT_PIN = 5;  // Вертикальное вращение (pitch)

const RF24 radio(9, 10);
const uint8_t address[6] = "HTFPV";

// Пределы сервоприводов (защита от ударов о крепление)
const int PAN_MIN = 10;
const int PAN_MAX = 170;
const int TILT_MIN = 20;   // Не смотреть вниз в землю
const int TILT_MAX = 160;

const int SERVO_CENTER = 90;
const int SERVO_RANGE = 100;  // Диапазон входных данных

// Таймаут и безопасность
const uint32_t TIMEOUT_MS = 500;
const int LED_PIN = 13;
const uint8_t MAX_YAW_VALUE = 100;
const uint8_t MAX_PITCH_VALUE = 100;

// ================= VARIABLES =================
struct __attribute__((packed)) {
    uint16_t yaw;    // 0-100
    uint16_t pitch;  // 0-100
    uint8_t home;    // 1 если в центре
    uint8_t checksum; // XOR чексумма
} rxData;

Servo servoPan;
Servo servoTilt;

int targetPan = SERVO_CENTER;
int targetTilt = SERVO_CENTER;
int currentPan = SERVO_CENTER;
int currentTilt = SERVO_CENTER;

uint32_t lastPacketTime = 0;
uint32_t errorCount = 0;
const uint32_t MAX_ERRORS = 50;

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
        errorCount++;
        return false;
    }

    // Проверка диапазона данных
    if (rxData.yaw > MAX_YAW_VALUE || rxData.pitch > MAX_PITCH_VALUE) {
        errorCount++;
        return false;
    }

    // Проверка флага home
    if (rxData.home > 1) {
        errorCount++;
        return false;
    }

    errorCount = 0;
    return true;
}

// ================= SAFE SERVO WRITE =================
void safeServoWrite(Servo& servo, int value, int minVal, int maxVal) {
    value = constrain(value, minVal, maxVal);
    servo.write(value);
}

// ================= SETUP =================
void setup() {
    Serial.begin(115200);

    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, LOW);

    servoPan.attach(SERVO_PAN_PIN);
    servoTilt.attach(SERVO_TILT_PIN);

    // Центральная позиция при старте
    safeServoWrite(servoPan, SERVO_CENTER, PAN_MIN, PAN_MAX);
    safeServoWrite(servoTilt, SERVO_CENTER, TILT_MIN, TILT_MAX);

    radio.begin();
    radio.setPALevel(RF24_PA_LOW);
    radio.setDataRate(RF24_250KBPS);
    radio.enableDynamicPayloads();
    radio.openReadingPipe(0, address);
    radio.startListening();

    // Watchdog 2 секунды
    wdt_enable(WDTO_2S);

    Serial.println("Приемник готов. Жду данные...");
}

// ================= LOOP =================
void loop() {
    wdt_reset(); // Сброс watchdog

    if (radio.available()) {
        radio.read(&rxData, sizeof(rxData));

        if (validateData()) {
            lastPacketTime = millis();
            digitalWrite(LED_PIN, LOW);  // Сигнал есть

            // Конвертируем из диапазона 0-100 в 0-180
            targetPan = map(rxData.yaw, 0, SERVO_RANGE, PAN_MIN, PAN_MAX);
            targetTilt = map(rxData.pitch, 0, SERVO_RANGE, TILT_MIN, TILT_MAX);
        } else {
            // Слишком много ошибок — сбрасываем буфер
            if (errorCount > MAX_ERRORS) {
                while (radio.available()) {
                    radio.read(&rxData, sizeof(rxData));
                }
                errorCount = 0;
            }
        }
    }

    // Проверка таймаута
    if (millis() - lastPacketTime > TIMEOUT_MS) {
        digitalWrite(LED_PIN, HIGH);  // Потеря сигнала

        // Плавный возврат в центр при потере сигнала
        targetPan = SERVO_CENTER;
        targetTilt = SERVO_CENTER;
    }

    // Плавное движение к цели (1 градус за итерацию)
    if (currentPan < targetPan) currentPan = min(currentPan + 1, targetPan);
    if (currentPan > targetPan) currentPan = max(currentPan - 1, targetPan);
    if (currentTilt < targetTilt) currentTilt = min(currentTilt + 1, targetTilt);
    if (currentTilt > targetTilt) currentTilt = max(currentTilt - 1, targetTilt);

    safeServoWrite(servoPan, currentPan, PAN_MIN, PAN_MAX);
    safeServoWrite(servoTilt, currentTilt, TILT_MIN, TILT_MAX);

    delay(10);
}