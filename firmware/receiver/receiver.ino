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
 */

#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <Servo.h>

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

// Параметры сглаживания
const int SMOOTHING_STEPS = 3;  // Чем больше, тем плавнее

// ================= VARIABLES =================
struct __attribute__((packed)) {
    uint16_t yaw;   // 0-100 * 1.8 = 0-180
    uint16_t pitch; // 0-100 * 1.8 = 0-180
    uint8_t home;  // 1 если в центре
} rxData;

Servo servoPan;
Servo servoTilt;

int targetPan = SERVO_CENTER;
int targetTilt = SERVO_CENTER;
int currentPan = SERVO_CENTER;
int currentTilt = SERVO_CENTER;

uint32_t lastPacketTime = 0;
const uint32_t TIMEOUT_MS = 500;  // Таймаут сигнала

// Индикация потери сигнала (LED)
const int LED_PIN = 13;  // Встроенный LED на Nano

// ================= SETUP =================
void setup() {
    Serial.begin(115200);

    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, LOW);

    servoPan.attach(SERVO_PAN_PIN);
    servoTilt.attach(SERVO_TILT_PIN);

    // Центральная позиция при старте
    servoPan.write(SERVO_CENTER);
    servoTilt.write(SERVO_CENTER);

    radio.begin();
    radio.setPALevel(RF24_PA_LOW);
    radio.setDataRate(RF24_250KBPS);
    radio.openReadingPipe(0, address);
    radio.startListening();

    Serial.println("Приемник готов. Жду данные...");
}

// ================= LOOP =================
void loop() {
    if (radio.available()) {
        radio.read(&rxData, sizeof(rxData));
        lastPacketTime = millis();
        digitalWrite(LED_PIN, LOW);  // Сигнал есть — LED выключен

        // Конвертируем из формата передатчика
        targetPan = (int)(rxData.yaw * 1.8f);
        targetTilt = (int)(rxData.pitch * 1.8f);

        // Ограничения
        targetPan = constrain(targetPan, PAN_MIN, PAN_MAX);
        targetTilt = constrain(targetTilt, TILT_MIN, TILT_MAX);
    }

    // Проверка таймаута
    if (millis() - lastPacketTime > TIMEOUT_MS) {
        digitalWrite(LED_PIN, HIGH);  // Потеря сигнала — мигаем

        // Плавный возврат в центр при потере сигнала
        targetPan = SERVO_CENTER;
        targetTilt = SERVO_CENTER;
    }

    // Плавное движение к цели
    if (currentPan < targetPan) currentPan = min(currentPan + 1, targetPan);
    if (currentPan > targetPan) currentPan = max(currentPan - 1, targetPan);
    if (currentTilt < targetTilt) currentTilt = min(currentTilt + 1, targetTilt);
    if (currentTilt > targetTilt) currentTilt = max(currentTilt - 1, targetTilt);

    servoPan.write(currentPan);
    servoTilt.write(currentTilt);

    // Debug
    // Serial.print("Pan: "); Serial.print(currentPan);
    // Serial.print(" Tilt: "); Serial.println(currentTilt);

    delay(10);
}