#include <Servo.h>
#include <VBCoreG4_arduino_system.h>
#include <string.h>

static const int servoPins[7] = {
    PA7, PC5, PC1, PC2,
    PC3, PA4, PB0}; // Третий 360 // {PA7, PC5, PB0, PC2, PC3, PA4, PC1}; //
                    // ЛАСТ 360

// Массив объектов для семи сервоприводов
Servo servoMotor[7];

FDCAN_HandleTypeDef *hfdcan1;   // Указатель на конфигурацию CAN
CanFD *canfd;                   // Объект для управления CAN
FDCAN_RxHeaderTypeDef RxHeader; // Заголовок (header) входящего сообщения

void setup() {
  Serial.begin(115200);
  pinMode(LED2, OUTPUT);

  // Настройка тактирования и CAN FD
  SystemClock_Config();
  canfd = new CanFD();
  canfd->init();

  // Параметры CAN FD (1 Mbps номинал / 8 Mbps data)
  canfd->write_default_params(); // для FD
  canfd->apply_config();
  canfd->default_start();
  hfdcan1 = canfd->get_hfdcan();

  // Инициализация и привязка 7 сервоприводов
  for (int i = 0; i < 7; i++) {
    servoMotor[i].attach(servoPins[i]);
  }

  Serial.println("CAN-FD receiver started. Servos attached.");
}

void loop() {
  // Проверяем, есть ли во входном FIFO принятые сообщения
  while (HAL_FDCAN_GetRxFifoFillLevel(hfdcan1, FDCAN_RX_FIFO0) > 0) {
    // Ожидаем 7 float = 28 байт
    const int length = 28;
    uint8_t rxData[length];

    // Читаем сообщение из FIFO
    if (HAL_FDCAN_GetRxMessage(hfdcan1, FDCAN_RX_FIFO0, &RxHeader, rxData) !=
        HAL_OK) {
      Serial.println("CAN receive error!");
      Error_Handler();
    } else {
      // Проверяем, что ID соответствует нашему (0x700)
      if (RxHeader.Identifier == 0x700) {
        // Распаковываем 7 float
        float servoValues[7];
        memcpy(servoValues, rxData, length);

        // Первые 6 значений — углы для обычных серво (0..180),
        // седьмое — значение для серво непрерывного вращения
        for (int i = 0; i < 6; i++) {
          float angle = servoValues[i];
          // Для безопасности ограничим диапазон
          if (angle < 0.0f)
            angle = 0.0f;
          if (angle > 180.0f)
            angle = 180.0f;
          servoMotor[i].write(angle);
        }

        // Седьмой сервопривод: непрерывное вращение
        // ~90 = остановка; <90 вращение в одну сторону, >90 — в другую
        float speed = servoValues[6];
        if (speed < 0.0f)
          speed = 0.0f;
        if (speed > 180.0f)
          speed = 180.0f;
        servoMotor[6].write(speed);

        // (необязательно) Для отладки
        Serial.print("Got servo values: ");
        for (int i = 0; i < 7; i++) {
          Serial.print(servoValues[i], 1);
          Serial.print(i < 6 ? ", " : "\n");
        }
      }
    }
  }

  delay(10); // Небольшая задержка для стабильности
}