# Модуль

В этой папке находится код для CAN-модуля ровера, отвечающий за обмен данными по шине CAN и управление исполнительными узлами.

- **can_receiver.ino** – скетч для микроконтроллера, управляющего CAN-интерфейсом:
  - Обеспечивает обмен данными с другими узлами системы через шину CAN.
  - Передаёт команды управления на механизмы привода и получает данные от датчиков.
  - Интегрируется с общей системой управления ровером для обеспечения синхронной работы всех модулей.

## Сборка и загрузка

1. Откройте файл `can_receiver.ino` в Arduino IDE или совместимой среде.
2. Подключите соответствующий микроконтроллер или CAN-модуль к ПК.
3. Проверьте конфигурацию шины CAN и установите верные параметры (скорость обмена, фильтры и т.д.).
4. Скомпилируйте и загрузите прошивку в устройство.

## Зависимости и аппаратные подключения

- Микроконтроллер с поддержкой CAN-интерфейса (или внешний CAN-трансивер)
- Корректно установите проводку шины CAN в соответствии с технической документацией вашего модуля.
- Убедитесь, что конфигурация (например, скорость CAN) соответствует настройкам всей системы.

## Примечания

- После загрузки прошивки рекомендуется провести тестирование обмена сообщениями по шине CAN с другими узлами системы.
