/***************************************************************************
 * Программа для считывания значений с двух джойстиков, потенциометра и
 * переключателя. Полученные данные выводятся в Serial Monitor в виде
 * JSON-подобной строки.
 ***************************************************************************/

// --------------------- Определение пинов для устройств ---------------------

// *** Джойстик 1 ***
const int joy1XPin = A0;
const int joy1YPin = A1;

const int joy1ButtonPin = 6;

// *** Джойстик 2 ***

const int joy2XPin = A5;
const int joy2YPin = A4;
const int joy2ButtonPin = 9;

// *** Потенциометр ***
const int potentiometerPin = A3;

// *** Переключатель ***
const int switchPin = 5;

// --------------------- Инициализация и настройка пинов ---------------------

void setup() {
  Serial.begin(115200);

  pinMode(joy1ButtonPin, INPUT);
  pinMode(joy2ButtonPin, INPUT);

  pinMode(switchPin, INPUT_PULLUP);

  pinMode(joy1XPin, INPUT);
  pinMode(joy1YPin, INPUT);
  pinMode(joy2XPin, INPUT);
  pinMode(joy2YPin, INPUT);

  digitalWrite(joy1ButtonPin, HIGH);
  digitalWrite(joy2ButtonPin, HIGH);
}

// --------------------- Основной цикл программы ---------------------

void loop() {
  // Считываем значения с осей первого джойстика
  // Для оси X делаем инверсию значений (1023 - значение), чтобы изменить
  // направление увеличения
  int joy1X = 1023 - analogRead(joy1XPin);
  int joy1Y = analogRead(joy1YPin);

  // Считываем состояние кнопки первого джойстика, используя отрицание, чтобы
  // получить true при нажатии
  bool joy1Button = !digitalRead(joy1ButtonPin);

  // Считываем значения с осей второго джойстика
  int joy2X = 1023 - analogRead(joy2XPin); // Инверсия для оси X
  int joy2Y = analogRead(joy2YPin);

  // Считываем состояние кнопки второго джойстика (true, если нажата)
  bool joy2Button = !digitalRead(joy2ButtonPin);

  // Считываем значение потенциометра (диапазон от 0 до 1023)
  int potValue = analogRead(potentiometerPin);

  // Считываем состояние переключателя
  // Так как переключатель подключён через INPUT_PULLUP, то
  // неактивное состояние = HIGH, активное = LOW
  bool switchState = digitalRead(switchPin);

  // Формируем строку в формате JSON для вывода данных
  Serial.print("{");
  Serial.print("\"joy1\": {\"x\": ");
  Serial.print(joy1X);
  Serial.print(", \"y\": ");
  Serial.print(joy1Y);
  Serial.print(", \"button\": ");

  // Выводим состояние кнопки как строку "true" или "false"
  Serial.print(joy1Button ? "true" : "false");
  Serial.print("}, ");

  Serial.print("\"joy2\": {\"x\": ");
  Serial.print(joy2X);
  Serial.print(", \"y\": ");
  Serial.print(joy2Y);
  Serial.print(", \"button\": ");
  Serial.print(joy2Button ? "true" : "false");
  Serial.print("}, ");

  Serial.print("\"potentiometer\": ");
  Serial.print(potValue);
  Serial.print(", \"switch\": ");
  Serial.print(switchState ? "true" : "false");
  Serial.println("}");

  // Небольшая задержка 50 миллисекунд для стабилизации частоты обновления
  delay(50);
}