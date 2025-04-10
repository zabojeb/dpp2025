// Джойстик 1
const int joy1XPin = A0;     // Аналоговый вход для оси X первого джойстика
const int joy1YPin = A1;     // Аналоговый вход для оси Y первого джойстика
const int joy1ButtonPin = 6; // Цифровой вход для кнопки первого джойстика

// Джойстик 2
const int joy2XPin = A5;     // Аналоговый вход для оси X второго джойстика
const int joy2YPin = A4;     // Аналоговый вход для оси Y второго джойстика
const int joy2ButtonPin = 9; // Цифровой вход для кнопки второго джойстика

// Потенциометр
const int potentiometerPin = A3; // Аналоговый вход для потенциометра

// Кнопка (переключатель)
const int switchPin = 5; // Цифровой вход для переключателя

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

void loop() {
  int joy1X = 1023 - analogRead(joy1XPin);
  int joy1Y = analogRead(joy1YPin);
  bool joy1Button = !digitalRead(joy1ButtonPin);

  int joy2X = 1023 - analogRead(joy2XPin);
  int joy2Y = analogRead(joy2YPin);
  bool joy2Button = !digitalRead(joy2ButtonPin);

  int potValue = analogRead(potentiometerPin);

  bool switchState = digitalRead(switchPin);

  Serial.print("{");
  Serial.print("\"joy1\": {\"x\": ");
  Serial.print(joy1X);
  Serial.print(", \"y\": ");
  Serial.print(joy1Y);
  Serial.print(", \"button\": ");
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

  delay(50);
}