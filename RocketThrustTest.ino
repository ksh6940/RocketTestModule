#include "HX711.h"
#include <SoftwareSerial.h>
#include <LiquidCrystal_I2C.h>
#include <Wire.h>
#include <SD.h>
#include <SPI.h>

HX711 Scale;

const uint8_t dataPin = 6;
const uint8_t clockPin = 7;
const uint8_t blueLED = 8;  // 블루 LED 핀

SoftwareSerial BTSerial(4, 5);       // Bluetooth 시리얼
LiquidCrystal_I2C lcd(0x27, 16, 2);  // LCD 디스플레이 설정

File myFile;
bool debuggingMode = false;
bool sensorActive = false;

void setup() {
  Serial.begin(115200);
  pinMode(blueLED, OUTPUT);
  digitalWrite(blueLED, LOW);

  // HX711 초기화
  Scale.begin(dataPin, clockPin);
  Scale.set_scale(127.15);
  Scale.tare();

  // Bluetooth 초기화
  BTSerial.begin(9600);

  // Display Module
  lcd.init();
  lcd.backlight();
  lcd.setCursor(0, 0);
  lcd.print("Initializing...");

  // SD Module 초기화
  if (!SD.begin(4)) {
    Serial.println("SD 카드 초기화 실패");
    while (1)
      ;
  }
  Serial.println("SD 카드 초기화 성공");

  // 영점 조절 및 페어링 대기
  calibrate();
  delay(2000);  // 잠시 대기

  // HM-10 블루투스 연결 대기
  waitForBluetoothConnection();
}

void loop() {
  if (BTSerial.available()) {
    String command = BTSerial.readStringUntil('\n');
    Command(command);
  }

  if (sensorActive) {
    float weight = Scale.get_units(10);
    if (myFile) {
      myFile.print(weight);
      myFile.println(" g");
      if (debuggingMode) {
        sendDebugMessage("Weight recorded: " + String(weight) + " g");
      }
    }
  }
}

void waitForBluetoothConnection() {
  unsigned long startMillis = millis();
  bool connected = false;

  while (millis() - startMillis < 30000) {  // 30초 동안 연결 대기
    if (BTSerial.available()) {
      String response = BTSerial.readStringUntil('\n');
      if (response.indexOf("CONNECTED") >= 0) {
        connected = true;
        break;
      }
    }
    delay(1000);  // 1초 대기
  }

  if (connected) {
    digitalWrite(blueLED, HIGH);  // 블루 LED 켜짐
    Serial.println("Bluetooth 연결 성공");
  } else {
    blinkLED(5);  // LED가 5초 간격으로 5번 깜빡임
    Serial.println("Bluetooth 연결 실패");
  }
}

void blinkLED(int times) {
  for (int i = 0; i < times; i++) {
    digitalWrite(blueLED, HIGH);
    delay(500);
    digitalWrite(blueLED, LOW);
    delay(500);
  }
}

void Command(String command) {
  if (command == "Setting ZeroPoint") {
    calibrate();
    sendFeedback("영점 조절 완료");
  } else if (command == "Start Sensor") {
    handleStartSensor();
  } else if (command == "Stop Sensor") {
    handleStopSensor();
  } else if (command == "File Open") {
    handleFileOpen();
  } else if (command == "Debugger Mode On") {
    handleDebuggerModeOn();
  } else if (command == "Debugger Mode Off") {
    handleDebuggerModeOff();
  } else if (command == "Restart Module") {
    setup();
  } else {
    handleUnknownCommand();
  }
}

void handleStartSensor() {
  if (!sensorActive) {
    myFile = SD.open("ThrustData.csv", FILE_WRITE);
    if (myFile) {
      sendFeedback("측정 시작 및 파일 열기");
      sensorActive = true;
    } else {
      sendFeedback("파일 열기 실패");
    }
  } else {
    sendFeedback("이미 측정이 시작되었습니다.");
  }
}

void handleStopSensor() {
  if (sensorActive) {
    myFile.close();
    sendFeedback("측정 중지 및 파일 닫기");
    sensorActive = false;
  } else {
    sendFeedback("측정이 활성화되지 않았습니다.");
  }
}

void handleFileOpen() {
  if (myFile) {
    myFile.close();  // 파일을 열기 전에 현재 열려 있는 파일을 닫음
  }
  myFile = SD.open("ThrustData.csv");
  if (myFile) {
    sendFeedback("파일 열기 성공");
  } else {
    sendFeedback("파일 열기 실패");
  }
}

void handleDebuggerModeOn() {
  debuggingMode = true;
  sendFeedback("디버깅 모드 활성화");
}

void handleDebuggerModeOff() {
  debuggingMode = false;
  sendFeedback("디버깅 모드 비활성화");
}

void handleUnknownCommand() {
  sendFeedback("존재하지 않는 명령어입니다.");
}

void calibrate() {
  Serial.println("로드셀에서 모든 물체를 제거하고, 영점을 설정하려면 엔터를 누르세요.");
  while (Serial.available()) Serial.read();
  while (Serial.available() == 0)
    ;
  Serial.println("영점을 설정 중입니다...");
  Scale.tare(20);                        // 20번 측정하여 평균값으로 영점을 설정
  uint32_t offset = Scale.get_offset();  // 설정된 영점 값을 가져옴
  Serial.print("설정된 OFFSET (영점 조절값): ");
  Serial.println(offset);
  Serial.print("\n프로젝트에서 다음을 사용하세요:\n");
  Serial.print("Scale.set_offset(");
  Serial.print(offset);
  Serial.print(");\n");
  Serial.println("\n\n영점 조절이 완료되었습니다. 저울이 준비되었습니다.");
}

void sendFeedback(const String& message) {
  Serial.println(message);
  BTSerial.println(message);
}

void sendDebugMessage(const String& message) {
  if (debuggingMode) {
    BTSerial.println(message);
  }
}
