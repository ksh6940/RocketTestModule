#include "HX711.h"
#include <CH376msc.h>
#include <SoftwareSerial.h>
#include <MPU6050.h>

#define calibration_factor -7050.0
#define DOUT 3
#define CLK 2

HX711 scale(DOUT, CLK);
SoftwareSerial HM10(8, 7);
SoftwareSerial CH376Serial(10, 11);
CH376msc usb(CH376Serial);

const int Led = 3;
bool sensorRunning = false;
bool debuggerMode = false;
bool horizonMode = false;

MPU6050 mpu;

void setup() {
  Serial.begin(9600);
  pinMode(Led, OUTPUT);

  scale.set_scale(calibration_factor);
  scale.tare();

  mpu.initialize();
  if (!mpu.testConnection()) {
    Serial.println("MPU6050 연결 실패");
    while (1);
  }
  Serial.println("MPU6050 연결 성공");

  HM10.begin(9600);
  CH376Serial.begin(9600);

  if (!usb.init()) {
    Serial.println("USB 모듈 초기화 실패");
    while (1);
  }
  Serial.println("USB 모듈 초기화 성공");

  if (!usb.mount()) {
    Serial.println("USB 연결 실패");
    while (1);
  }
  Serial.println("USB 연결 성공");
}

void handleDebuggerModeCommands(const String& command) {
  if (command == "Exit Debug") {
    debuggerMode = false;
    HM10.println("디버거 모드 종료");
  } else if (command == "Restart") {
    HM10.println("재시작 중...");
    setup();
  } else if (command == "Horizon Mode") {
    horizonMode = true;
    HM10.println("수평 측정 모드 진입");
  } else if (command == "Exit Horizon Mode") {
    horizonMode = false;
    HM10.println("수평 측정 모드 탈출");
  } else {
    HM10.println("디버거 모드에서 유효하지 않은 명령어입니다.");
  }
}

void handleNormalModeCommands(const String& command) {
  if (command == "Setting Zero-point") {
    scale.tare();
    HM10.println("영점 설정 완료");
  } else if (command == "Start Sensor") {
    sensorRunning = true;
    HM10.println("센서 시작");
  } else if (command == "Stop Sensor") {
    sensorRunning = false;
    HM10.println("센서 중지");
  } else if (command == "Debugger Mode") {
    debuggerMode = true;
    HM10.println("디버거 모드 진입");
  } else {
    HM10.println("유효하지 않은 명령어입니다.");
  }
}

void handleSensorData() {
  float weightGrams = scale.get_units() * 1000;

  HM10.print("Weight: ");
  HM10.print(weightGrams, 1);
  HM10.println(" g");

  if (usb.checkConnection()) {
    if (usb.openFile("ThrustData.csv", FILE_WRITE)) {
      String data = String(weightGrams, 1) + "\n";
      usb.writeFile(data.c_str(), data.length());
      usb.closeFile();
      HM10.println("데이터 쓰기 완료");
    } else {
      HM10.println("파일 열기 실패");
    }
  } else {
    HM10.println("USB 연결 해제");
  }
}

void handleHorizonMode() {
  VectorInt16 gyro;
  mpu.getRotation(&gyro.x, &gyro.y, &gyro.z);

  HM10.print("Gyroscope X: ");
  HM10.print(gyro.x);
  HM10.print(" Y: ");
  HM10.print(gyro.y);
  HM10.print(" Z: ");
  HM10.println(gyro.z);
}

void loop() {
  if (HM10.available()) {
    digitalWrite(Led, HIGH);

    String command = HM10.readStringUntil('\n').trim();

    if (debuggerMode) {
      handleDebuggerModeCommands(command);
    } else {
      handleNormalModeCommands(command);
    }
  }

  if (sensorRunning) {
    handleSensorData();
    delay(1000);
  } else if (horizonMode) {
    handleHorizonMode();
    delay(1000);
  } else {
    digitalWrite(Led, LOW);
    HM10.println("센서 실행 중지");
  }
}
