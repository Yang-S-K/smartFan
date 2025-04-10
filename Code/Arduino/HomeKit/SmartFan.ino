#include <WiFi.h>
#include "HomeSpan.h"

// **Wi-Fi 設定**
const char *ssid = "yskedf"; //Wi-Fi 名稱
const char *password = "0905888636"; //Wi-Fi 密碼

#define UART_TX 17  // TX (NUCLEO RX)
#define UART_RX 16  // RX (NUCLEO TX)

// **函式前置宣告**
void sendCommand();

// **HomeKit 配件識別服務**
struct IdentifyFan : Service::AccessoryInformation {
  IdentifyFan() : Service::AccessoryInformation() {
    new Characteristic::Identify();
    new Characteristic::Name("智能風扇");
  }
};

// **狀態變數**
int fanState = 0;
int fanSpeed = 50;
int horizontalAngle = 50;
int verticalAngle = 50;
unsigned long lastSendTime = 0;
float temperature = 25.0;  // 初始溫度

// **風速控制**
struct FanControl : Service::Fan {
  SpanCharacteristic *active;
  SpanCharacteristic *speed;

  FanControl() : Service::Fan() {
    new Characteristic::Name("風速"); 
    active = new Characteristic::Active(1); 
    speed = new Characteristic::RotationSpeed(50);

    Serial.println("HomeKit 風速控制已啟動");
  }

  void loop() {
      fanState = active->getVal();
      fanSpeed = speed->getVal();
  }
};

// **左右旋轉控制**
struct HorizontalRotationControl : Service::WindowCovering {
  SpanCharacteristic *horizontal;
  SpanCharacteristic *currentHorizontal;

  HorizontalRotationControl() : Service::WindowCovering() {
    new Characteristic::Name("左右");
    currentHorizontal = new Characteristic::CurrentPosition(50);
    horizontal = new Characteristic::TargetPosition(50);
    horizontal->setRange(0, 100);

    Serial.println("HomeKit 左右旋轉控制已啟動");
  }

  void loop() {
      horizontalAngle = horizontal->getVal();
  }
};

// **上下旋轉控制**
struct VerticalRotationControl : Service::WindowCovering {
  SpanCharacteristic *vertical;
  SpanCharacteristic *currentVertical;

  VerticalRotationControl() : Service::WindowCovering() {
    new Characteristic::Name("上下");
    currentVertical = new Characteristic::CurrentPosition(50);
    vertical = new Characteristic::TargetPosition(50);
    vertical->setRange(0, 100);

    Serial.println("HomeKit 上下旋轉控制已啟動");
  }

  void loop() {
      verticalAngle = vertical->getVal();
  }
};

// **透過 UART 發送完整狀態**
void sendCommand() {
  String command = String(fanState) + "," + 
                   String(fanSpeed) + "," + 
                   String(horizontalAngle) + "," + 
                   String(verticalAngle);

  Serial2.println(command);
  Serial.print("發送 UART 指令: ");
  Serial.println(command);
}


FanControl *fanService;  
HorizontalRotationControl *hService;  
VerticalRotationControl *vService;
SpanCharacteristic *tempSensor;
// **Wi-Fi 連線**
void setup() {
  Serial.begin(115200);
  Serial2.begin(115200, SERIAL_8N1, UART_RX, UART_TX);

  WiFi.begin(ssid, password);
  Serial.print("連線至 Wi-Fi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWi-Fi 連線成功！");
  Serial.print("IP 地址: ");
  Serial.println(WiFi.localIP());

  // **啟動 HomeSpan（HomeKit 配置）**
  homeSpan.begin(Category::Fans, "智能風扇");

  // **建立 HomeKit 配件**
  new SpanAccessory();
  new IdentifyFan();
  fanService = new FanControl();  // **存到全域變數**

  new SpanAccessory();
  new IdentifyFan();
  hService = new HorizontalRotationControl();  // **存到全域變數**
  vService = new VerticalRotationControl();  // **存到全域變數**

  new SpanAccessory();  
  new IdentifyFan();
  SpanService *temperatureService = new Service::TemperatureSensor();
  tempSensor = new Characteristic::CurrentTemperature(temperature);
  tempSensor->setRange(-40, 100);  // 設定 HomeKit 支援的溫度範圍
}

void receiveCommand() {
  if (Serial2.available()) {
    String receivedData = Serial2.readStringUntil('\n');  // 讀取 UART 數據直到換行符
    receivedData.trim();  // 移除多餘的換行或空格

    int newFanState, newFanSpeed, newHorizontalAngle, newVerticalAngle;
    float newTemperature;
    // 解析數據
    if (sscanf(receivedData.c_str(), "%d,%d,%d,%d,%f",
               &newFanState, &newFanSpeed,
               &newHorizontalAngle, &newVerticalAngle, &newTemperature) == 5) {
      // **確保 fanState 只會是 0 或 1**
      if (newFanState == 0 || newFanState == 1) {
        fanState = newFanState;
      }

      fanSpeed = constrain(newFanSpeed, 0, 100);
      horizontalAngle = constrain(newHorizontalAngle, 0, 100);
      verticalAngle = constrain(newVerticalAngle, 0, 100);
      temperature = constrain(newTemperature, -40, 100); // 限制溫度範圍

      // 正確更新 HomeKit 變數
      fanService->active->setVal(fanState);
      fanService->speed->setVal(fanSpeed);
      hService->horizontal->setVal(horizontalAngle);
      vService->vertical->setVal(verticalAngle);
      tempSensor->setVal(temperature);


      String reCommand = String(newFanState) + "," + 
                   String(newFanSpeed) + "," + 
                   String(newHorizontalAngle) + "," + 
                   String(newVerticalAngle) + "," +
                   String(newTemperature);
      Serial.print("收到 UART 指令: ");
      Serial.println(reCommand);
    } else {
      Serial.println("UART 數據解析失敗");
    }
  }
}



void loop() {
  homeSpan.poll();
  receiveCommand();  // **檢查 UART 是否有新數據**
  // **每秒傳送一次 UART 訊息**
  if (millis() - lastSendTime >= 100) {
    lastSendTime = millis();
    sendCommand();
  }
}
