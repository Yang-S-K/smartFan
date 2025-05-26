#include "ESP32.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

extern UART_HandleTypeDef huart6;
static FanData fanData = {0, 0, 50, 50};  // 預設數值

// **解析 ESP32 傳送的數據**
void ESP32_UART_Receive(void) {
	uint8_t buffer[50];  // 用於接收數據的暫存區
	  uint8_t temp;
	  uint8_t index = 0;
	  memset(buffer, 0, sizeof(buffer));  // 清除 testBuffer
	  // **逐字讀取直到遇到 `\n` 或 `\r`**
	  while (index < sizeof(buffer) - 1) {
	      if (HAL_UART_Receive(&huart6, &temp, 1, 500) == HAL_OK) {  // 增加 timeout
	          if (temp == '\n' || temp == '\r') {  // 遇到換行符，表示數據接收完成
	              break;
	          }
	          buffer[index++] = temp;
	      }
	  }
	  buffer[index] = '\0';  // 確保字串結束

	  // **解析數據**
	  char *token = strtok((char *)buffer, ",");
	  if (token) fanData.fanState = atoi(token);

	  token = strtok(NULL, ",");
	  if (token) fanData.fanSpeed = atoi(token);

	  token = strtok(NULL, ",");
	  if (token) fanData.hAngle = atoi(token);

	  token = strtok(NULL, ",");
	  if (token) fanData.vAngle = atoi(token);

}

void SendDataToESP32(int fanState, int fanSpeed, int hAngle, int vAngle,float temperature) {
    char txBuffer[50];
    sprintf(txBuffer, "%d,%d,%d,%d,%f\n", fanState, fanSpeed, hAngle, vAngle,temperature);
    HAL_UART_Transmit(&huart6, (uint8_t*)txBuffer, strlen(txBuffer), 100);
}
// **取得解析後的風扇數據**
FanData ESP32_GetFanData(void) {
    return fanData;
}
