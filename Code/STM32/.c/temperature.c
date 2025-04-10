#include "temperature.h"

extern ADC_HandleTypeDef hadc1;
extern TIM_HandleTypeDef htim2;

static float temperature = 0.0;  // 全域變數儲存溫度
// 讀取 ADC 並轉換為攝氏溫度
float ADC1_temperature(void) {
    HAL_ADC_PollForConversion(&hadc1, 1000);
    uint32_t adc_value = HAL_ADC_GetValue(&hadc1); // 讀取 ADC 數值
    float voltage = (adc_value / 4095.0) * 3300; // 計算電壓
    return voltage*0.1;
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    if (htim->Instance == TIM2) {  // 確保是 TIM2
        temperature = ADC1_temperature();  // 更新溫度變數
    }
}
float Get_Temperature(void) {
    return temperature;
}
