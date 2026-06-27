#include "encoder.h"
#include "main.h"

// Константы для перевода тиков энкодера в м/с
#define GEAR_RATIO      46.0f      // Передаточное число редуктора 1:46
#define ENCODER_CPR     44.0f      // 11 PPR * 4 (счет по обоим фронтам)
#define WHEEL_DIAMETER  0.10f      // Колесо 100 мм (0.1 м)
#define PI_F            3.1415926f
#define METERS_PER_TICK (PI_F * WHEEL_DIAMETER / (GEAR_RATIO * ENCODER_CPR))
#define CONTROL_DT      0.02f      // 20 мс

int16_t current_speeds[6] = {0};
float current_wheel_speeds[6] = {0};

void read_encoders(void) {
    static uint32_t last_cnt[6] = {0}; // Предыдущие значения счетчиков
    uint32_t cnt[6];
    
    // Считываем значения из регистров
    cnt[0] = TIM1->CNT;
    cnt[1] = TIM2->CNT; // 32-битный таймер
    cnt[2] = TIM3->CNT;
    cnt[3] = TIM4->CNT;
    cnt[4] = TIM5->CNT; // 32-битный таймер
    cnt[5] = TIM8->CNT;
    
    for (int i = 0; i < 6; i++) {
        // Жесткое приведение к int16_t безопасно и корректно обрабатывает 
        // переполнение счетчиков (как 16-битных, так и 32-битных)
        current_speeds[i] = (int16_t)(cnt[i] - last_cnt[i]);
        last_cnt[i] = cnt[i]; // Сохраняем для следующего цикла
        
        // Пересчёт тиков за 20 мс в м/с для ПИД-регулятора
        current_wheel_speeds[i] = (current_speeds[i] / CONTROL_DT) * METERS_PER_TICK;
    }
}
