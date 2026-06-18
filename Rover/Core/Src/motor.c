#include "motor.h"
#include "main.h"

void apply_motor_power(float *powers) {
    for (int i = 0; i < 6; ++i) {
        int32_t power_int = (int32_t)powers[i];
        
        // Управление направлением (пины PF0-PF5)
        if (power_int >= 0) {
            GPIOF->BSRR = (1U << i);            // Устанавливаем пин PF[i] в 1 (движение вперед)
        } else {
            GPIOF->BSRR = (1U << (16 + i));     // Сбрасываем пин PF[i] в 0 (движение назад)
            power_int = -power_int;             // Берем модуль мощности для ШИМ
        }
        
        // Ограничение максимальной мощности
        if (power_int > 1000) power_int = 1000;
        
        // Применение значения в регистр сравнения таймера (ШИМ)
        switch(i) {
            case 0: TIM9->CCR1  = power_int; break;
            case 1: TIM10->CCR1 = power_int; break;
            case 2: TIM11->CCR1 = power_int; break;
            case 3: TIM12->CCR2 = power_int; break; // Важно: для TIM12 сконфигурирован Канал 2!
            case 4: TIM13->CCR1 = power_int; break;
            case 5: TIM14->CCR1 = power_int; break;
        }
    }
}
