#include "motor.h"
#include "main.h"

#define MAX_PWM 8999

void apply_motor_power(float *powers) {
    for (int i = 0; i < 6; ++i) {
        int32_t power_int = (int32_t)powers[i];
        int32_t pwm_val;
        
        // Управление направлением и расчет ШИМ (пины PF0-PF5)
        if (power_int >= 0) {
            GPIOF->BSRR = (1U << (16 + i));     // Сбрасываем пин PF[i] в 0 (DIR = 0)
            pwm_val = power_int;                // ШИМ как есть
        } else {
            GPIOF->BSRR = (1U << i);            // Устанавливаем пин PF[i] в 1 (DIR = 1)
            pwm_val = MAX_PWM - (-power_int);   // Инвертируем ШИМ
        }
        
        // Ограничение (Clamping)
        if (pwm_val > MAX_PWM) pwm_val = MAX_PWM;
        if (pwm_val < 0) pwm_val = 0;
        
        // Применение значения в регистр сравнения таймера (ШИМ)
        switch(i) {
            case 0: TIM9->CCR1  = pwm_val; break;
            case 1: TIM10->CCR1 = pwm_val; break;
            case 2: TIM11->CCR1 = pwm_val; break;
            case 3: TIM12->CCR2 = pwm_val; break; // Важно: для TIM12 сконфигурирован Канал 2!
            case 4: TIM13->CCR1 = pwm_val; break;
            case 5: TIM14->CCR1 = pwm_val; break;
        }
    }
}
