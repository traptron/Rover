#include "encoder.h"
#include "main.h"

int16_t current_speeds[6] = {0};

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
    }
}
