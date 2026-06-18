#ifndef ENCODER_H
#define ENCODER_H

#include <stdint.h>

// Массив с текущими скоростями в тиках за цикл
extern int16_t current_speeds[6];

// Опрос счетчиков аппаратных таймеров (энкодеров)
void read_encoders(void);

#endif /* ENCODER_H */
