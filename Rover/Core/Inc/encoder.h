#ifndef ENCODER_H
#define ENCODER_H

#include <stdint.h>

// Массив с текущими скоростями в тиках за цикл
extern int16_t current_speeds[6];

// Массив с текущими скоростями в м/с (для ПИД-регулятора)
extern float current_wheel_speeds[6];

// Опрос счетчиков аппаратных таймеров (энкодеров)
void read_encoders(void);

#endif /* ENCODER_H */
