#ifndef PID_H
#define PID_H

// Структура для ПИД-регулятора
typedef struct {
    float Kp;
    float Ki;
    float Kd;
    
    float integral;     // Накопленная интегральная сумма
    float prev_error;   // Ошибка на предыдущем шаге
    
    float out_min;      // Минимальное выходное значение
    float out_max;      // Максимальное выходное значение
} PID_Controller_t;

// Массив контроллеров для 6 моторов
extern PID_Controller_t pid_controllers[6];

// Инициализация ПИД-регулятора
void PID_Init(PID_Controller_t *pid, float kp, float ki, float kd, float out_min, float out_max);

// Вычисление управляющего воздействия (выхода)
float PID_Update(PID_Controller_t *pid, float setpoint, float current_value, float dt);

#endif /* PID_H */
