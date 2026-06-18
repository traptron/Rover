#include "pid.h"

PID_Controller_t pid_controllers[6];

// Функция инициализации контроллера
void PID_Init(PID_Controller_t *pid, float kp, float ki, float kd, float out_min, float out_max) {
    pid->Kp = kp;
    pid->Ki = ki;
    pid->Kd = kd;
    pid->integral = 0.0f;
    pid->prev_error = 0.0f;
    pid->out_min = out_min;
    pid->out_max = out_max;
}

// Функция обновления состояния регулятора
float PID_Update(PID_Controller_t *pid, float setpoint, float current_value, float dt) {
    // Вычисляем текущую ошибку (разницу между уставкой и текущим значением)
    float error = setpoint - current_value;
    
    // Накапливаем интегральную сумму
    pid->integral += error * dt;
    
    // Anti-windup: ограничение интегральной суммы, чтобы избежать насыщения
    if (pid->Ki > 0.0f) {
        float max_i = pid->out_max / pid->Ki;
        float min_i = pid->out_min / pid->Ki;
        if (pid->integral > max_i) pid->integral = max_i;
        else if (pid->integral < min_i) pid->integral = min_i;
    }
    
    // Вычисляем производную ошибки
    float d_term = (error - pid->prev_error) / dt;
    pid->prev_error = error; // Запоминаем ошибку для следующего шага
    
    // Итоговое значение управляющего воздействия
    float output = (pid->Kp * error) + (pid->Ki * pid->integral) + (pid->Kd * d_term);
    
    // Ограничиваем итоговый выход (ШИМ) допустимыми рамками
    if (output > pid->out_max) output = pid->out_max;
    if (output < pid->out_min) output = pid->out_min;
    
    return output;
}
