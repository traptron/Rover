#ifndef PID_H
#define PID_H

#include <stdint.h>

// Главный ПИД-регулятор (Master) - по одному на каждый борт (Левый и Правый)
typedef struct {
    float kp, ki, kd;
    float kff;            // Feed-Forward коэффициент
    float min_pwm_offset; // Порог страгивания (мертвая зона)
    float integral_sum;
    float prev_error;
    float out_max;        // Максимальный ШИМ (например, 8999)
} Master_PID;

// Подчиненный П-регулятор (Slave) - для каждого отдельного колеса
typedef struct {
    float kp_sync;        // Коэффициент выравнивания скорости колеса
} Slave_Controller;

// Глобальные контроллеры
extern Master_PID left_board_pid;
extern Master_PID right_board_pid;
extern Slave_Controller wheel_slaves[6]; // 0,1,2 - Левые; 3,4,5 - Правые

// Инициализация Master PID
void Master_PID_Init(Master_PID *pid, float kp, float ki, float kd,
                     float kff, float min_pwm_offset, float out_max);

// Инициализация Slave контроллера
void Slave_Init(Slave_Controller *slave, float kp_sync);

// Вычисление базового ШИМ для борта (Master)
float calculate_master_pwm(Master_PID *pid, float target_vel, float current_avg_vel, float dt);

// Главная функция обновления скоростей (вызывать каждые 20 мс)
void update_rover_control(float target_left_vel, float target_right_vel, float dt, float *output_pwm);

#endif /* PID_H */
