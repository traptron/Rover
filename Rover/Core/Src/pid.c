#include "pid.h"
#include "encoder.h"

// Определение глобальных контроллеров
Master_PID left_board_pid;
Master_PID right_board_pid;
Slave_Controller wheel_slaves[6];

void Master_PID_Init(Master_PID *pid, float kp, float ki, float kd,
                     float kff, float min_pwm_offset, float out_max) {
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->kff = kff;
    pid->min_pwm_offset = min_pwm_offset;
    pid->integral_sum = 0.0f;
    pid->prev_error = 0.0f;
    pid->out_max = out_max;
}

void Slave_Init(Slave_Controller *slave, float kp_sync) {
    slave->kp_sync = kp_sync;
}

/**
 * Вычисление базового ШИМ для всего борта (Master)
 */
float calculate_master_pwm(Master_PID *pid, float target_vel, float current_avg_vel, float dt) {
    float error = target_vel - current_avg_vel;
    
    // ПИД
    float P = pid->kp * error;
    pid->integral_sum += error * dt;
    // Anti-windup (ограничение интегратора)
    if (pid->integral_sum > (pid->out_max / 2.0f)) pid->integral_sum = (pid->out_max / 2.0f);
    if (pid->integral_sum < -(pid->out_max / 2.0f)) pid->integral_sum = -(pid->out_max / 2.0f);
    float I = pid->ki * pid->integral_sum;
    float D = pid->kd * ((error - pid->prev_error) / dt);
    pid->prev_error = error;
    
    // Feed-Forward
    float ff_output = 0.0f;
    if (target_vel > 0.001f) {
        ff_output = (pid->kff * target_vel) + pid->min_pwm_offset;
    } else if (target_vel < -0.001f) {
        ff_output = (pid->kff * target_vel) - pid->min_pwm_offset;
    }
    
    return P + I + D + ff_output;
}

/**
 * Главная функция обновления скоростей (Вызывать каждые 20 мс)
 * @param target_left_vel  Целевая скорость левого борта (из кинематики ROS2)
 * @param target_right_vel Целевая скорость правого борта
 * @param dt               Шаг времени (0.02 сек)
 * @param output_pwm       Массив куда запишется итоговый ШИМ для 6 моторов
 */
void update_rover_control(float target_left_vel, float target_right_vel, float dt, float *output_pwm) {
    
    // 1. Считаем среднюю скорость каждого борта (по реальным энкодерам)
    float avg_left_vel = (current_wheel_speeds[0] + current_wheel_speeds[1] + current_wheel_speeds[2]) / 3.0f;
    float avg_right_vel = (current_wheel_speeds[3] + current_wheel_speeds[4] + current_wheel_speeds[5]) / 3.0f;
    
    // 2. Master: Вычисляем базовый ШИМ для каждого борта
    float base_pwm_left = calculate_master_pwm(&left_board_pid, target_left_vel, avg_left_vel, dt);
    float base_pwm_right = calculate_master_pwm(&right_board_pid, target_right_vel, avg_right_vel, dt);
    
    // 3. Slave: Распределяем ШИМ по колесам с учетом их индивидуального отклонения от среднего
    // Левый борт (моторы 0, 1, 2)
    for (int i = 0; i < 3; i++) {
        // Если колесо крутится медленнее среднего - добавляем ему мощности, если быстрее - убавляем
        float sync_error = avg_left_vel - current_wheel_speeds[i]; 
        float sync_correction = wheel_slaves[i].kp_sync * sync_error;
        
        output_pwm[i] = base_pwm_left + sync_correction;
    }
    
    // Правый борт (моторы 3, 4, 5)
    for (int i = 3; i < 6; i++) {
        float sync_error = avg_right_vel - current_wheel_speeds[i]; 
        float sync_correction = wheel_slaves[i].kp_sync * sync_error;
        
        output_pwm[i] = base_pwm_right + sync_correction;
    }
    
    // 4. Ограничение финального ШИМ
    for (int i = 0; i < 6; i++) {
        if (output_pwm[i] > left_board_pid.out_max) output_pwm[i] = left_board_pid.out_max;
        if (output_pwm[i] < -left_board_pid.out_max) output_pwm[i] = -left_board_pid.out_max;
    }
}
