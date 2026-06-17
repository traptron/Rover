#ifndef PID_H
#define PID_H

// Структура для ПИД-регулятора
typedef struct {
    float Kp;
    float Ki;
    float Kd;
} PID_Controller_t;

extern PID_Controller_t pid_controllers[6];

#endif /* PID_H */
