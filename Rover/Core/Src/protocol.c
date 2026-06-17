#include "protocol.h"

// Определение глобальных переменных
FixedPacket rx_packet;


/**
 * @brief Обработчик и парсер полученного бинарного пакета.
 * @param packet Указатель на структуру пакета FixedPacket
 */
void parse_packet(FixedPacket *packet) {
    switch (packet->msg_id) {
        case 1: { // Сообщение движения
            float target_lin = packet->payload.movement.linear_x;
            float target_ang = packet->payload.movement.angular_z;
            uint8_t flags = packet->payload.movement.cmd_flags;
            
            // Рассчитываем уставки для моторов...
            // (Временное подавление варнингов о неиспользованных переменных)
            (void)target_lin;
            (void)target_ang;
            (void)flags;
            break;
        }
            
        case 2: { // Настройка ПИД
            uint8_t motor = packet->payload.pid_tune.motor_id;
            if (motor < 6) {
                pid_controllers[motor].Kp = packet->payload.pid_tune.kp;
                pid_controllers[motor].Ki = packet->payload.pid_tune.ki;
                pid_controllers[motor].Kd = packet->payload.pid_tune.kd;
            }
            break;
        }
            
        default:
            // Неизвестный ID сообщения
            break;
    }
}
