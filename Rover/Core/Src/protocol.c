#include "protocol.h"
#include "main.h"
#include <string.h>

// Определение глобальных переменных
FixedPacket rx_packet;
float target_left_speed = 0.0f;
float target_right_speed = 0.0f;
uint32_t last_packet_time = 0;

void calculate_kinematics(float linear_x, float angular_z) {
    // Коэффициент усиления поворота для skid-steer.
    // При angular_z = 1.0 мотор получит скорость +/- 1.0
    float angular_scale = 1.0f;
    
    target_left_speed  = linear_x - (angular_z * angular_scale);
    target_right_speed = linear_x + (angular_z * angular_scale);
}


/**
 * @brief Обработчик и парсер полученного бинарного пакета.
 * @param packet Указатель на структуру пакета FixedPacket
 */
void parse_packet(FixedPacket *packet) {
    switch (packet->msg_id) {
        case 1: { // Сообщение движения
            float target_lin = packet->payload.movement.linear_x;
            float target_ang = packet->payload.movement.angular_z;
            uint8_t led_mask = packet->payload.movement.led_mask;
            
            // Если linear_x > 0 — зажигаем зеленый LED (LD1).
            // Если linear_x < 0 — красный (LD3).
            // Если linear_x == 0 — гасим оба.
            if (target_lin > 0.0f) {
                HAL_GPIO_WritePin(LD1_GPIO_Port, LD1_Pin, GPIO_PIN_SET);   // Зеленый ВКЛ
                HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_RESET); // Красный ВЫКЛ
            } else if (target_lin < 0.0f) {
                HAL_GPIO_WritePin(LD1_GPIO_Port, LD1_Pin, GPIO_PIN_RESET); // Зеленый ВЫКЛ
                HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_SET);   // Красный ВКЛ
            } else {
                HAL_GPIO_WritePin(LD1_GPIO_Port, LD1_Pin, GPIO_PIN_RESET); // Зеленый ВЫКЛ
                HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_RESET); // Красный ВЫКЛ
            }
            
            // Синий LED (LD2) зажигаем/гасим в зависимости от нулевого бита поля led_mask.
            if (led_mask & 0x01) {
                HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);   // Синий ВКЛ
            } else {
                HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET); // Синий ВЫКЛ
            }
            
            // Рассчитываем уставки для моторов
            calculate_kinematics(target_lin, target_ang);
            break;
        }
            
        case 2: { // Настройка ПИД (motor_id: 0 = левый борт, 1 = правый борт)
            uint8_t motor = packet->payload.pid_tune.motor_id;
            Master_PID *pid = NULL;
            if (motor == 0) pid = &left_board_pid;
            else if (motor == 1) pid = &right_board_pid;
            
            if (pid != NULL) {
                pid->kp = packet->payload.pid_tune.kp;
                pid->ki = packet->payload.pid_tune.ki;
                pid->kd = packet->payload.pid_tune.kd;
            }
            break;
        }
            
        default:
            // Неизвестный ID сообщения
            break;
    }
}

/**
 * @brief Расчет CRC-8 для буфера данных (полином 0x07, стандартный SMBus/CRC-8).
 * @param data Указатель на массив байт
 * @param size Размер массива
 * @return Вычисленное значение CRC-8
 */
uint8_t calculate_crc8(const uint8_t *data, uint16_t size) {
    uint8_t crc = 0;
    for (uint16_t i = 0; i < size; i++) {
        crc ^= data[i];
        for (uint8_t j = 0; j < 8; j++) {
            if (crc & 0x80) {
                crc = (crc << 1) ^ 0x07;
            } else {
                crc <<= 1;
            }
        }
    }
    return crc;
}

/**
 * @brief Побайтовый парсер входящего потока данных.
 *        Находит маркер начала 0xAA55 (как в little-endian, так и в big-endian виде),
 *        накапливает пакет, сверяет CRC и вызывает обработчик parse_packet.
 * @param byte Очередной полученный байт из UART (из прерывания или буфера)
 */
void parse_incoming_byte(uint8_t byte) {
    static enum {
        STATE_SEARCH,
        STATE_ACCUMULATE
    } state = STATE_SEARCH;
    
    static uint8_t buf[sizeof(FixedPacket)];
    static uint8_t idx = 0;
    
    if (state == STATE_SEARCH) {
        buf[idx] = byte;
        if (idx == 0) {
            // Заголовок 0xAA55 может быть принят в виде [0x55, 0xAA] (little-endian) или [0xAA, 0x55] (big-endian/network order)
            if (byte == 0x55 || byte == 0xAA) {
                idx = 1;
            }
        } else if (idx == 1) {
            if ((buf[0] == 0x55 && byte == 0xAA) || (buf[0] == 0xAA && byte == 0x55)) {
                buf[1] = byte;
                idx = 2;
                state = STATE_ACCUMULATE;
            } else if (byte == 0x55 || byte == 0xAA) {
                // Если пришел повторный первый байт, остаемся на поиске второго
                buf[0] = byte;
                idx = 1;
            } else {
                idx = 0;
            }
        }
    } else if (state == STATE_ACCUMULATE) {
        buf[idx++] = byte;
        if (idx >= sizeof(FixedPacket)) {
            // Вычисляем CRC-8 для всего пакета, за исключением последнего байта (поля crc)
            uint8_t calc_crc = calculate_crc8(buf, sizeof(FixedPacket) - 1);
            if (calc_crc == buf[sizeof(FixedPacket) - 1]) {
                // Пакет корректен, копируем его в глобальный приемный буфер
                memcpy(&rx_packet, buf, sizeof(FixedPacket));
                last_packet_time = HAL_GetTick(); // Обновляем время последнего успешного пакета
                parse_packet(&rx_packet);
            }
            idx = 0;
            state = STATE_SEARCH;
        }
    }
}
