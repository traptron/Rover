#include <stdint.h>
#include "pid.h"

#pragma pack(push, 1)

// Подструктура 1: Скорость и управление
typedef struct {
    float linear_x;      // 4 байта
    float angular_z;     // 4 байта
    uint8_t led_mask;    // 1 байт
    uint8_t reserved[11]; // 11 байт паддинга для выравнивания размера
} MsgMovement;

// Подструктура 2: Настройка ПИД (например, для одного из моторов)
typedef struct {
    uint8_t motor_id;    // 1 байт (какой мотор настраиваем: 0-5)
    float kp;            // 4 байта
    float ki;            // 4 байта
    float kd;            // 4 байта
    uint8_t reserved[7]; // 7 байт паддинга для выравнивания размера
} MsgPidTune;

// Главный фиксированный пакет (Всего: 2 + 1 + 20 + 1 = 24 байта)
typedef struct {
    uint16_t header;     // 2 байта: Маркер начала (например, 0xAA55)
    uint8_t msg_id;      // 1 байт: Тип сообщения (1 - Движение, 2 - ПИД)
    
    union {
        MsgMovement movement;
        MsgPidTune pid_tune;
        uint8_t raw_payload[20]; // Фиксированный размер полезной нагрузки
    } payload;
    
    uint8_t crc;         // 1 байт: Контрольная сумма
} FixedPacket;

#pragma pack(pop)

extern FixedPacket rx_packet;


// Прототип функции парсинга пакета
void parse_packet(FixedPacket *packet);

// Функция расчета CRC-8 (полином 0x07, SMBus)
uint8_t calculate_crc8(const uint8_t *data, uint16_t size);

// Функция-парсер входящих байт (вызывается из прерывания или периодически)
void parse_incoming_byte(uint8_t byte);