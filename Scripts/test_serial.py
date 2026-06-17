import serial
import struct
import time
import threading

# Настройки COM-порта (настройте под ваш Orange Pi, например /dev/ttyUSB0 или /dev/ttyS0)
SERIAL_PORT = '/dev/ttyS7'
BAUD_RATE = 115200

# Заголовок пакета
HEADER = 0xAA55
MSG_ID_MOVEMENT = 1

def calculate_crc8(data: bytes) -> int:
    """
    Расчет CRC-8 с полиномом 0x07 (SMBus / CCITT-8)
    Начальное значение: 0x00
    """
    crc = 0x00
    for byte in data:
        crc ^= byte
        for _ in range(8):
            if crc & 0x80:
                crc = (crc << 1) ^ 0x07
            else:
                crc <<= 1
            crc &= 0xFF
    return crc

def pack_movement_packet(linear_x: float, angular_z: float, led_mask: int) -> bytes:
    """
    Упаковка пакета Movement
    Формат (Little-Endian):
    uint16_t header = 0xAA55
    uint8_t msg_id = 1
    float linear_x
    float angular_z
    uint8_t led_mask
    uint8_t reserved[11]
    uint8_t crc
    """
    # Пакуем все кроме CRC. 
    # Формат: < H B f f B 11s
    # H - uint16 (2 байта)
    # B - uint8 (1 байт)
    # f - float (4 байта)
    # 11s - 11 байт char[] (паддинг)
    reserved = b'\x00' * 11
    
    packet_without_crc = struct.pack(
        '<HBffB11s',
        HEADER,
        MSG_ID_MOVEMENT,
        linear_x,
        angular_z,
        led_mask,
        reserved
    )
    
    # Считаем CRC-8 от данных без байта CRC
    crc = calculate_crc8(packet_without_crc)
    
    # Добавляем CRC (1 байт, формат B)
    packet_with_crc = packet_without_crc + struct.pack('<B', crc)
    
    return packet_with_crc

def read_uart_thread(ser):
    if not ser:
        return
    
    buf = bytearray()
    packet_size = 27 # 2 (header) + 24 (6 * int32) + 1 (crc)
    
    while True:
        try:
            if ser.in_waiting > 0:
                buf.extend(ser.read(ser.in_waiting))
                
                while len(buf) >= packet_size:
                    if buf[0] == 0xBB and buf[1] == 0xBB:
                        packet = buf[:packet_size]
                        
                        crc_calc = calculate_crc8(packet[:-1])
                        crc_recv = packet[-1]
                        
                        if crc_calc == crc_recv:
                            unpacked = struct.unpack('<H6iB', packet)
                            encoders = unpacked[1:7]
                            print(f"\n[RX] Энкодеры: {encoders}")
                        else:
                            print(f"\n[RX] Ошибка CRC: calc={crc_calc}, recv={crc_recv}")
                        
                        buf = buf[packet_size:]
                    else:
                        buf.pop(0)
            else:
                time.sleep(0.01)
        except Exception as e:
            print(f"\n[RX] Ошибка чтения порта: {e}")
            break

def main():
    print(f"Открытие порта {SERIAL_PORT} на скорости {BAUD_RATE}...")
    try:
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
        print("Порт успешно открыт!")
    except Exception as e:
        print(f"Ошибка при открытии порта {SERIAL_PORT}: {e}")
        print("Скрипт будет запущен в режиме эмуляции (без реальной отправки в порт).")
        ser = None

    print("\n--- Тестовый терминал управления Rover ---")
    print("Введите значения для отправки.")
    print("Для выхода нажмите Ctrl+C")
    
    if ser and ser.is_open:
        rx_thread = threading.Thread(target=read_uart_thread, args=(ser,), daemon=True)
        rx_thread.start()
    
    while True:
        try:
            print("-" * 40)
            linear_x_str = input("linear_x (float) [0.0]: ")
            linear_x = float(linear_x_str) if linear_x_str.strip() else 0.0
            
            angular_z_str = input("angular_z (float) [0.0]: ")
            angular_z = float(angular_z_str) if angular_z_str.strip() else 0.0
            
            led_mask_str = input("led_mask (hex или int) [0]: ")
            if not led_mask_str.strip():
                led_mask = 0
            elif led_mask_str.startswith('0x'):
                led_mask = int(led_mask_str, 16)
            else:
                led_mask = int(led_mask_str)
                
            # Ограничиваем led_mask до 1 байта
            led_mask &= 0xFF
            
            packet = pack_movement_packet(linear_x, angular_z, led_mask)
            
            print(f"Сформирован пакет ({len(packet)} байт): {packet.hex().upper()}")
            
            if ser and ser.is_open:
                ser.write(packet)
                print("Пакет отправлен!")
            else:
                print("Эмуляция отправки...")
                
        except ValueError:
            print("Ошибка ввода! Пожалуйста, вводите числа (для float используйте точку).")
        except KeyboardInterrupt:
            print("\nВыход из программы...")
            break
        except Exception as e:
            print(f"Произошла ошибка: {e}")

    if ser and ser.is_open:
        ser.close()

if __name__ == '__main__':
    main()
