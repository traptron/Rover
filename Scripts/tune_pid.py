import serial
import struct
import time
import threading

# Настройки COM-порта
SERIAL_PORT = '/dev/ttyS7'
BAUD_RATE = 115200

HEADER = 0xAA55

# Глобальные переменные для фоновой отправки
current_linear_x = 0.0
current_angular_z = 0.0
running = True

def calculate_crc8(data: bytes) -> int:
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

def pack_movement_packet(linear_x: float, angular_z: float, led_mask: int = 0) -> bytes:
    reserved = b'\x00' * 11
    packet_without_crc = struct.pack(
        '<HBffB11s', HEADER, 1, linear_x, angular_z, led_mask, reserved
    )
    crc = calculate_crc8(packet_without_crc)
    return packet_without_crc + struct.pack('<B', crc)

def pack_pid_tune_packet(board_id: int, kp: float, ki: float, kd: float, kff: float, min_pwm: int, kp_sync: float) -> bytes:
    kp_sync_x100 = int(kp_sync * 100)
    if kp_sync_x100 < 0: kp_sync_x100 = 0
    if kp_sync_x100 > 255: kp_sync_x100 = 255
    
    packet_without_crc = struct.pack(
        '<H B B f f f f H B',
        HEADER, 2, board_id, kp, ki, kd, kff, min_pwm, kp_sync_x100
    )
    crc = calculate_crc8(packet_without_crc)
    return packet_without_crc + struct.pack('<B', crc)

def uart_tx_thread(ser):
    """Фоновый поток для отправки пакетов Movement (чтобы не срабатывал failsafe)"""
    global current_linear_x, current_angular_z, running
    while running:
        if ser and ser.is_open:
            packet = pack_movement_packet(current_linear_x, current_angular_z)
            try:
                ser.write(packet)
            except:
                pass
        time.sleep(0.02) # 50 Гц

def uart_rx_thread(ser):
    global running
    if not ser: return
    buf = bytearray()
    packet_size = 27
    
    while running:
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
                            # Раскомментируйте, если хотите видеть сырые энкодеры
                            # print(f"\r[RX] Энкодеры: {encoders}          ", end="") 
                        buf = buf[packet_size:]
                    else:
                        buf.pop(0)
            else:
                time.sleep(0.01)
        except:
            break

def prompt_float(prompt_text, default):
    val = input(f"{prompt_text} [{default}]: ")
    return float(val) if val.strip() else default

def prompt_int(prompt_text, default):
    val = input(f"{prompt_text} [{default}]: ")
    return int(val) if val.strip() else default

def main():
    global current_linear_x, current_angular_z, running
    
    print(f"Открытие порта {SERIAL_PORT} на скорости {BAUD_RATE}...")
    try:
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
        print("Порт успешно открыт!")
    except Exception as e:
        print(f"Ошибка: {e}. Работа в режиме эмуляции.")
        ser = None

    tx_th = threading.Thread(target=uart_tx_thread, args=(ser,), daemon=True)
    tx_th.start()
    rx_th = threading.Thread(target=uart_rx_thread, args=(ser,), daemon=True)
    rx_th.start()

    # Сохраняем текущие значения, чтобы не вводить их каждый раз заново
    params = {
        'board_id': 0, 'kp': 1.0, 'ki': 0.0, 'kd': 0.0, 
        'kff': 0.0, 'min_pwm': 0, 'kp_sync': 0.5
    }

    print("\n--- Интерактивный тюнинг ПИД-регулятора ---")
    print("Команды:")
    print("  M - задать скорость (Movement)")
    print("  P - задать коэффициенты (PID Tune)")
    print("  Q - выход")

    try:
        while True:
            cmd = input("\n[M/P/Q]> ").strip().upper()
            
            if cmd == 'Q':
                break
            elif cmd == 'M':
                current_linear_x = prompt_float("linear_x", current_linear_x)
                current_angular_z = prompt_float("angular_z", current_angular_z)
                print(f"Скорость обновлена: x={current_linear_x}, z={current_angular_z}")
                
            elif cmd == 'P':
                params['board_id'] = prompt_int("Board ID (0=Left, 1=Right)", params['board_id'])
                params['kp'] = prompt_float("Kp", params['kp'])
                params['ki'] = prompt_float("Ki", params['ki'])
                params['kd'] = prompt_float("Kd", params['kd'])
                params['kff'] = prompt_float("Kff", params['kff'])
                params['min_pwm'] = prompt_int("Min PWM Offset", params['min_pwm'])
                params['kp_sync'] = prompt_float("Kp Sync", params['kp_sync'])
                
                packet = pack_pid_tune_packet(
                    params['board_id'], params['kp'], params['ki'], params['kd'],
                    params['kff'], params['min_pwm'], params['kp_sync']
                )
                
                if ser and ser.is_open:
                    ser.write(packet)
                    print(f"Параметры отправлены для борта {params['board_id']}!")
                else:
                    print("Эмуляция отправки ПИД-пакета...")
                    
    except KeyboardInterrupt:
        pass
    finally:
        running = False
        print("\nВыход...")
        if ser and ser.is_open:
            ser.close()

if __name__ == '__main__':
    main()
