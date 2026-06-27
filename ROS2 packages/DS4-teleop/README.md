# Телеоперация 6-колёсного ровера через DualShock 4

Узел ROS2 Humble для управления 6-колёсным ровером с помощью джойстика DualShock 4, подключённого по Bluetooth к Orange Pi 3B.

## Архитектура

```
DualShock 4 (Bluetooth)
    ↓
/dev/input/event5
    ↓
joy_node → /joy (sensor_msgs/Joy)
    ↓
teleop_node → /cmd_vel (geometry_msgs/Twist)
    ↓
STM32 Motor Controller (UART)
```

## Управление

| Действие | Кнопка | Эффект |
|----------|--------|--------|
| Движение вперёд | R2 (триггер) | linear.x → [0, +max] |
| Движение назад | L2 (триггер) | linear.x → [-max, 0] |
| Поворот | Left Stick X | angular.z → [-max, +max] |

**Нормализация триггеров:** Триггеры из диапазона `[1.0, -1.0]` переводятся в `[0, 1]` по формуле:
```
value = (1.0 - axis) / 2.0
```

## Быстрый старт

### 1. Сборка пакета

```bash
cd ~/rover_ws
colcon build --packages-select teleop_DS4
source install/setup.bash
```

### 2. Запуск системы

```bash
ros2 launch teleop_DS4 teleop.launch.py joy_device:=/dev/input/event5
```

### 3. Отладка (с выводом логов)

```bash
ros2 launch teleop_DS4 teleop.launch.py \
  joy_device:=/dev/input/event5 \
  debug:=true
```

## Мониторинг

### Просмотр команд скорости

```bash
ros2 topic echo /cmd_vel
```

### Просмотр входных данных джойстика

```bash
ros2 topic echo /joy
```

### Список параметров

```bash
ros2 param list /teleop_ds4_node
```

## Параметры

| Параметр | Тип | Значение по умолчанию | Описание |
|----------|-----|----------------------|---------|
| `max_linear_speed` | float | 1.0 | Максимальная линейная скорость (м/с) |
| `max_angular_speed` | float | 1.0 | Максимальная угловая скорость (рад/с) |
| `deadzone` | float | 0.05 | Мёртвая зона для аналоговых входов [0, 1] |
| `accel_limit` | float | 0.5 | Ограничение ускорения (ед/с²) |
| `watchdog_timeout` | float | 0.2 | Таймаут отказа безопасности (сек) |
| `debug` | bool | false | Включить отладочные логи |

### Изменение параметров во время работы

```bash
# Установить максимальную скорость
ros2 param set /teleop_ds4_node max_linear_speed 2.0

# Включить отладку
ros2 param set /teleop_ds4_node debug true
```

## Безопасность

1. **Watchdog Timer** — если сообщение от джойстика не получено в течение 200 мс, публикуется нулевая скорость
2. **Deadzone** — входы ниже порога 0.05 игнорируются (исключает дрейф)
3. **Rate Limiting** — ограничение ускорения предотвращает резкие прыжки скорости
4. **Clamping** — все выходы ограничены диапазоном `[-max, +max]`

## Техническое описание

### Вычисление скорости

```python
# Нормализация триггеров
L2_norm = (1.0 - L2_raw) / 2.0   # [1, -1] → [0, 1]
R2_norm = (1.0 - R2_raw) / 2.0   # [1, -1] → [0, 1]

# Вычисление желаемых скоростей
linear = (R2_norm - L2_norm) * max_linear_speed
angular = left_stick_x * max_angular_speed

# Применение мёртвой зоны, ограничение и сглаживание
# ...

# Публикация
twist.linear.x = linear
twist.angular.z = angular
```

### Цикл управления

- **Основной цикл** — 50 Гц (публикация `/cmd_vel`)
- **Watchdog** — 100 Гц (проверка timeout)
- **Отладочные логи** — 5 Гц (throttled)

## Устранение неполадок

### Джойстик не распознаётся

```bash
# Проверить, что устройство доступно
ls -la /dev/input/event*

# Проверить права доступа
sudo usermod -a -G input $USER
```

### Нет команд на `/cmd_vel`

```bash
# Проверить, что joy_node работает
ros2 topic echo /joy

# Если нет данных, проверить джойстик
ros2 node list | grep joy
```

### Предупреждения Watchdog

```
[WARN] Watchdog timeout: no joy message for 0.200s
```
Джойстик потерял связь. Проверьте Bluetooth соединение.

## Индексы осей DualShock 4 (стандартная нумерация)

| Ось | Индекс | Диапазон | Описание |
|-----|--------|----------|---------|
| LX | 0 | [-1, 1] | Left stick X |
| LY | 1 | [-1, 1] | Left stick Y |
| RX | 2 | [-1, 1] | Right stick X |
| RY | 3 | [-1, 1] | Right stick Y |
| L2 | 4 | [1, -1] | L2 триггер |
| R2 | 5 | [1, -1] | R2 триггер |

## Зависимости

- ROS2 Humble
- rclpy
- sensor_msgs
- geometry_msgs
- joy (пакет для чтения джойстика)

## Сборка и установка

```bash
# Вспомогательные зависимости
rosdep install --from-paths src --ignore-src -r -y

# Сборка
colcon build --packages-select teleop_DS4

# Источник
source install/setup.bash
```

## Тестирование

```bash
# Запустить с тестовыми параметрами
ros2 launch teleop_DS4 teleop.launch.py \
  max_linear_speed:=0.5 \
  max_angular_speed:=0.5 \
  debug:=true
```

## Лицензия

Apache-2.0

## Автор

traptron
