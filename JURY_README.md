# Резервная одометрия по модели — инструкция для жюри

Хакатон «Московский транспорт», кейс «Резервная одометрия по модели».

Решение вычисляет продольную скорость и положение трамвая **без GNSS и без IMU
в основном контуре**, используя только:

- положение ручки контроллера водителя (`/vehicle/driver_position_cmd`);
- скорость передних колёс (`/vehicle/front_bogie_velocity`);
- скорость задних колёс (`/vehicle/rear_bogie_velocity`).

GNSS-топики (`/sensing/gnss/*`) используются **только для начальной выставки** в
первые ~2.5 с записи и для однократной калибровки масштаба колёс — как разрешено
условиями задачи.

---

## 1. Состав решения

| Пакет | Тип | Назначение |
|---|---|---|
| `tram_vehicle_msgs` | ament_cmake | Кастомные сообщения `VelocitySensor`, `DriverControllerCommand`. |
| `tram_odometry` | ament_cmake (C++17) | Нода оценки скорости и положения. |

Сборка — стандартный `colcon build` без внешнего интернета. Все зависимости —
только стандартные пакеты ROS 2 Humble и `tram_vehicle_msgs`.

---

## 2. Сборка

```bash
# 1. Скопировать пакеты в src рабочей области ROS 2 Humble
#    (tram_vehicle_msgs/ и tram_odometry/ лежат в cpp/)
cp -r cpp/tram_vehicle_msgs cpp/tram_odometry <ws>/src/

# 2. Собрать
cd <ws>
colcon build --packages-select tram_vehicle_msgs tram_odometry
source install/setup.bash
```

`tram_vehicle_msgs` собирается **первым** (он — зависимость `tram_odometry`).

---

## 3. Запуск на rosbag

Нода не запускает `ros2 bag play` сама — жюри запускает оба процесса:

```bash
# Терминал 1 — нода (use_clock=true => bag играется с --clock)
source install/setup.bash
ros2 launch tram_odometry replay.launch.py use_clock:=true

# Терминал 2 — воспроизведение bag
source install/setup.bash
ros2 bag play <bag> --clock
```

Если bag играется **без** `--clock`, запускайте ноду с `use_clock:=false`.

Конфигурация — `cpp/tram_odometry/config/params.yaml` (передаётся через launch);
все параметры можно переопределить через `-p`.

---

## 4. Входные топики (подписки)

| Топик | Тип |
|---|---|
| `/vehicle/front_bogie_velocity` | `tram_vehicle_msgs/msg/VelocitySensor` |
| `/vehicle/rear_bogie_velocity` | `tram_vehicle_msgs/msg/VelocitySensor` |
| `/vehicle/driver_position_cmd` | `tram_vehicle_msgs/msg/DriverControllerCommand` |
| `/sensing/gnss/master/fix` | `sensor_msgs/msg/NavSatFix` (только инициализация) |
| `/sensing/gnss/master/vel` | `geometry_msgs/msg/TwistStamped` (только инициализация) |
| `/sensing/gnss/rover/fix` | `sensor_msgs/msg/NavSatFix` (только инициализация) |
| `/sensing/gnss/rover/vel` | `geometry_msgs/msg/TwistStamped` (только инициализация) |

Единицы: `VelocitySensor.velocity` в записях bag — **км/ч** (подтверждено эмпирически
сверкой с GNSS и физической правдоподобностью). Нода конвертирует в м/с на входе.

---

## 5. Выходные топики (публикации)

| Топик | Тип | Поле | Примечание |
|---|---|---|---|
| `/result/velocity` | `tram_vehicle_msgs/msg/VelocitySensor` | `velocity` (м/с) | продольная скорость |
| `/result/position` | `nav_msgs/msg/Odometry` | `pose.pose.position.x/y/z` | положение в локальной метрической СК |
| `/result/diagnostics` | `diagnostic_msgs/msg/DiagnosticArray` | — | диагностика/статус проскальзывания |
| `/result/latency` | `std_msgs/msg/Float64` | `data` (мс) | задержка «вход → результат» |

- Частота `/result/velocity` и `/result/position` — **50 Гц** (≥ 10 Гц по ТЗ).
- `header.stamp` берётся **из времени bag** (время входного сообщения), а не из
  стеновых часов ноды — для корректной синхронизации судьёй (допуск ~0.05 с).
- `header.frame_id`: `map` (геопривязанная СК) или `odom_local` (относительная
  одометрия, когда GNSS отсутствует); `child_frame_id = base_link`.
- Координаты `pose.pose.position` — в метрической СК, согласованной с эталоном
  (UTM/MGRS минус начало записи).

QoS издателя — reliable по умолчанию, совместим с best-effort подписчиком судьи.

---

## 6. Как посмотреть логи, метрики и задержку

```bash
# метрики задержки (мс) в реальном времени
ros2 topic echo /result/latency

# скорость и положение
ros2 topic echo /result/velocity
ros2 topic echo /result/position

# диагностика: trust (доверие к одометрии), slip_index, gnss_used и др.
ros2 topic echo /result/diagnostics
```

Ключевые поля диагностики (`/result/diagnostics`):
- `velocity_mps`, `accel_mps2` — оценённая скорость/ускорение;
- `odometry_trust` — доверие к колёсной одометрии (0..1);
- `slip_index`, `slip_reason` — статус проскальзывания;
- `latency_ms`, `cycle_ms` — задержка и время цикла;
- `gnss_used`, `gnss_published` — использовался ли GNSS в текущем цикле.

---

## 7. Проверка точности

Судья сравнивает `/result/velocity` и `/result/position` с эталоном
(`/localization/kinematic_state` или GNSS-телеметрией). Оценка — RMSE/MAE скорости
(м/с) и отклонение положения (along-track/cross-track, дрейф %).

Методика внутренней сверки с GNSS-эталоном и таблицы ошибок — в `cpp/README.md`
(раздел «What IS verified»).
