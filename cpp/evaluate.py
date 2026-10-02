#!/usr/bin/env python3
import sys
import math
import numpy as np

# Предварительная проверка окружения
try:
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
except ImportError as e:
    print(f"[FATAL] Ошибка импорта системных библиотек ROS 2: {e}")
    sys.exit(1)

if len(sys.argv) > 1 and sys.argv[1] == "--check":
    print(">>> [Pre-flight] Проверка импортов и среды evaluate.py: УСПЕШНО")
    sys.exit(0)

def latlon_to_xy(lat, lon, lat0, lon0):
    r_earth = 6378137.0
    dlat = math.radians(lat - lat0)
    dlon = math.radians(lon - lon0)
    return dlon * r_earth * math.cos(math.radians(lat0)), dlat * r_earth

def evaluate(bag_path):
    print(f"\n>>> Чтение и расчет метрик для: {bag_path}")

    reader = rosbag2_py.SequentialReader()
    storage_options = rosbag2_py.StorageOptions(uri=str(bag_path), storage_id='')
    converter_options = rosbag2_py.ConverterOptions(
        input_serialization_format='cdr',
        output_serialization_format='cdr'
    )

    try:
        reader.open(storage_options, converter_options)
    except Exception as e:
        print(f"[ОШИБКА] Не удалось открыть bag: {e}")
        return

    topics_and_types = reader.get_all_topics_and_types()
    type_map = {t.name: get_message(t.type) for t in topics_and_types}

    gt_vel, est_vel = [], []
    gt_pos, est_pos = [], []
    latencies = []
    slip_events = 0

    while reader.has_next():
        topic, data, timestamp = reader.read_next()
        msg_cls = type_map.get(topic)
        if not msg_cls:
            continue

        msg = deserialize_message(data, msg_cls)

        # Время в секундах
        if hasattr(msg, 'header') and msg.header.stamp.sec > 0:
            t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        else:
            t = timestamp * 1e-9

        if topic == '/sensing/gnss/master/vel':
            gt_vel.append((t, math.hypot(msg.twist.linear.x, msg.twist.linear.y)))
        elif topic == '/result/velocity':
            est_vel.append((t, msg.velocity))
        elif topic == '/sensing/gnss/master/fix':
            if msg.status.status >= 0 and not math.isnan(msg.latitude):
                gt_pos.append((t, msg.latitude, msg.longitude))
        elif topic == '/result/position':
            est_pos.append((t, msg.pose.pose.position.x, msg.pose.pose.position.y))
        elif topic == '/result/latency':
            latencies.append(msg.data)
        elif topic == '/result/diagnostics':
            for status in getattr(msg, 'status', []):
                for kv in getattr(status, 'values', []):
                    if kv.key == 'slip_reason' and kv.value != 'none':
                        slip_events += 1

    print("\n" + "="*60)
    print("         ОТЧЕТ РАБОТЫ АВТОНОМНОЙ ОДОМЕТРИИ")
    print("="*60)

    if not est_vel:
        print("[ОШИБКА] Топик /result/velocity пуст. Нода не публиковала данные!")
        return

    duration = est_vel[-1][0] - est_vel[0][0]
    velocities = [v for _, v in est_vel]
    # Численное интегрирование пройденного пути
    dist_integrated = sum(velocities[i] * (est_vel[i][0] - est_vel[i-1][0]) for i in range(1, len(est_vel)))

    print(f"Режим:                    {'С ЭТАЛОНОМ GNSS' if gt_vel else 'АВТОНОМНЫЙ (БЕЗ GNSS)'}")
    print(f"Длительность записи:      {duration:.1f} сек ({duration/60:.1f} мин)")
    print(f"Опубликовано точек:       {len(est_vel)} (частота: {len(est_vel)/max(duration, 0.001):.1f} Гц)")
    print(f"Средняя скорость:         {np.mean(velocities)*3.6:.2f} км/ч ({np.mean(velocities):.2f} м/с)")
    print(f"Максимальная скорость:     {np.max(velocities)*3.6:.2f} км/ч ({np.max(velocities):.2f} м/с)")
    print(f"Оцененный путь:           {dist_integrated:.1f} м")

    if est_pos:
        print(f"Конечная координата:      X={est_pos[-1][1]:.2f} м, Y={est_pos[-1][2]:.2f} м")

    if slip_events > 0:
        print(f"Зафиксировано проскальзываний: {slip_events}")

    # Сверка с GNSS (если он есть в файле)
    if gt_vel:
        vel_errors = []
        gt_idx = 0
        for t_est, v_est in est_vel:
            while gt_idx < len(gt_vel) - 1 and gt_vel[gt_idx][0] < t_est - 0.05:
                gt_idx += 1
            if abs(t_est - gt_vel[gt_idx][0]) <= 0.05:
                vel_errors.append(abs(v_est - gt_vel[gt_idx][1]))

        if vel_errors:
            vel_errors = np.array(vel_errors)
            print("-" * 60)
            print("ТОЧНОСТЬ (СВЕРКА С GNSS ЭТАЛОНОМ):")
            print(f"  Точек синхронизации:    {len(vel_errors)}")
            print(f"  RMSE скорости:          {np.sqrt(np.mean(vel_errors**2)):.3f} м/с")
            print(f"  MAE скорости:           {np.mean(vel_errors):.3f} м/с")
            print(f"  Макс. ошибка:           {np.max(vel_errors):.3f} м/с")

    if latencies:
        print("-" * 60)
        print(f"Средняя задержка:         {np.mean(latencies):.2f} мс (норма <= 100 мс)")
        print(f"99-й перцентиль задержки: {np.percentile(latencies, 99):.2f} мс")
    print("="*60 + "\n")

if __name__ == '__main__':
    if len(sys.argv) < 2:
        print("Использование: python3 evaluate.py <путь_к_багу>")
        sys.exit(1)
    evaluate(sys.argv[1])
