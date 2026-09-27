#!/bin/bash
set -e

BAG_NAME="$1"
RATE="${2:-5.0}"  # Скорость 5x по умолчанию

if [ "$BAG_NAME" == "--eval-only" ]; then
    TARGET_BAG="$2"
    python3 /ws/evaluate.py "/eval_results/$TARGET_BAG"
    exit 0
fi

if [ -z "$BAG_NAME" ]; then
    echo "Ошибка: укажите имя папки с bag-файлом."
    exit 1
fi

BAG_PATH="/bags/$BAG_NAME"
if [ ! -d "$BAG_PATH" ]; then
    echo "Ошибка: Папка $BAG_PATH не найдена!"
    exit 1
fi

# 1. PRE-FLIGHT CHECK
echo ">>> [0/4] Предварительная проверка аналитического скрипта..."
python3 /ws/evaluate.py --check

echo "=================================================="
echo " Прогон записи: $BAG_NAME"
ros2 bag info "$BAG_PATH" | grep -E "(Duration|Count):" || true
echo " Скорость воспроизведения: ${RATE}x"
echo "=================================================="

# Сохраняем в отдельный примонтированный том
OUT_DIR="/eval_results/$BAG_NAME"
mkdir -p /eval_results
rm -rf "$OUT_DIR"

echo ">>> [1/4] Запуск ноды tram_odometry..."
ros2 launch tram_odometry replay.launch.py use_clock:=true > /tmp/odometry.log 2>&1 &
NODE_PID=$!
sleep 2

if ! kill -0 $NODE_PID 2>/dev/null; then
    echo "[ОШИБКА] Нода одометрии не смогла запуститься. Лог:"
    cat /tmp/odometry.log
    exit 1
fi

echo ">>> [2/4] Запуск записи результатов (в $OUT_DIR)..."
ros2 bag record -o "$OUT_DIR" \
    /result/velocity \
    /result/position \
    /result/latency \
    /sensing/gnss/master/fix \
    /sensing/gnss/master/vel > /dev/null 2>&1 &
REC_PID=$!
sleep 1

echo ">>> [3/4] Воспроизведение записи (скорость ${RATE}x)..."
ros2 bag play "$BAG_PATH" --clock -r "$RATE"

echo ""
echo ">>> Воспроизведение завершено. Остановка процессов..."
kill -2 $REC_PID 2>/dev/null || true
kill -2 $NODE_PID 2>/dev/null || true
sleep 2

echo ">>> [4/4] Расчёт метрик..."
python3 /ws/evaluate.py "$OUT_DIR"
