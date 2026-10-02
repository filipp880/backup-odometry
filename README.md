# Резервная одометрия трамвая по модели

Хакатон «Московский транспорт», кейс «Резервная одометрия по модели».

Нода оценивает **продольную скорость и положение трамвая без GNSS и без IMU в
основном контуре**. Используются только три топика: положение ручки контроллера
водителя и скорости передней/задней тележек. GNSS нужен лишь для начальной
выставки начала координат в первые ~2.5 с и для однократной калибровки масштаба
колёс — как разрешено условиями.

Оценщик — 6-состоянийный error-state фильтр Калмана поверх модели продольной
динамики, с онлайн-обнаружением проскальзывания и адаптацией модели. Подробный
разбор математики — в [MODEL.md](MODEL.md).

---

## Документация

| Документ | О чём |
|---|---|
| [JURY_README.md](JURY_README.md) | **Инструкция для жюри**: сборка, запуск на bag, входные и выходные топики |
| [cpp/README.md](cpp/README.md) | Статус C++ ноды: что проверено, что осознанно не сделано, выгрузка обучающих данных |
| [MODEL.md](MODEL.md) | Математика решения: физика тяги, ESKF, проекция на маршрут, ML-корректор |
| [LIMITATIONS.md](LIMITATIONS.md) | Допущения, ограничения и план доработки |
| [ml/docs/ML_CONTRACT.md](ml/docs/ML_CONTRACT.md) | Контракт ML-артефакта: имена и порядок признаков, схема, происхождение весов |
| [cpp/tram_odometry/models/README.md](cpp/tram_odometry/models/README.md) | Формат и правила для файлов модели в рантайме |
| [ml/docs/FEATURES.md](ml/docs/FEATURES.md) | Признаки обучающей таблицы (Python-сторона) |

---

## Структура репозитория

```
cpp/                     ROS 2 (Humble, C++17) — рантайм
  tram_vehicle_msgs/     VelocitySensor, DriverControllerCommand
  tram_odometry/         ядро оценки + нода + конфиг + тесты
ml/                      Python-конвейер обучения (пакет odom_ml)
  src/odom_ml/           библиотека: данные, признаки, ESKF, модели
  scripts/               пронумерованные скрипты конвейера 01..23
  tests/                 pytest
  docs/                  контракты и заметки по ML
artifacts/               обученные модели (JSON + joblib)
models/                  выгрузка для C++ (дескриптор + веса + отчёт)
docs/                    материалы хакатона
data/                    датасет bag'ов — НЕ в git, см. ниже
```

---

## Быстрый старт

Полная инструкция — в [JURY_README.md](JURY_README.md). Коротко:

```bash
# 1. Собрать (tram_vehicle_msgs собирается первым — это зависимость)
cp -r cpp/tram_vehicle_msgs cpp/tram_odometry <ws>/src/
cd <ws> && colcon build --packages-select tram_vehicle_msgs tram_odometry
source install/setup.bash

# 2. Запустить ноду и bag двумя терминалами
ros2 launch tram_odometry replay.launch.py use_clock:=true
ros2 bag play <bag> --clock
```

Публикует `/result/velocity` (м/с) и `/result/position` на 50 Гц.

---

## Данные

Датасет с bag-ами (~824 МБ rosbag2 sqlite3) **в git не входит** — организаторы
распространяют его отдельно. Скопировать в корень репозитория:

```bash
robocopy <dataset>\data  data\  /E
robocopy <dataset>\tram_vehicle_msgs  data\tram_vehicle_msgs  /E
```

Кэш `.cache/` (~480 МБ, ресемплированные таблицы) генерируется заново примерно
за минуту — после `pip install -e ml/` (см. ниже) командой
`python -m odom_ml.data.build`.

---

## Тесты

```bash
# C++ — 42 теста: 40 gtest (test_core 22 + test_ml 18) + 2 служебных ctest
colcon build --packages-select tram_vehicle_msgs tram_odometry
colcon test --packages-select tram_odometry && colcon test-result --verbose

# Python — 77 быстрых тестов
pip install -e ml/
pytest ml/tests

# Python — 12 медленных (прогоняют весь кэш прогонов, ~8 мин)
pytest ml/tests -m slow
```

---

## Статус проверки

Фиксировано по состоянию репозитория.

**Работает и проверено:**

- **Полная сборка под ROS 2 Humble:** `colcon build` для `tram_vehicle_msgs` и
  `tram_odometry` проходит без ошибок, `colcon test` — **42 теста, 0 ошибок,
  0 падений**. Это 40 gtest-кейсов (`test_core` 22 + `test_ml` 18) плюс 2
  служебных теста, которые добавляет ctest. Проверено в контейнере из
  `cpp/Dockerfile`, то есть весь C++-код ноды, включая `odometry_node.cpp`,
  `params.cpp`, `inference.cpp` и `ml_corrector.cpp`.
- Python-конвейер компилируется целиком (55 файлов), `pytest ml/tests` — **77 passed**.
- ML-артефакт настоящий, не заглушка: обучена одна голова `a_residual`,
  RMSE **0.205 против baseline 0.919** (в 4.5 раза) на 6.6 млн строк,
  5-кратный GroupKFold по прогонам — см. `models/validation_report.json`.

**Известные проблемы:**

1. **2 теста `ml/tests/test_registration.py` падают** — сохранённая регистрация
   расходится со свежим фитом на 0.164° при допуске 0.05°. Падает и на коде до
   текущей чистки, значение совпадает — проблема не связана с ней.
2. **Модель обучает 1 выход из 3.** Строки `log_scale` и `mu` обнулены намеренно:
   их коррекция ухудшала метрики. Фактически контракт из трёх выходов работает
   как один.
3. **Проверка на утечку данных не отработала** — `"leakage": {"pearson_r": null, "n": 0}`.
4. **CI в репозитории отсутствует**, хотя `bench_core.cpp` описан как проверяемый
   в CI. Сборка и тесты воспроизводятся через `cpp/Dockerfile`, но автоматического
   прогона при коммите нет.
5. **Прогон ноды на реальном bag не выполнялся.** Сборка и тесты проходят, но
   качество оценки на данных из `data/` здесь не измерялось.

Остальное — осознанные технические решения и нереализованные пункты — перечислено
в [cpp/README.md](cpp/README.md) §3 и в [LIMITATIONS.md](LIMITATIONS.md).