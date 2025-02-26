#!/bin/bash
set -e

# Проверяем, существует ли файл setup.bash
if [ -f install/setup.bash ]; then
  echo "[INFO] Код уже сбилжен"
  source install/setup.bash
else
  echo "[INFO] Файл install/setup.bash не найден. Запускаем сборку..."
  # Выполняем сборку, при ошибке удаляем build, install, log и выходим
  source "/opt/ros/jazzy/setup.bash"
  if ! colcon build --packages-select stingray_cam stingray_launch; then
    echo "[ERROR] Сборка завершилась с ошибкой. Удаляем build, install, log..."
    rm -rf build install log
    exit 1
  fi
  echo "[INFO] Сборка завершена успешно. Выполняем source install/setup.bash..."
  source install/setup.bash
fi

# exec "$@"
exec ros2 launch stingray_launch zbar.launch.py
