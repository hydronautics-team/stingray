#!/bin/bash
set -e

# Проверяем, существует ли файл setup.bash
if [ -f install/setup.bash ]; then
  echo "[INFO] Код уже сбилжен"
  source install/setup.bash
  source "/additional_packages/install/setup.bash"
else
  echo "[INFO] Файл install/setup.bash не найден. Запускаем сборку..."
  # Выполняем сборку, при ошибке удаляем build, install, log и выходим
  source "/opt/ros/jazzy/setup.bash"
  source "/additional_packages/install/setup.bash"
  if ! colcon build --packages-select stingray_devices stingray_interfaces stingray_launch stingray_missions stingray_movement stingray_utils stingray_core_interfaces stingray_core_communication stingray_core_launch; then
    echo "[ERROR] Сборка завершилась с ошибкой. Удаляем build, install, log..."
    rm -rf build install log
    exit 1
  fi
  echo "[INFO] Сборка завершена успешно. Выполняем source install/setup.bash..."
  source install/setup.bash
fi

# exec "$@"
exec ros2 launch stingray_launch control.launch.py
