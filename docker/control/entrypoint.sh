#!/bin/bash
set -e

# Проверяем, существует ли файл setup.bash
if [ -f /stingray/install/setup.bash ]; then
  echo "[INFO] Код уже сбилжен"
  source /stingray/install/setup.bash
else
  echo "[INFO] Файл install/setup.bash не найден. Запускаем сборку..."
  # Выполняем сборку, при ошибке удаляем build, install, log и выходим

  source "/opt/ros/humble/setup.bash"
  source "/additional_packages/install/setup.bash"
  source /stingray_core/install/setup.bash

  if ! colcon build --packages-select stingray_devices stingray_interfaces stingray_launch stingray_missions stingray_movement stingray_utils; then
    echo "[ERROR] Сборка завершилась с ошибкой. Удаляем build, install, log..."
    rm -rf build install log
    exit 1
  fi
  echo "[INFO] Сборка завершена успешно. Выполняем source install/setup.bash..."
  source /stingray/install/setup.bash
fi

exec "$@"
