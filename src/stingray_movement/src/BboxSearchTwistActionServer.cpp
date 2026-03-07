#include "stingray_movement/BboxSearchTwistActionServer.h"

BboxSearchTwistActionServer::BboxSearchTwistActionServer(
    std::shared_ptr<rclcpp::Node> _node,
    const std::string &actionName)
    : AbstractSearchTwistActionServer<
          stingray_interfaces::action::BboxSearchTwistAction,
          stingray_interfaces::action::BboxSearchTwistAction_Goal>(_node, actionName)
{
    // Инициализируем счётчики
    target_found_counter = 0;
    target_found_threshold = 0;
    found_target_yaw = 0.0f;
    target_yaw_step = 0.0f;
}

/**
 * @brief Колбэк подписки на массив Bbox. 
 *        Если нашли bbox с нужным именем (target_bbox_name), 
 *        увеличиваем target_found_counter, иначе сбрасываем.
 */
void BboxSearchTwistActionServer::bboxArrayCallback(const stingray_interfaces::msg::BboxArray &msg)
{
    bool found_target_this_frame = false;

    for (auto &bbox : msg.bboxes)
    {
        if (bbox.name == target_bbox_name)
        {
            target_found_counter++;
            found_target_this_frame = true;
            // Если мы уже превысили порог, запоминаем угол (один раз)
            if (target_found_counter > target_found_threshold)
            {
                found_target_yaw = bbox.horizontal_angle;
            }
            RCLCPP_INFO(_node->get_logger(), "target_found_counter %d", target_found_counter);
        }
    }

    // Если в этом кадре не видели цель, сбрасываем счётчик
    if (!found_target_this_frame)
    {
        target_found_counter = 0;
    }
}

/**
 * @brief Проверяет, закончились ли манёвры по глубине/роллу/питчу (если вам нужно).
 *        Сейчас сделано так, чтобы учитывать isDepthDone / isRollDone / isPitchDone,
 *        если это реализовано в AbstractSearchTwistActionServer. 
 *        Возвращает true, только если все достигнуты.
 */
bool BboxSearchTwistActionServer::isTwistDone(
    const std::shared_ptr<const stingray_interfaces::action::BboxSearchTwistAction_Goal> goal)
{
    // Если вы хотите, чтобы проверялись depth/roll/pitch, раскомментируйте:
    bool depth_ok = isDepthDone(goal->depth);
    bool roll_ok = isRollDone(goal->roll);
    bool pitch_ok = isPitchDone(goal->pitch);

    return (depth_ok && roll_ok && pitch_ok);
}

/**
 * @brief Проверяет, достигли ли мы порога «нашли цель» (т.е. consecutive frames).
 *        Как только счётчик consecutive frames (target_found_counter) 
 *        превысит target_found_threshold, считаем, что цель найдена.
 */
bool BboxSearchTwistActionServer::isSearchTwistDone()
{
    return (target_found_counter > target_found_threshold);
}

/**
 * @brief Основная логика выполнения экшена
 */
void BboxSearchTwistActionServer::execute(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<stingray_interfaces::action::BboxSearchTwistAction>> goal_handle)
{
    using ActionT = stingray_interfaces::action::BboxSearchTwistAction;
    RCLCPP_INFO(_node->get_logger(), "Execute BboxSearchTwistActionServer action...");

    auto goal_result = std::make_shared<ActionT::Result>();
    goal_result->success = false;
    goal_result->finded = false;

    // 1. Проверяем доступность сервиса
    while (!twistSrvClient->wait_for_service(1s))
    {
        if (!rclcpp::ok())
        {
            RCLCPP_ERROR(_node->get_logger(), "Interrupted while waiting for SetTwist service");
            goal_handle->abort(goal_result);
            return;
        }
        RCLCPP_WARN(_node->get_logger(), "SetTwist service not available, waiting...");
    }

    // 2. Получаем данные goal
    auto goal = goal_handle->get_goal();
    // if (goal->duration < 0.0)
    // {
    //     RCLCPP_ERROR(_node->get_logger(), "Duration must be >= 0.0");
    //     goal_handle->abort(goal_result);
    //     return;
    // }

    // 3. Создаём подписку на топик с bbox
    bboxArraySub = _node->create_subscription<stingray_interfaces::msg::BboxArray>(
        goal->bbox_topic, 10,
        std::bind(&BboxSearchTwistActionServer::bboxArrayCallback, this, std::placeholders::_1));

    // 4. Запоминаем параметры
    target_bbox_name       = goal->bbox_name;
    target_found_threshold = goal->found_threshold;
    target_yaw_step        = goal->first_clockwise ? goal->yaw_step : -goal->yaw_step;

    // Обнуляем счётчики
    target_found_counter = 0;
    found_target_yaw     = 0.0f;

    // 5. Задаём глубину/ролл/питч
    auto twistSrvRequest = std::make_shared<stingray_core_interfaces::srv::SetTwist::Request>();
    twistSrvRequest->depth = goal->depth;
    twistSrvRequest->roll  = goal->roll;
    twistSrvRequest->pitch = goal->pitch;
    // surge, если нужно
    // twistSrvRequest->surge = ???

    // 6. Если нужно ограничить общее время поиска – используем AsyncTimer
    // AsyncTimer timer(goal->duration * 1000);  // ms
    // if (goal->duration > 0.0)
    // {
    //     timer.start();
    // }

    // 7. Начальное значение yaw = текущий yaw
    float start_yaw = current_uv_state.yaw;
    twistSrvRequest->yaw = start_yaw;
    twistSrvClient->async_send_request(twistSrvRequest).wait();

    // 8. Частота цикла
    rclcpp::Rate checkRate(goal->search_rate > 0.0 ? goal->search_rate : 2.0);

    // === Основной цикл поиска ===
    while (rclcpp::ok())
    {
        // 8.1 Проверка на cancel
        if (goal_handle->is_canceling())
        {
            RCLCPP_INFO(_node->get_logger(), "Goal canceled");
            goal_result->success = false;
            goal_handle->canceled(goal_result);

            cleanupState();            // обнулить переменные, отписаться
            stopTwist(twistSrvRequest); // остановить движение
            return;
        }

        // 8.2 Если время ограничено, проверяем
        // if (goal->duration > 0.0 && !timer.isBusy())
        // {
        //     RCLCPP_INFO(_node->get_logger(), "Search time ended (duration=%.2f)", goal->duration);
        //     // Не нашли — завершаем с неудачей
        //     goal_result->success = false;
        //     goal_result->finded  = false;
        //     goal_handle->succeed(goal_result);

        //     cleanupState();
        //     stopTwist(twistSrvRequest);
        //     return;
        // }

        // 8.3 Проверяем, не достигли ли глубины/ролла/питча и не «нашли» ли мы bbox
        //     isTwistDone и isSearchTwistDone
        // Обратите внимание: isTwistDone сейчас у вас просто проверяет (depth/roll/pitch).
        // Если у вас нет строгой необходимости, можно убрать.
        if (isTwistDone(goal) && isSearchTwistDone())
        {
            RCLCPP_INFO(_node->get_logger(),
                "Target found and twist done; found_target_yaw=%.2f", found_target_yaw);

            // Повернёмся к нужному углу
            twistSrvRequest->yaw = found_target_yaw;
            twistSrvClient->async_send_request(twistSrvRequest).wait();

            goal_result->success = true;
            goal_result->finded  = true;
            goal_handle->succeed(goal_result);

            cleanupState();
            stopTwist(twistSrvRequest);
            return;
        }

        // 8.4 Проверяем, не вышли ли за предел yaw
        float yaw_diff = std::fabs(current_uv_state.yaw - start_yaw);
        if (yaw_diff > goal->max_yaw)
        {
            RCLCPP_INFO(_node->get_logger(),
                "Reached max_yaw=%.2f (current=%.2f, start=%.2f). Stopping search.",
                goal->max_yaw, current_uv_state.yaw, start_yaw);

            // Если хотим считать это «неудачей» — ставим success=false
            goal_result->success = false;
            goal_result->finded  = (isSearchTwistDone()); // если вдруг нашли
            goal_handle->succeed(goal_result);

            cleanupState();
            stopTwist(twistSrvRequest);
            return;
        }

        // 8.5 Прибавляем/вычитаем шаг yaw
        twistSrvRequest->yaw = current_uv_state.yaw + target_yaw_step;
        RCLCPP_INFO(_node->get_logger(),
            "Search: current_yaw=%.2f => next_yaw=%.2f", current_uv_state.yaw, twistSrvRequest->yaw);

        // Отправляем запрос
        twistSrvClient->async_send_request(twistSrvRequest).wait();

        // 8.6 Небольшая пауза
        checkRate.sleep();
    } // while (rclcpp::ok())

    // Если дошли сюда — значит rclcpp::ok()==false (выключение ROS).
    RCLCPP_WARN(_node->get_logger(), "ROS shutting down, abort search mission");
    goal_result->success = false;
    goal_handle->abort(goal_result);

    cleanupState();
    stopTwist(twistSrvRequest);
}

/**
 * @brief Утилита для очистки внутреннего состояния 
 *        после завершения или отмены экшена
 */
void BboxSearchTwistActionServer::cleanupState()
{
    // Отписываемся
    bboxArraySub.reset();

    // Обнуляем переменные
    target_bbox_name.clear();
    target_found_threshold = 0;
    target_found_counter   = 0;
    found_target_yaw       = 0.0f;
    target_yaw_step        = 0.0f;
}

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("bbox_search_twist_action_server");
    node->declare_parameter("bbox_search_twist_action", "/stingray/actions/bbox_search_twist");
    BboxSearchTwistActionServer server = BboxSearchTwistActionServer(node, node->get_parameter("bbox_search_twist_action").as_string());
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}