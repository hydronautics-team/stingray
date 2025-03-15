#include "stingray_movement/BboxCenteringTwistActionServer.h"

#include <algorithm>  // std::find
#include <cmath>      // fabs

BboxCenteringTwistActionServer::BboxCenteringTwistActionServer(
    std::shared_ptr<rclcpp::Node> _node,
    const std::string &actionName)
    : AbstractCenteringTwistActionServer(_node, actionName)
{
    // Изначально считаем, что «ничего нет»
    current_target_bbox.pos_x = 1000.0f;
    current_target_bbox.pos_y = 1000.0f;
    current_target_bbox.pos_z = 1000.0f;

    current_avoid_target_bbox.pos_x = 1000.0f;
    current_avoid_target_bbox.pos_y = 1000.0f;
    current_avoid_target_bbox.pos_z = 1000.0f;
}

/**
 * @brief Колбэк подписки на топик с массивом bbox
 */
void BboxCenteringTwistActionServer::bboxArrayCallback(const stingray_interfaces::msg::BboxArray &msg)
{
    bool found_target = false;
    bool found_avoid_target = false;

    // Сбросим перед поиском
    // (если в этом кадре не найдём ни одного avoid, оставим bbox = 1000.f)
    current_avoid_target_bbox.pos_x = 1000.0f;
    current_avoid_target_bbox.pos_y = 1000.0f;
    current_avoid_target_bbox.pos_z = 1000.0f;

    for (auto &bbox : msg.bboxes)
    {
        // Проверяем, не является ли bbox объектом, который надо избегать
        if (std::find(
                target_avoid_bbox_name_array.begin(),
                target_avoid_bbox_name_array.end(),
                bbox.name) != target_avoid_bbox_name_array.end())
        {
            // Выбираем ближайший (по z) avoid
            if (bbox.pos_z < current_avoid_target_bbox.pos_z)
            {
                current_avoid_target_bbox = bbox;
                found_avoid_target = true;
            }
        }

        // Проверяем, не является ли bbox целевым
        if (bbox.name == target_bbox_name)
        {
            // Просто берём первый (или тоже можно выбирать ближайший)
            current_target_bbox = bbox;
            found_target = true;
            // Сбрасываем счётчик, что цель опять видим
            target_disappeared_counter = 0;
        }
    }

    // Если мы target не увидели в этом кадре – увеличиваем счётчик
    if (!found_target)
    {
        target_disappeared_counter++;
    }

    // Если ни одного avoid-объекта не нашли, оставляем current_avoid_target_bbox = 1000.f
    // (либо уже сброшен в начале, если хотите)
    // found_avoid_target – можно использовать, если нужно в логах отмечать, что avoid не найден
}

/**
 * @brief Проверка, закончили ли движение (по глубине / крену / тангажу).
 *
 * Если, например, в вашей логике нужно проверить, достигли ли желаемых глубины/ролла/питча.
 */
bool BboxCenteringTwistActionServer::isTwistDone(
    const std::shared_ptr<const stingray_interfaces::action::BboxCenteringTwistAction_Goal> goal)
{
    // Допустим, мы переопределили эти методы в базовом классе:
    // isDepthDone(goal->depth), isRollDone(goal->roll), isPitchDone(goal->pitch).
    // Если их нет — впишите нужную вам логику.
    return isDepthDone(goal->depth) && isRollDone(goal->roll) && isPitchDone(goal->pitch);
}

/**
 * @brief Проверка, достаточно ли близко подошли к цели по z
 */
bool BboxCenteringTwistActionServer::isCenteringTwistDone()
{
    return (current_target_bbox.pos_z < target_distance_threshold);
}

/**
 * @brief Проверка, не пропала ли цель надолго
 */
bool BboxCenteringTwistActionServer::isTargetLost()
{
    return (target_disappeared_counter > target_lost_thresh);
}

/**
 * @brief Основная логика экшена
 */
void BboxCenteringTwistActionServer::execute(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<stingray_interfaces::action::BboxCenteringTwistAction>> goal_handle)
{
    using stingray_interfaces::action::BboxCenteringTwistAction;

    RCLCPP_INFO(_node->get_logger(), "Execute BboxCenteringTwistActionServer action");

    // Подготовка к работе
    auto goal_result = std::make_shared<BboxCenteringTwistAction::Result>();
    goal_result->success = false;

    // Извлекаем goal
    auto goal = goal_handle->get_goal();

    // Проверяем duration
    if (goal->duration <= 0.0)
    {
        RCLCPP_ERROR(_node->get_logger(), "Duration must be > 0.0");
        goal_handle->abort(goal_result);
        return;
    }

    // Подписка на топик детекций
    bboxArraySub = _node->create_subscription<stingray_interfaces::msg::BboxArray>(
        goal->bbox_topic, 10,
        std::bind(&BboxCenteringTwistActionServer::bboxArrayCallback, this, std::placeholders::_1));

    // Сервис, отвечающий за движение
    // Проверяем доступность
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

    // Запоминаем имя target и avoid
    target_bbox_name = goal->bbox_name;
    target_avoid_bbox_name_array = goal->avoid_bbox_name_array;
    target_distance_threshold = goal->distance_threshold;
    target_lost_thresh = goal->lost_threshold;

    // Сбрасываем состояние текущих bbox
    current_target_bbox.pos_x = 1000.0f;
    current_target_bbox.pos_y = 1000.0f;
    current_target_bbox.pos_z = 1000.0f;
    current_avoid_target_bbox.pos_x = 1000.0f;
    current_avoid_target_bbox.pos_y = 1000.0f;
    current_avoid_target_bbox.pos_z = 1000.0f;
    target_disappeared_counter = 0;

    // Запускаем таймер
    AsyncTimer timer(goal->duration * 1000);
    timer.start();

    // Частота обновления управляющих воздействий
    rclcpp::Rate checkRate(goal->centering_rate > 0.0 ? goal->centering_rate : 0.5);

    // Основной цикл
    while (rclcpp::ok())
    {
        // 1. Проверка: отмена экшена?
        if (goal_handle->is_canceling())
        {
            RCLCPP_INFO(_node->get_logger(), "Goal canceled");
            goal_result->success = false;
            // Остановка
            stopTwist();
            goal_handle->canceled(goal_result);
            cleanupAfterFinish();
            return;
        }

        // 2. Проверка: вышло ли время?
        if (!timer.isBusy())
        {
            // Если к этому моменту потеряли цель – завершим экшен
            if (isTargetLost())
            {
                RCLCPP_ERROR(_node->get_logger(), "Duration ended, target lost!");
            }
            else
            {
                RCLCPP_WARN(_node->get_logger(), "Duration ended, finishing action by time");
            }
            goal_result->success = false;
            stopTwist();
            goal_handle->succeed(goal_result);
            cleanupAfterFinish();
            return;
        }

        // 3. Проверка: потеряли ли цель (слишком много кадров без неё)?
        if (isTargetLost())
        {
            RCLCPP_ERROR(_node->get_logger(), "Target lost for too many frames!");
            goal_result->success = false;
            stopTwist();
            goal_handle->succeed(goal_result);
            cleanupAfterFinish();
            return;
        }

        // 4. Проверка: достигли ли нужного «twist» (глубина/ролл/питч) и достаточно близко к цели?
        if (isTwistDone(goal) && isCenteringTwistDone())
        {
            RCLCPP_INFO(_node->get_logger(), "Centering done, finishing action");
            goal_result->success = true;
            stopTwist();
            goal_handle->succeed(goal_result);
            cleanupAfterFinish();
            return;
        }

        // Формируем новый запрос на движение
        auto twistSrvRequest = std::make_shared<stingray_core_interfaces::srv::SetTwist::Request>();
        twistSrvRequest->surge = goal->surge; // как задано в goal
        twistSrvRequest->depth = goal->depth;
        twistSrvRequest->roll = goal->roll;
        twistSrvRequest->pitch = goal->pitch;

        // === 5. Логика обхода (avoid) ===
        RCLCPP_INFO(_node->get_logger(), "Target distance: %f", current_target_bbox.pos_z);
        RCLCPP_INFO(_node->get_logger(), "Avoid target distance: %f", current_avoid_target_bbox.pos_z);
        bool needAvoid = (current_avoid_target_bbox.pos_z < goal->avoid_distance_threshold &&
                          std::fabs(current_avoid_target_bbox.pos_x) < goal->avoid_horizontal_threshold);
        if (needAvoid)
        {
            // Уходим в сторону, противоположную bbox.pos_x
            if (current_avoid_target_bbox.pos_x < 0.0f)
                twistSrvRequest->sway = -goal->sway;
            else
                twistSrvRequest->sway = goal->sway;

            RCLCPP_INFO(_node->get_logger(), "Avoiding obstacle: sway = %f", twistSrvRequest->sway);
        }
        // === 6. Логика центрирования по target ===
        else if (current_target_bbox.pos_x != 1000.0f)
        {
            // Простейший P-регулятор
            // Допустим, pos_x = смещение по горизонтали (пиксели / условная единица)
            float Kp = 0.5f; // подбирайте под себя
            float raw_cmd = Kp * current_target_bbox.pos_x;
            // Ограничим макс. команду
            if (raw_cmd > goal->sway)
                raw_cmd = goal->sway;
            if (raw_cmd < -goal->sway)
                raw_cmd = -goal->sway;

            twistSrvRequest->sway = raw_cmd;
            RCLCPP_INFO(_node->get_logger(), "Centering: sway = %f", twistSrvRequest->sway);
        }
        // 7. Если цели нет и обходить нечего — можно «шарить» или оставаться на месте
        else
        {
            // Пример: остаёмся на месте по оси sway
            twistSrvRequest->sway = 0.0f;
            RCLCPP_INFO(_node->get_logger(), "No target, no avoid, staying still");
        }

        // Поворот на угол target
        // (если у вас приходит horizontal_angle - прибавьте к текущему yaw)
        twistSrvRequest->yaw = current_uv_state.yaw + current_target_bbox.horizontal_angle;

        // 8. Отправляем запрос на сервис
        twistSrvClient->async_send_request(twistSrvRequest).wait();

        // 9. Делаем небольшой sleep, чтобы не забивать цикл
        checkRate.sleep();
    }

    // Если выходим из while — либо rclcpp::ok() = false
    RCLCPP_INFO(_node->get_logger(), "ROS shutdown or loop ended unexpectedly");
    stopTwist();
    goal_handle->abort(goal_result);
    cleanupAfterFinish();
}

/**
 * @brief Вспомогательный метод для очистки после завершения
 */
void BboxCenteringTwistActionServer::cleanupAfterFinish()
{
    // Сброс подписки
    bboxArraySub.reset();

    // Обнуление полей
    target_bbox_name.clear();
    target_avoid_bbox_name_array.clear();
    current_target_bbox.pos_x = 1000.0f;
    current_target_bbox.pos_y = 1000.0f;
    current_target_bbox.pos_z = 1000.0f;
    current_avoid_target_bbox.pos_x = 1000.0f;
    current_avoid_target_bbox.pos_y = 1000.0f;
    current_avoid_target_bbox.pos_z = 1000.0f;
    target_disappeared_counter = 0;
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("bbox_centering_twist_action_server");
    node->declare_parameter("bbox_centering_twist_action", "/stingray/actions/bbox_centering_twist");
    BboxCenteringTwistActionServer server = BboxCenteringTwistActionServer(node, node->get_parameter("bbox_centering_twist_action").as_string());
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
};