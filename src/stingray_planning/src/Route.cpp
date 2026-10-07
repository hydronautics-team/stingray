#include "stingray_planning/Route.hpp"
#include <cmath>
#include <vector>
#include <queue>
#include <algorithm>
#include <iostream>


namespace stingray::planning{

void obstacles_inflation(std::vector<uint8_t>& field, const GRID& grid, Point center, int radius) {
    for (int x = center.x - radius; x <= center.x + radius; x++) {
        for (int y = center.y - radius; y <= center.y + radius; y++) {
            if (!grid.inside(x, y)) {
                continue;
            }
            if (heuristic(center.x, center.y, x, y) <= radius * radius) {
                field[grid.index(x, y)] = 1;
            }
        }
    }
}

std::vector<Point> theta_star(Point start, Point target, const GRID& grid, const std::vector<uint8_t>& temp_field) {
    if (!grid.inside(start.x, start.y) || !grid.inside(target.x, target.y)) {
        return {};
    }
    if (grid.field[grid.index(start)] != 0 || grid.field[grid.index(target)] != 0) {
        return {};
    }
    int size = grid.x_size * grid.y_size;
    std::vector<double> g_score(size, INFINITY);
    std::vector<uint8_t> closed_set(size, 0);
    std::vector<Point> parents(size);

    using QueueNode = std::pair<double, Point>;
    auto compare = [](const QueueNode& a, const QueueNode& b) {
        return a.first > b.first;
    };
    std::priority_queue<QueueNode, std::vector<QueueNode>, decltype(compare)> open(compare);
    
    int start_index = grid.index(start);
    g_score[start_index] = 0;
    parents[start_index] = start;
    open.push({sqrt(heuristic(start.x, start.y, target.x, target.y)), start});

    const int directions[8][2] = {
        {-1,  0}, { 1,  0}, { 0, -1}, { 0,  1},
        {-1, -1}, {-1,  1}, { 1, -1}, { 1,  1}
    };

    while (!open.empty()) {
        Point current = open.top().second;
        open.pop();
        int current_index = grid.index(current);
        if (closed_set[current_index]) {
            continue;
        }
        closed_set[current_index] = 1;

        if (current.x == target.x && current.y == target.y) {
            std::vector<Point> path;
            Point p = target;
            while (!(p.x == start.x && p.y == start.y)) {
                path.push_back(p);
                p = parents[grid.index(p)];
            }
            path.push_back(start);
            std::reverse(path.begin(), path.end());
            return path;
        }

        for (int i = 0; i < 8; i++) {
            Point neighbor{current.x + directions[i][0], current.y + directions[i][1]};
            if (!grid.inside(neighbor.x, neighbor.y)) {
                continue;
            }
            int neighbor_index = grid.index(neighbor);
            if (grid.field[neighbor_index] == 1 || temp_field[neighbor_index] == 1) {
                continue;
            }
            if (!grid.valid_move(current, neighbor)) {
                continue;
            }

            Point current_parent = parents[current_index];
            double new_g;
            if (grid.line_of_sight(current_parent, neighbor, temp_field)) {
                new_g = g_score[grid.index(current_parent)] + sqrt(heuristic(current_parent.x, current_parent.y, neighbor.x, neighbor.y));
                if (new_g < g_score[neighbor_index]) {
                    g_score[neighbor_index] = new_g;
                    parents[neighbor_index] = current_parent;
                    double f = new_g + sqrt(heuristic(neighbor.x, neighbor.y, target.x, target.y));
                    open.push({f, neighbor});
                }
            } else {
                new_g = g_score[current_index] + sqrt(heuristic(current.x, current.y, neighbor.x, neighbor.y));
                if (new_g < g_score[neighbor_index]) {
                    g_score[neighbor_index] = new_g;
                    parents[neighbor_index] = current;
                    double f = new_g + sqrt(heuristic(neighbor.x, neighbor.y, target.x, target.y));
                    open.push({f, neighbor});
                }
            }
        }
    }
    return {};
}

GRID::GRID(int grid_x_size, int grid_y_size, std::vector<Point> grid_targets, std::vector<Point> grid_obstacles) {
    x_size = grid_x_size;
    y_size = grid_y_size;
    targets = grid_targets;
    obstacles = grid_obstacles;
    field.resize(grid_x_size * grid_y_size, 0);
}

bool GRID::valid_move(Point current, Point neighbor) const {
    int x0 = current.x;
    int y0 = current.y;
    int x1 = neighbor.x;
    int y1 = neighbor.y;

    if ((x1 - x0 != 0) && (y1 - y0 != 0)) {
        if ((field[index(x1, y0)] == 1) || (field[index(x0, y1)] == 1)) {
            return false;
        }
    }
    return true;
}

bool GRID::line_of_sight(Point parent, Point neighbor, const std::vector<uint8_t>& clean_field) const {
    std::vector<Point> cells = bresenham(parent, neighbor);
    for (size_t i = 0; i < cells.size(); i++) {
        if (inside(cells[i].x, cells[i].y)) {
            if (clean_field[index(cells[i])] == 1 || field[index(cells[i])] == 1) {
                return false;
            }
        } else {
            return false;
        }
    }
    return true;
}

std::vector<Point> AUV::build_full_route(const std::vector<Point>& targets, const GRID& grid) {
    std::vector<Point> full_path;
    Point current = start_point;
    GRID navigation_grid = grid;
    const std::vector<uint8_t> original_field = grid.field;
    for (int y = 0; y < grid.y_size; ++y) {
        for (int x = 0; x < grid.x_size; ++x) {
            if (original_field[grid.index(x, y)] != 0) {
                obstacles_inflation(navigation_grid.field, grid, {x, y}, radius);
            }
        }
    }
    for (size_t i = 0; i < targets.size(); i++) {
        Point current_target = targets[i];
        std::vector<uint8_t> temp_field(grid.x_size * grid.y_size, 0);
        for (size_t j = i + 1; j < targets.size(); j++) {
            obstacles_inflation(temp_field, grid, targets[j], radius);
        }
        std::vector<Point> path_segment = theta_star(
            current, current_target, navigation_grid, temp_field);
        if (path_segment.empty()) {
            return {};
        }
        if (!full_path.empty()) {
            full_path.insert(full_path.end(), path_segment.begin() + 1, path_segment.end());
        } else {
            full_path.insert(full_path.end(), path_segment.begin(), path_segment.end());
        }
        current = current_target;
    }
    return full_path;
}


AUV::AUV(Point start, double length, double weight, double max_vel, double min_vel, double max_angle_vel, double min_angle_vel) {
    start_point = start;
    radius = static_cast<int>(std::ceil(0.5 * sqrt(length * length + weight * weight)));
    max_velocity = max_vel;
    min_velocity = min_vel;
    max_angle_velocity = max_angle_vel;
    min_angle_velocity = min_angle_vel;
}

std::vector<Point> bresenham(Point start, Point finish) {
    std::vector<Point> cells;
    int dx = std::abs(finish.x - start.x);
    int dy = std::abs(finish.y - start.y);
    int Sx = (start.x < finish.x) ? 1 : -1;
    int Sy = (start.y < finish.y) ? 1 : -1;
    int err = dx - dy;

    while (true) {
        cells.push_back({start.x, start.y});
        if (start.x == finish.x && start.y == finish.y) {
            break;
        }
        int local_err = err * 2;
        if (local_err > -dy) {
            err -= dy;
            start.x += Sx;
        }
        if (local_err < dx) {
            err += dx;
            start.y += Sy;
        }
    }
    return cells;
}


double normalize_angle(double angle) {
    while (angle > 180.0) angle -= 360.0;
    while (angle < -180.0) angle += 360.0;
    return angle;
}

std::vector<WaypointCommand> compute_commands(const std::vector<Point>& path, double max_vel, double min_vel, double max_angle_vel, double min_angle_vel, double /*K*/){
    std::vector<WaypointCommand> result;
    if (path.empty()) return result;
    const size_t n = path.size();
    result.resize(n);
    
    if (n == 1) {
        result[0] = {0.0, 0.0, 0.0, 0.0, 0.0};
        return result;
    }
    std::vector<double> turn_angles(n, 0.0);
    std::vector<double> velocities(n, 0.0);
    std::vector<double> x_vel(n, 0.0);
    std::vector<double> y_vel(n, 0.0);
    std::vector<double> angle_vel(n, 0.0);

    double first_dx = path[1].x - path[0].x;
    double first_dy = path[1].y - path[0].y;
    turn_angles[0] = normalize_angle(-(std::atan2(first_dy, first_dx) * 180.0 / M_PI));
    if (std::abs(turn_angles[0]) > 1e-9) {
        angle_vel[0] = std::copysign(
            min_angle_vel + (std::abs(turn_angles[0]) / 180.0) * (max_angle_vel - min_angle_vel),
            turn_angles[0]);
    }

    for (size_t i = 1; i < n - 1; ++i) {
        double in_dx = path[i].x - path[i - 1].x;
        double in_dy = path[i].y - path[i - 1].y;

        double out_dx = path[i + 1].x - path[i].x;
        double out_dy = path[i + 1].y - path[i].y;

        double in_angle  = std::atan2(in_dy, in_dx) * 180.0 / M_PI;
        double out_angle = std::atan2(out_dy, out_dx) * 180.0 / M_PI;

        turn_angles[i] = normalize_angle(in_angle - out_angle);
        if (std::abs(turn_angles[i]) > 1e-9) {
            angle_vel[i] = std::copysign(
                min_angle_vel + (std::abs(turn_angles[i]) / 180.0) * (max_angle_vel - min_angle_vel),
                turn_angles[i]);
        }
    }

    turn_angles[n - 1] = 0.0;

    for (size_t i = 0; i < n - 1; ++i) {
        if (i == 0){
            double next_turn = turn_angles[i + 1];
            velocities[i] = min_vel + (max_vel - min_vel) * std::cos(0.5 * next_turn * M_PI / 180.0);
            double out_angle = std::atan2(first_dy, first_dx);
            x_vel[i] = velocities[i] * std::cos(out_angle);
            y_vel[i] = velocities[i] * std::sin(out_angle);
        }
        else{
            double out_dx = path[i + 1].x - path[i].x;
            double out_dy = path[i + 1].y - path[i].y;
            double out_angle = std::atan2(out_dy, out_dx);

            double next_turn = turn_angles[i + 1];
            velocities[i] = min_vel + (max_vel - min_vel) * std::cos(0.5 * next_turn * M_PI / 180.0);
            x_vel[i] = velocities[i] * std::cos(out_angle);
            y_vel[i] = velocities[i] * std::sin(out_angle);
        }
        
    }

    velocities[n - 1] = 0.0; 
    angle_vel[n - 1] = 0.0;
    
    for (size_t i = 0; i < n; ++i) {
        result[i].turn_angle = turn_angles[i];
        result[i].velocity = velocities[i];
        result[i].x_vel = x_vel[i];
        result[i].y_vel = y_vel[i];
        result[i].angle_vel = angle_vel[i];
    }
    return result;
}

void angle_velocity_output(const std::vector<Point>& path, double max_vel, double min_vel, double max_angle_vel, double min_angle_vel, double K){
    auto cmds = compute_commands(path, max_vel, min_vel, max_angle_vel, min_angle_vel, K);
    if (cmds.empty()) return;
    std::cout << "Point\t\tTurn Angle\tVelocity\tX vel\tY vel\tAngle Velocity\n";
    for (size_t i = 0; i < cmds.size(); ++i) {
        std::cout << path[i].x << ", " << path[i].y << "\t\t"
                  << cmds[i].turn_angle << "\t\t"
                  << cmds[i].velocity   << ", "
                  << cmds[i].x_vel      << ", "
                  << cmds[i].y_vel      << "\t"
                  << cmds[i].angle_vel  << std::endl;
    }
}

}
