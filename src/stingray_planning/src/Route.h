#ifndef ROUTE_H
#define ROUTE_H

#include <vector>
#include <cstdint>

// Структура точки пространства
struct Point {
    int x;
    int y;
};

// Находит все точки, лежащие на прямой, соединяющей две точки
std::vector<Point> bresenham(Point start, Point finish);

// Класс сетки точек пространства
class GRID {
public:
    int x_size, y_size;
    std::vector<Point> targets, obstacles;
    std::vector<uint8_t> field;

    GRID(int grid_x_size, int grid_y_size, std::vector<Point> grid_targets, std::vector<Point> grid_obstacles);

    // Возвращает индекс точки в одномерном массиве по полям конструкции Point
    inline int index(Point p) const {
        return p.y * x_size + p.x;
    }
    // Возвращает индекс точки в одномерном массиве по координатам x,y
    inline int index(int x, int y) const {
        return y * x_size + x;
    }
    // Проверяет, лежит ли точка в границах сетки
    inline bool inside(int x, int y) const {
        return (x >= 0 && x < x_size && y >= 0 && y < y_size); 
    }

    // Проверяет, можно ли пройти из одной точки в другую
    bool valid_move(Point current, Point neighbor);
    // Проверяет, лежат ли точки на одной прямой без препятствий
    bool line_of_sight(Point parent, Point neighbor, std::vector<uint8_t> clean_field);
};

// Переводит метры в единицы сетки
inline double metres_to_grid_units(double dist, double K){
    return dist * K;
}
inline double grid_units_to_metres(double dist, double K){
    return dist / K;
}
// Расстояние между точками
inline int heuristic(int x0, int y0, int x1, int y1) {
    return (x1 - x0) * (x1 - x0) + (y1 - y0) * (y1 - y0);
}

// Надувание препятствий 
void obstacles_inflation(std::vector<uint8_t>& field, GRID grid, Point center, int radius);

// Поиск пути Theta*
std::vector<Point> theta_star(Point start, Point target, GRID grid, std::vector<uint8_t> temp_field);

// Класс аппарата
class AUV {
public:
    Point start_point;
    int radius;
    double max_velocity;
    double min_velocity;
    double max_angle_velocity;
    double min_angle_velocity;

    AUV(Point start, double length, double weight, double max_vel, double min_vel, double max_angle_vel, double min_angle_vel);
    std::vector<Point> build_full_route(std::vector<Point> targets, GRID grid);
};

// Вывод скорости и угла поворота для каждой точки
void angle_velocity_output(std::vector<Point> path, double max_vel, double min_vel, double max_angle_vel, double min_angle_vel, double K);

#endif