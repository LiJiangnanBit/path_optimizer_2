//
// Created by ljn on 20-1-26.
//
#include <cfloat>
#include "tools/tools.hpp"
#include "tools/spline.h"
#include "data_struct/data_struct.hpp"
#include "config/planning_flags.hpp"

namespace PathOptimizationNS {

double time_s(const clock_t &begin, const clock_t &end) {
    return static_cast<double>(end - begin) / CLOCKS_PER_SEC;
}

double time_ms(const clock_t &begin, const clock_t &end) {
    return static_cast<double>(end - begin) / CLOCKS_PER_SEC * 1000;
}

void time_s_out(const clock_t &begin, const clock_t &end, const std::string &text) {
    std::cout << text << " time cost: " << time_s(begin, end) << " s." << std::endl;
}

void time_ms_out(const clock_t &begin, const clock_t &end, const std::string &text) {
    std::cout << text << " time cost: " << time_ms(begin, end) << " ms." << std::endl;
}

bool isEqual(double a, double b) {
    return fabs(a - b) < FLAGS_epsilon;
}

double getHeading(const tk::spline &xs, const tk::spline &ys, double s) {
    double x_d1 = xs.deriv(1, s);
    double y_d1 = ys.deriv(1, s);
    return atan2(y_d1, x_d1);
}

double getCurvature(const tk::spline &xs, const tk::spline &ys, double tmp_s) {
    double x_d1 = xs.deriv(1, tmp_s);
    double y_d1 = ys.deriv(1, tmp_s);
    double x_d2 = xs.deriv(2, tmp_s);
    double y_d2 = ys.deriv(2, tmp_s);
    return (x_d1 * y_d2 - y_d1 * x_d2) / pow(pow(x_d1, 2) + pow(y_d1, 2), 1.5);
}

double distance(const State &p1, const State &p2) {
    return sqrt(pow(p1.x - p2.x, 2) + pow(p1.y - p2.y, 2));
}

State local2Global(const State &reference, const State &target) {
    double x = target.x * cos(reference.heading) - target.y * sin(reference.heading) + reference.x;
    double y = target.x * sin(reference.heading) + target.y * cos(reference.heading) + reference.y;
    double z = reference.heading + target.heading;
    return {x, y, z, target.k, target.s};
}

State global2Local(const State &reference, const State &target) {
    double dx = target.x - reference.x;
    double dy = target.y - reference.y;
    double x = dx * cos(reference.heading) + dy * sin(reference.heading);
    double y = -dx * sin(reference.heading) + dy * cos(reference.heading);
    double z = target.heading - reference.heading;
    return {x, y, z, target.k, 0};
}

State getProjection(const tk::spline &xs,
                    const tk::spline &ys,
                    double target_x,
                    double target_y,
                    double max_s,
                    double start_s) {
    if (max_s <= start_s) {
        return State{xs(start_s), ys(start_s)};
    }
    static const double grid = 1.0;
    double tmp_s = start_s, min_dis_s = start_s;
    auto min_dis = DBL_MAX;
    State target_state{target_x, target_y};
    while (tmp_s <= max_s) {
        State state_on_spline{xs(tmp_s), ys(tmp_s)};
        double tmp_dis = distance(state_on_spline, target_state);
        if (tmp_dis < min_dis) {
            min_dis = tmp_dis;
            min_dis_s = tmp_s;
        }
        tmp_s += grid;
    }
    State end_state(xs(max_s), ys(max_s), getHeading(xs, ys, max_s));
    end_state.s = max_s;
    auto max_s_distance = distance(end_state, target_state);
    if (max_s_distance < min_dis) {
        return end_state;
    }
    // Newton's method
    return getProjectionByNewton(xs, ys, target_x, target_y, max_s, min_dis_s);
}

State getProjectionByNewton(const tk::spline &xs,
                            const tk::spline &ys,
                            double target_x,
                            double target_y,
                            double max_s,
                            double hint_s) {
    hint_s = std::min(hint_s, max_s);
    double cur_s = hint_s;
    double prev_s = hint_s;
    for (int i = 0; i < 20; ++i) {
        double x = xs(cur_s);
        double y = ys(cur_s);
        double dx = xs.deriv(1, cur_s);
        double dy = ys.deriv(1, cur_s);
        double ddx = xs.deriv(2, cur_s);
        double ddy = ys.deriv(2, cur_s);
        // Ignore coeff 2 in J and H.
        double j = (x - target_x) * dx + (y - target_y) * dy;
        double h = dx * dx + (x - target_x) * ddx + dy * dy + (y - target_y) * ddy;
        cur_s -= j / h;
        if (fabs(cur_s - prev_s) < 1e-5) break;
        prev_s = cur_s;
    }

    cur_s = std::min(cur_s, max_s);
    State ret{xs(cur_s), ys(cur_s), getHeading(xs, ys, cur_s)};
    ret.s = cur_s;
    return ret;
}

State getDirectionalProjection(const tk::spline &xs,
                               const tk::spline &ys,
                               double target_x,
                               double target_y,
                               double angle,
                               double max_s,
                               double start_s) {
    if (max_s <= start_s) {
        return State{xs(start_s), ys(start_s), getHeading(xs, ys, start_s)};
    }
    static const double grid = 2.0;
    double tmp_s = start_s, min_dot_value_s = start_s;
    double v1 = sin(angle);
    double v2 = -cos(angle);
    auto min_dot_value = DBL_MAX;
    while (tmp_s <= max_s) {
        State state_on_spline{xs(tmp_s), ys(tmp_s)};
        double tmp_dot_value = fabs(v1 * (state_on_spline.x - target_x) + v2 * (state_on_spline.y - target_y));
        if (tmp_dot_value < min_dot_value) {
            tmp_dot_value = min_dot_value;
            min_dot_value_s = tmp_s;
        }
        tmp_s += grid;
    }
    // Newton's method
    return getDirectionalProjectionByNewton(xs, ys, target_x, target_y, angle, max_s, min_dot_value_s);
}

State getDirectionalProjectionByNewton(const tk::spline &xs,
                                       const tk::spline &ys,
                                       double target_x,
                                       double target_y,
                                       double angle,
                                       double max_s,
                                       double hint_s) {
    hint_s = std::min(hint_s, max_s);
    double cur_s = hint_s;
    double prev_s = hint_s;
    double v1 = sin(angle);
    double v2 = -cos(angle);
    for (int i = 0; i < 20; ++i) {
        double x = xs(cur_s);
        double y = ys(cur_s);
        double dx = xs.deriv(1, cur_s);
        double dy = ys.deriv(1, cur_s);
        double ddx = xs.deriv(2, cur_s);
        double ddy = ys.deriv(2, cur_s);
        // Ignore coeff 2 in J and H.
        double p1 = v1 * (x - target_x) + v2 * (y - target_y);
        double p2 = v1 * dx + v2 * dy;
        double j = p1 * p2;
        double h = p1 * (v1 * ddx + v2 * ddy) + p2 * p2;
        cur_s -= j / h;
        if (fabs(cur_s - prev_s) < 1e-5) break;
        prev_s = cur_s;
    }

    cur_s = std::min(cur_s, max_s);
    State ret{xs(cur_s), ys(cur_s), getHeading(xs, ys, cur_s)};
    ret.s = cur_s;
    return ret;
}

}
