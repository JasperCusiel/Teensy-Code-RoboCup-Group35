//
// Created by Jasper Cusiel on 03/09/2026.
//

#include "pure-pursuit.h"
#include "occupancy-grid.h"
#include "mapping.h"

#include <math.h>
#include <Arduino.h>
#include <wiring.h>

// This module implements the pure pursuit algorthm which takes a set of points and generates smooth motion commands to link the points.

namespace
{
    // PP config
    constexpr float kLookaheadDistanceM = 0.50f;
    constexpr float kGoalToleranceM = 0.15f;
    constexpr float kNominalSpeedMps = 0.30f;
    constexpr float kMinimumTrackingSpeedMps = 0.1f;

    // Turn-in-place config
    constexpr float kTurnInPlaceEnterErrorRad = PI / 4.0f; // 45 deg
    constexpr float kTurnInPlaceExitErrorRad = PI / 12.0f; // 15 deg
    constexpr float kTurnInPlaceKp = 2.0f;
    constexpr float kMaxTurnInPlaceRateRadPerSec = 1.0f;

    constexpr float kMaxTrackingTurnRateRadPerSec = 2.5f;

    bool turning_in_place = false;

    // Helper functions.
    float wrap_angle(float angle)
    {
        while (angle > PI) angle -= 2.0f * PI;
        while (angle < -PI) angle += 2.0f * PI;
        return angle;
    }

    void cell_to_world(grid_point_t cell, float* x, float* y)
    {
        *x = MAP_WORLD_MIN_X + (static_cast<float>(cell.x) + 0.5f) * MAP_M_PER_CELL;
        *y = MAP_WORLD_MIN_Y + (static_cast<float>(cell.y) + 0.5f) * MAP_M_PER_CELL;
    }

    float squared_distance_to_cell(const pose_t* pose, grid_point_t cell)
    {
        float x = 0.0f;
        float y = 0.0f;
        cell_to_world(cell, &x, &y);
        const float dx = x - pose->x;
        const float dy = y - pose->y;
        return dx * dx + dy * dy;
    }

    int find_lookahead_index(const path_t* path, const pose_t* pose)
    {
        // Finds next cell to look at to smooth path.
        int nearest = 0;
        float nearest_distance = squared_distance_to_cell(pose, path->points[0]);
        for (int i = 1; i < path->length; ++i)
        {
            const float distance = squared_distance_to_cell(pose, path->points[i]);
            if (distance <= nearest_distance)
            {
                nearest = i;
                nearest_distance = distance;
            }
        }
        float accumulated = 0.0f;
        int lookahead = nearest;
        for (int i = nearest + 1; i < path->length; ++i)
        {
            const int dx = path->points[i].x - path->points[i - 1].x;
            const int dy = path->points[i].y - path->points[i - 1].y;
            accumulated += sqrtf(static_cast<float>(dx * dx + dy * dy)) * MAP_M_PER_CELL;
            lookahead = i;
            if (accumulated >= kLookaheadDistanceM) break;
        }
        return lookahead;
    }

    float heading_to_cell(const pose_t* pose, grid_point_t cell)
    {
        float x = 0.0f;
        float y = 0.0f;
        cell_to_world(cell, &x, &y);
        // This project defines heading zero along world +Y and positive CCW.
        return atan2f(-(x - pose->x), y - pose->y);
    }
} // namespace

velocity_command_t pure_pursuit_update(const path_t* path, const pose_t* robot_pose)
{
    // Function generates velocity command based on the next point on the path.
    velocity_command_t command = {
        robot_pose != nullptr ? robot_pose->theta : 0.0f,
        0.0f, 0.0f, true
    };

    if (path == nullptr || robot_pose == nullptr || path->length == 0 ||
        pure_pursuit_path_complete(path, robot_pose))
    {
        turning_in_place = false;
        // Serial.println(
        //     "path == nullptr || robot_pose == nullptr || path->length == 0 || pure_pursuit_path_complete(path, robot_pose)");
        return command;
    }

    const int lookahead_index = find_lookahead_index(path, robot_pose);
    const grid_point_t lookahead_point = path->points[lookahead_index];

    // Desired heading towards lookahead point.
    command.heading = heading_to_cell(robot_pose, lookahead_point);

    const float heading_error = wrap_angle(command.heading - robot_pose->theta);

    // Enter turn-in-place mode if we are facing substantially
    // away from the path.
    if (!turning_in_place &&
        fabsf(heading_error) >= kTurnInPlaceEnterErrorRad)
    {
        turning_in_place = true;
    }

    // Once turning in place, remain in this mode until we are
    // well aligned with the path. This hysteresis prevents rapid
    // switching between tracking and turn-in-place.
    if (turning_in_place)
    {
        if (fabsf(heading_error) <= kTurnInPlaceExitErrorRad)
        {
            turning_in_place = false;
        }
        else
        {
            command.linear_speed = 0.0f;

            command.turn_rate =
                kTurnInPlaceKp * heading_error;

            command.turn_rate = fmaxf(
                -kMaxTurnInPlaceRateRadPerSec,
                fminf(
                    kMaxTurnInPlaceRateRadPerSec,
                    command.turn_rate));

            command.stop = false;
            // Serial.println("else");
            return command;
        }
    }

    // Actual distance from robot to selected lookahead point.
    const float lookahead_distance = sqrtf(squared_distance_to_cell(robot_pose, lookahead_point));

    // Protect against division by zero.
    if (lookahead_distance < 0.001f)
    {
        // Serial.println("lookahead_distance < 0.001f");
        return command;
    }

    // Reduce forward speed as heading error increases.
    const float alignment = fmaxf(0.0f, cosf(heading_error));

    command.linear_speed = fmaxf(kMinimumTrackingSpeedMps, kNominalSpeedMps * alignment);

    // Pure Pursuit curvature:
    //
    //             2 sin(alpha)
    // curvature = ------------
    //                  Ld
    //
    const float curvature = 2.0f * sinf(heading_error) / lookahead_distance;

    // Convert curvature into angular velocity:
    //
    // omega = v * curvature
    //
    command.turn_rate = command.linear_speed * curvature;

    // Limit angular velocity while tracking.
    command.turn_rate = fmaxf(-kMaxTrackingTurnRateRadPerSec, fminf(kMaxTrackingTurnRateRadPerSec, command.turn_rate));

    command.stop = false;

    // Serial.printf(
    //     "PP: idx=%d heading=%.1f err=%.1f v=%.3f w=%.3f\n",
    //     lookahead_index,
    //     command.heading * 180.0f / PI,
    //     heading_error * 180.0f / PI,
    //     command.linear_speed,
    //     command.turn_rate
    // );

    return command;
}

bool pure_pursuit_path_complete(const path_t* path, const pose_t* robot_pose)
{
    // Assume completed when we are within kGoalToleranceM of the final point.
    if (path == nullptr || robot_pose == nullptr || path->length == 0)
    {
        return true;
    }
    return squared_distance_to_cell(robot_pose, path->points[path->length - 1]) <= kGoalToleranceM * kGoalToleranceM;
}
