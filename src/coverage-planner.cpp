//
// Created by Jasper Cusiel on 16/09/2026.
//

#include "coverage-planner.h"

#include "occupancy-grid.h"
#include "odometry.h"
#include "mapping.h"

namespace
{
    constexpr int kSurveySpacingCells = 6; // 0.6 m at the current map resolution.
    constexpr int kCoverageMarginCells = 3; // Keep survey points away from arena edges.
    constexpr int kStartY = 1;
    constexpr int kCandidateClearanceCells = 3; // Reject cells close to mapped obstacles.
    constexpr uint16_t kMaxCoverageGoals = 128;

    grid_point_t goals[kMaxCoverageGoals] = {};
    uint16_t goal_count = 0;
    uint16_t current_goal = 0;
    bool started = false;

    bool in_map(int x, int y)
    {
        return x >= 0 && y >= 0 && x < MAP_WIDTH && y < MAP_HEIGHT;
    }

    bool cell_has_clearance(int x, int y)
    {
        if (!in_map(x, y) || map_get_state(x, y) != FREE)
        {
            return false;
        }

        for (int dx = -kCandidateClearanceCells; dx <= kCandidateClearanceCells; ++dx)
        {
            for (int dy = -kCandidateClearanceCells; dy <= kCandidateClearanceCells; ++dy)
            {
                if (dx * dx + dy * dy >
                    kCandidateClearanceCells * kCandidateClearanceCells)
                {
                    continue;
                }

                const int nx = x + dx;
                const int ny = y + dy;
                if (!in_map(nx, ny) || map_get_state(nx, ny) == OCCUPIED)
                {
                    return false;
                }
            }
        }

        return true;
    }

    void append_goal(int x, int y)
    {
        if (goal_count >= kMaxCoverageGoals || !in_map(x, y))
        {
            return;
        }

        goals[goal_count++] = {x, y};
    }

    void build_lawnmower_goals()
    {
        goal_count = 0;
        current_goal = 0;

        bool left_to_right = true;

        // Home is at the low-Y end of the arena. Begin at the opposite end so
        // the robot works back toward home as the round progresses.
        for (int y = MAP_HEIGHT - 1 - kCoverageMarginCells;
             y >= kStartY + kCoverageMarginCells;
             y -= kSurveySpacingCells)
        {
            if (left_to_right)
            {
                for (int x = kCoverageMarginCells;
                     x < MAP_WIDTH - kCoverageMarginCells;
                     x += kSurveySpacingCells)
                {
                    append_goal(x, y);
                }
            }
            else
            {
                for (int x = MAP_WIDTH - 1 - kCoverageMarginCells;
                     x >= kCoverageMarginCells;
                     x -= kSurveySpacingCells)
                {
                    append_goal(x, y);
                }
            }

            left_to_right = !left_to_right;
        }
    }
} // namespace

void coverage_planner_init()
{
    coverage_planner_reset();
}

void coverage_planner_reset()
{
    goal_count = 0;
    current_goal = 0;
    started = false;
}

void coverage_planner_start()
{
    started = true;
    build_lawnmower_goals();
}

bool coverage_planner_started() { return started; }

bool coverage_planner_get_goal(grid_point_t* goal)
{
    if (goal == nullptr || !started)
    {
        return false;
    }

    pose_t pose = {};
    get_ekf_pose(&pose.x, &pose.y, &pose.theta);

    grid_point_t robot_cell = {};

    if (!world_to_map(
        pose.x, pose.y,
        &robot_cell.x, &robot_cell.y))
    {
        return false;
    }

    bool reachable[MAP_WIDTH][MAP_HEIGHT] = {};

    if (!astar_build_reachable_mask(
        robot_cell.x,
        robot_cell.y,
        reachable))
    {
        return false;
    }

    for (uint16_t candidate_index = current_goal;
         candidate_index < goal_count;
         ++candidate_index)
    {
        const grid_point_t candidate = goals[candidate_index];

        if (!cell_has_clearance(candidate.x, candidate.y))
        {
            continue;
        }

        if (!reachable[candidate.x][candidate.y])
        {
            continue;
        }

        current_goal = candidate_index;
        *goal = candidate;
        return true;
    }

    return false;
}

void coverage_planner_advance_goal()
{
    if (started && current_goal < goal_count)
    {
        ++current_goal;
    }
}

void coverage_planner_reject_goal()
{
    coverage_planner_advance_goal();
}

bool coverage_planner_complete()
{
    return started && current_goal >= goal_count;
}

uint16_t coverage_planner_goal_count() { return goal_count; }

uint16_t coverage_planner_goal_index() { return current_goal; }
