//
// Created by Jasper Cusiel on 03/09/2026.
//

#include "navigation.h"

#include "coverage-planner.h"
#include "frontier-detection.h"
#include "mission.h"
#include "odometry.h"
#include "occupancy-grid.h"
#include "mapping.h"
#include <limits.h>
#include <math.h>
#include <Arduino.h>

// Navigation module own the high-level target and planned path, other modules query this state.

namespace
{
    // How close we have to be to the goal before its marked as complete
    constexpr float kDefaultGoalToleranceM = 0.5f;
    constexpr float kCoverageGoalToleranceM = 0.25f;
    constexpr float kPi = 3.14159265f;
    constexpr float kNoFrontierScanAngleBeforeCompleteRad = 2.5f * kPi;
    constexpr uint8_t kRejectedFrontierCount = 8;
    constexpr uint8_t kRejectedFrontierRadiusCells = 3;
    constexpr uint8_t kPathValidationAheadCells = 14;
    constexpr uint8_t kReplansBeforeGoalReject = 5;

    constexpr float kHomeStagingY = 0.20f;

    navigation_goal_t active_goal = {NAV_GOAL_NONE, {0, 0}};
    // Current navigation target, either none, frontier or base.
    path_t active_path = {}; // Lastest A* path to goal
    navigation_status_t status = NAV_STATUS_IDLE; // What the nav is doing
    grid_point_t base_cell = {0, 0}; // Cell where the base is (for return to home)
    bool base_is_set = false; // Whether base cell is valid
    bool replan_requested = false; // Event flag to request path regeneration
    uint8_t replans_for_active_goal = 0;
    float no_frontier_scan_angle_rad = 0.0f;
    float last_no_frontier_heading_rad = 0.0f;
    bool no_frontier_scan_started = false;
    frontier_goal_t rejected_frontiers[kRejectedFrontierCount] = {};
    uint8_t rejected_frontier_count = 0;
    uint8_t next_rejected_frontier = 0;

    bool cells_equal(grid_point_t a, grid_point_t b)
    {
        return a.x == b.x && a.y == b.y;
    }

    float goal_tolerance()
    {
        // Tighter tolerance if we are trying to go home
        if (active_goal.type == NAV_GOAL_BASE)
        {
            return 0.12f;
        }
        // Relax tolerance if we are just doing coverage
        return active_goal.type == NAV_GOAL_COVERAGE
                   ? kCoverageGoalToleranceM
                   : kDefaultGoalToleranceM;
    }

    float wrap_angle(float angle)
    {
        while (angle > kPi) angle -= 2.0f * kPi;
        while (angle < -kPi) angle += 2.0f * kPi;
        return angle;
    }

    void reset_no_frontier_scan()
    {
        no_frontier_scan_angle_rad = 0.0f;
        last_no_frontier_heading_rad = 0.0f;
        no_frontier_scan_started = false;
    }

    void reset_replan_budget()
    {
        replans_for_active_goal = 0;
    }

    bool active_goal_can_be_rejected()
    {
        // We don't want to reject return home goal
        return active_goal.type == NAV_GOAL_FRONTIER ||
            active_goal.type == NAV_GOAL_COVERAGE;
    }

    bool no_frontier_scan_complete(float heading)
    {
        if (!no_frontier_scan_started)
        {
            last_no_frontier_heading_rad = heading;
            no_frontier_scan_started = true;
            return false;
        }

        no_frontier_scan_angle_rad += fabsf(wrap_angle(heading - last_no_frontier_heading_rad));
        last_no_frontier_heading_rad = heading;

        return no_frontier_scan_angle_rad >= kNoFrontierScanAngleBeforeCompleteRad;
    }

    // Converts a map cell index to world space coordinate of that cell's center.
    float cell_world_x(int x)
    {
        return MAP_WORLD_MIN_X + (static_cast<float>(x) + 0.5f) * MAP_M_PER_CELL;
    }

    float cell_world_y(int y)
    {
        return MAP_WORLD_MIN_Y + (static_cast<float>(y) + 0.5f) * MAP_M_PER_CELL;
    }

    // Consider goal reached when robot is within tolerance of the target cell centre.
    bool pose_at_goal(const pose_t& pose)
    {
        if (active_goal.type == NAV_GOAL_NONE)
        {
            return false;
        }
        const float dx = cell_world_x(active_goal.cell.x) - pose.x;
        const float dy = cell_world_y(active_goal.cell.y) - pose.y;
        const float tolerance = goal_tolerance();
        return dx * dx + dy * dy <= tolerance * tolerance;
    }


    bool choose_frontier(const grid_point_t& robot_cell, float robot_heading)
    {
        // Function finds the new frontier to navigate to. Only declare exploration
        // complete after scanning around in place with no accepted frontier.
        frontier_goal_t frontier = {};
        if (!frontier_find_largest_goal_excluding(robot_cell.x, robot_cell.y,
                                                  rejected_frontiers,
                                                  rejected_frontier_count,
                                                  kRejectedFrontierRadiusCells,
                                                  &frontier))
        {
            active_goal = {NAV_GOAL_NONE, {0, 0}};
            active_path.length = 0;
            reset_replan_budget();
            status = no_frontier_scan_complete(robot_heading)
                         ? NAV_STATUS_EXPLORATION_COMPLETE
                         : NAV_STATUS_IDLE;
            return false;
        }

        reset_no_frontier_scan();

        navigation_set_goal({NAV_GOAL_FRONTIER, {frontier.x, frontier.y}});
        return true;
    }

    bool choose_coverage_goal()
    {
        grid_point_t coverage_goal = {};
        if (!coverage_planner_get_goal(&coverage_goal))
        {
            active_goal = {NAV_GOAL_NONE, {0, 0}};
            active_path.length = 0;
            reset_replan_budget();
            // An unavailable coverage goal may become reachable as we move, only reject after so many fails.
            status = coverage_planner_complete()
                         ? NAV_STATUS_EXPLORATION_COMPLETE
                         : NAV_STATUS_IDLE;
            return false;
        }

        navigation_set_goal({NAV_GOAL_COVERAGE, coverage_goal});
        return true;
    }

    void start_coverage()
    {
        active_goal = {NAV_GOAL_NONE, {0, 0}};
        active_path.length = 0;
        replan_requested = false;
        reset_replan_budget();
        reset_no_frontier_scan();
        status = NAV_STATUS_IDLE;
        coverage_planner_start();
    }

    bool plan_from(const grid_point_t& start)
    {
        // Plan a path with A star
        status = NAV_STATUS_PLANNING;

        active_path.length = 0;
        active_path.escape_prefix_length = 0;
        replan_requested = false;

        const bool found = astar_find_path(
            start.x,
            start.y,
            active_goal.cell.x,
            active_goal.cell.y,
            &active_path);

        status = found ? NAV_STATUS_FOLLOWING_PATH : NAV_STATUS_PATH_FAILED;


        return found;
    }

    uint16_t closest_path_index(const grid_point_t& robot_cell)
    {
        // To get look ahead point
        if (active_path.length == 0)
        {
            return 0;
        }

        uint16_t closest_index = 0;
        int closest_distance = INT_MAX;

        for (uint16_t i = 0; i < active_path.length; ++i)
        {
            const int dx = active_path.points[i].x - robot_cell.x;
            const int dy = active_path.points[i].y - robot_cell.y;
            const int distance = dx * dx + dy * dy;

            if (distance < closest_distance)
            {
                closest_distance = distance;
                closest_index = i;
            }
        }

        return closest_index;
    }

    bool path_half_complete(const grid_point_t& robot_cell)
    {
        // Checks if we are half way through the path or not. Used to replan when half way
        if (active_path.length < 2)
        {
            return false;
        }

        const uint16_t closest_index = closest_path_index(robot_cell);
        return closest_index >= active_path.length / 2;
    }

    bool path_blocked_ahead(const grid_point_t& robot_cell)
    {
        // Check if path becomes blocked along the planned path as we move, replan if route it blocked.
        if (active_path.length == 0)
        {
            return false;
        }

        // A marked escape path must include a normal-clearance
        // destination after its escape prefix.
        if (active_path.escape_prefix_length > 0 &&
            active_path.escape_prefix_length >= active_path.length)
        {
            return true;
        }

        const uint16_t closest_index =
            closest_path_index(robot_cell);

        // Validate the next point rather than blindly skipping two.
        // The closest point represents our current path position.
        const uint16_t first_index = closest_index + 1;

        if (first_index >= active_path.length)
        {
            return false;
        }

        uint16_t end_index = closest_index + kPathValidationAheadCells + 1;

        if (end_index > active_path.length)
        {
            end_index = active_path.length;
        }

        for (uint16_t i = first_index; i < end_index; ++i)
        {
            const grid_point_t& point = active_path.points[i];

            if (point.x < 0 || point.x >= MAP_WIDTH ||
                point.y < 0 || point.y >= MAP_HEIGHT)
            {
                return true;
            }

            const uint8_t state =
                map_get_state(point.x, point.y);

            // No upcoming path point may be occupied.
            if (state == OCCUPIED)
            {
                return true;
            }

            const bool is_escape_point =
                i < active_path.escape_prefix_length;

            if (is_escape_point)
            {
                // Escape relaxes clearance to try to escape
                if (state != FREE)
                {
                    return true;
                }

                continue;
            }

            if (cells_equal(point, active_goal.cell))
            {
                continue;
            }

            if (state != FREE ||
                !astar_has_obstacle_clearance(point.x, point.y))
            {
                return true;
            }
        }

        return false;
    }
} // namespace

void navigation_init()
{
    // Save startup pose as return to home target (base position).
    active_goal = {NAV_GOAL_NONE, {0, 0}};
    active_path.length = 0;
    status = NAV_STATUS_IDLE;
    replan_requested = false;
    reset_replan_budget();
    coverage_planner_init();
    reset_no_frontier_scan();
    rejected_frontier_count = 0;
    next_rejected_frontier = 0;

    pose_t pose = {};
    get_ekf_pose(&pose.x, &pose.y, &pose.theta);
    base_is_set = world_to_map(
        pose.x,
        kHomeStagingY,
        &base_cell.x,
        &base_cell.y);
}

void navigation_task()
{
    // Main navigation state update loop. Converts mission intent into a goal
    // and ensures there is valid path from the robot's current cell to that goal.

    // Nav drops active target is mission commands stop.
    if (mission_should_stop())
    {
        if (active_goal.type != NAV_GOAL_NONE)
        {
            navigation_clear_goal();
        }
        return;
    }

    pose_t pose = {};
    get_ekf_pose(&pose.x, &pose.y, &pose.theta);
    grid_point_t robot_cell = {};
    if (!world_to_map(pose.x, pose.y, &robot_cell.x, &robot_cell.y))
    {
        active_path.length = 0;
        reset_replan_budget();
        status = NAV_STATUS_PATH_FAILED;
        return;
    }

    if (pose_at_goal(pose))
    {
        active_path.length = 0;
        reset_replan_budget();
        status = NAV_STATUS_GOAL_REACHED;
        if (mission_should_explore())
        {
            if (active_goal.type == NAV_GOAL_COVERAGE)
            {
                coverage_planner_advance_goal();
            }
            active_goal = {NAV_GOAL_NONE, {0, 0}};
            reset_replan_budget();
        }
        else
        {
            return;
        }
    }

    // Use frontier to start, then go to coverage during round
    if (mission_should_explore() && !coverage_planner_started() &&
        mission_should_use_coverage())
    {
        start_coverage();
    }

    // Refresh exploration goals once the robot has made progress along
    // the current path, new map data can make a different frontier better.
    if (mission_should_explore() &&
        active_goal.type == NAV_GOAL_FRONTIER &&
        active_path.length > 0 &&
        !pose_at_goal(pose) &&
        path_half_complete(robot_cell))
    {
        active_goal = {NAV_GOAL_NONE, {0, 0}};
        active_path.length = 0;
        reset_replan_budget();
    }

    // Exploration advances by repeatedly selecting the best frontier once the previous frontier goal has been reached.
    if (mission_should_explore() && active_goal.type == NAV_GOAL_NONE)
    {
        if (!coverage_planner_started())
        {
            if (!choose_frontier(robot_cell, pose.theta))
            {
                if (status == NAV_STATUS_EXPLORATION_COMPLETE)
                {
                    start_coverage();
                }
                else
                {
                    return;
                }
            }
        }

        if (coverage_planner_started() && active_goal.type == NAV_GOAL_NONE &&
            !choose_coverage_goal())
        {
            return;
        }
    }
    // Return home mission overrides frontier selection and targets the save base cell.
    else if (mission_should_return_home())
    {
        if (!base_is_set)
        {
            status = NAV_STATUS_PATH_FAILED;
            return;
        }
        if (active_goal.type != NAV_GOAL_BASE ||
            !cells_equal(active_goal.cell, base_cell))
        {
            navigation_set_goal({NAV_GOAL_BASE, base_cell});
        }
    }

    // Replan if path becomes blocked
    if (active_goal.type != NAV_GOAL_NONE &&
        active_path.length > 0 &&
        path_blocked_ahead(robot_cell))
    {
        replan_requested = true;
    }

    if (active_goal.type != NAV_GOAL_NONE &&
        (active_path.length == 0 || replan_requested))
    {
        const bool replan_from_existing_path =
            replan_requested && active_path.length > 0;
        if (replan_from_existing_path && active_goal_can_be_rejected())
        {
            if (replans_for_active_goal < kReplansBeforeGoalReject)
            {
                ++replans_for_active_goal;
            }

            if (replans_for_active_goal >= kReplansBeforeGoalReject)
            {
                navigation_reject_current_goal();
                return;
            }
        }

        if (!plan_from(robot_cell) && mission_should_explore())
        {
            navigation_reject_current_goal();
        }
    }
}

void navigation_set_goal(navigation_goal_t goal)
{
    // Changing goal invalidates the current paths, so should be re-planned.
    if (goal.type == NAV_GOAL_NONE)
    {
        navigation_clear_goal();
        return;
    }
    if (active_goal.type != goal.type ||
        !cells_equal(active_goal.cell, goal.cell))
    {
        active_goal = goal;
        active_path.length = 0;
        replan_requested = true;
        reset_replan_budget();
    }
}

void navigation_clear_goal()
{
    // Reset navigation to an idle state with no target or path.
    active_goal = {NAV_GOAL_NONE, {0, 0}};
    active_path.length = 0;
    replan_requested = false;
    reset_replan_budget();
    reset_no_frontier_scan();
    status = NAV_STATUS_IDLE;
}


// Getter and helper functions.
bool navigation_has_goal() { return active_goal.type != NAV_GOAL_NONE; }

navigation_goal_t navigation_get_goal() { return active_goal; }

bool navigation_has_path() { return active_path.length > 0; }

const path_t* navigation_get_path()
{
    return navigation_has_path() ? &active_path : nullptr;
}

bool navigation_goal_reached() { return status == NAV_STATUS_GOAL_REACHED; }

bool navigation_exploration_complete()
{
    return status == NAV_STATUS_EXPLORATION_COMPLETE;
}

bool navigation_path_failed() { return status == NAV_STATUS_PATH_FAILED; }

navigation_status_t navigation_get_status() { return status; }


void navigation_request_replan()
{
    if (active_goal.type != NAV_GOAL_NONE)
    {
        replan_requested = true;
    }
}

void navigation_reject_current_frontier()
{
    if (active_goal.type != NAV_GOAL_FRONTIER)
    {
        return;
    }

    rejected_frontiers[next_rejected_frontier] = {
        active_goal.cell.x,
        active_goal.cell.y,
        0
    };
    next_rejected_frontier =
        static_cast<uint8_t>((next_rejected_frontier + 1) % kRejectedFrontierCount);
    if (rejected_frontier_count < kRejectedFrontierCount)
    {
        ++rejected_frontier_count;
    }

    active_goal = {NAV_GOAL_NONE, {0, 0}};
    active_path.length = 0;
    replan_requested = false;
    reset_replan_budget();
    status = NAV_STATUS_IDLE;
}

void navigation_reject_current_goal()
{
    if (active_goal.type == NAV_GOAL_FRONTIER)
    {
        navigation_reject_current_frontier();
        return;
    }

    if (active_goal.type == NAV_GOAL_COVERAGE)
    {
        coverage_planner_reject_goal();
        active_goal = {NAV_GOAL_NONE, {0, 0}};
        active_path.length = 0;
        replan_requested = false;
        reset_replan_budget();
        status = NAV_STATUS_IDLE;
    }
}

void navigation_set_base(float world_x, float world_y)
{
    base_is_set = world_to_map(world_x, world_y, &base_cell.x, &base_cell.y);
}
