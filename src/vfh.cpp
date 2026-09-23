//
// Created by Jasper Cusiel on 25/07/2026.
//

#include <ToF-Sensors.h>
#include <math.h>
#include <stdint.h>
#include <vfh.h>


VFH vfh;

namespace
{
    constexpr float kHistogramCostWeight = 0.5f;
    constexpr float kSteeringChangeCostWeight = 0.75f;

    float wrap_angle(float angle)
    {
        while (angle > PI) angle -= 2.0f * PI;
        while (angle < -PI) angle += 2.0f * PI;
        return angle;
    }

    float clamp_angle_to_fov(float angle)
    {
        if (angle < FOV_MIN) return FOV_MIN;
        if (angle > FOV_MAX) return FOV_MAX;
        return angle;
    }

    float angular_distance(float a, float b)
    {
        return fabsf(wrap_angle(a - b));
    }
}

void vfh_init()
{
    for (int i = 0; i < NUM_SECTORS; i++)
    {
        vfh.histogram[i] = 0.0f;
        vfh.free_directions[i] = true;
        vfh.sector_angles[i] = FOV_MIN + (i + 0.5f) * SECTOR_WIDTH;
    }
    vfh.target_angle = 0.0f;
    vfh.steering_angle = NAN;
    vfh.forward_clearance = MAX_RANGE;
}

void add_histogram_value(float vfh_histogram[NUM_SECTORS], int sector,
                         float weight, float range)
{
    int spread = (int)ceilf(
        atan2f(ROBOT_CLEARANCE * VFH_INFLATION_SCALE, range) / SECTOR_WIDTH);
    for (int i = -spread; i <= spread; i++)
    {
        int s = sector + i;

        if (s >= 0 && s < NUM_SECTORS)
        {
            vfh_histogram[s] += weight;
        }
    }
}

void build_histogram()
{
    lidar_scan* new_lidar_scan = get_scan();
    vfh.forward_clearance = MAX_RANGE;
    for (size_t i = 0; i < NUM_SECTORS; i++)
    {
        vfh.histogram[i] = 0.0f;
    }

    for (size_t i = 0; i < NumOfTOFSensors * NumOfZonesPerSensor; i++)
    {
        float r = new_lidar_scan->ranges[i];

        if (r <= 0.01f || r > MAX_RANGE)
        {
            continue;
        }
        if (fabsf(new_lidar_scan->angles[i]) <= FRONT_CLEARANCE_CONE &&
            r < vfh.forward_clearance)
        {
            vfh.forward_clearance = r;
        }
        // closer obstacles = higher density
        float x = (MAX_RANGE - r) / MAX_RANGE;
        float weight = x * x;

        uint8_t sector = new_lidar_scan->sector_index[i];

        if (sector < NUM_SECTORS)
        {
            add_histogram_value(vfh.histogram, sector, weight, r);
        }
    }
}

void threshold_histogram()
{
    for (int i = 0; i < NUM_SECTORS; i++)
    {
        if (vfh.free_directions[i])
        {
            vfh.free_directions[i] = (vfh.histogram[i] < VFH_BLOCKED_THRESHOLD);
        }
        else
        {
            vfh.free_directions[i] = (vfh.histogram[i] < VFH_FREE_THRESHOLD);
        }
    }
}

float vfh_get_best_direction(float target_angle)
{
    float best_angle = NAN;
    float best_cost = 1e9;
    const float target = clamp_angle_to_fov(target_angle);
    const bool have_previous = isfinite(vfh.steering_angle);

    for (size_t i = 0; i < NUM_SECTORS; i++)
    {
        if (!vfh.free_directions[i])
        {
            continue;
        }
        // sector to angle
        float angle = vfh.sector_angles[i];

        float diff = angular_distance(angle, target);
        float steering_change =
            have_previous ? angular_distance(angle, vfh.steering_angle) : 0.0f;
        float cost = diff +
            vfh.histogram[i] * kHistogramCostWeight +
            steering_change * kSteeringChangeCostWeight;

        if (cost < best_cost)
        {
            best_cost = cost;
            best_angle = angle;
        }
    }

    return best_angle;
}

void compute_vfh()
{
    build_histogram();
    threshold_histogram();
    vfh.steering_angle = vfh_get_best_direction(vfh.target_angle);
}

float* vfh_get_histogram()
{
    return vfh.histogram;
}

void vfh_set_target_angle(const float target_angle)
{
    vfh.target_angle = target_angle;
}

float vfh_get_target_angle()
{
    return vfh.target_angle;
}

float vfh_get_steering_angle()
{
    return vfh.steering_angle;
}

float vfh_get_forward_clearance()
{
    return vfh.forward_clearance;
}
