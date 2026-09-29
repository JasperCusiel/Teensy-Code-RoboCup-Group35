//
// Created by Jasper Cusiel on 03/08/2026.
//
#include "weight-detection.h"
#include "DFRobot_MatrixLidar.h"
#include "ToF-Sensors.h"
#include "button.h"
#include "mission.h"
#include <math.h>


#define TOF_ARRAY_ADDRESS 0x33
#define WEIGHT_MIN_DELTA 25
#define WEIGHT_MAX_DELTA 150
#define WEIGHT_MAX_ACTIVE_CELLS 32
#define WALL_FRONT_CONE_RAD (35.0f * Pi / 180.0f)
#define WALL_MIN_RANGE_M 0.05f
#define WALL_MAX_RANGE_M 0.55f
#define WALL_MAX_RANGE_SPREAD_M 0.12f
#define WALL_MIN_ADJACENT_POINTS 4
#define WEIGHT_DETECTED_CYCLES 5
#define TOF_SAMPLE_INTERVAL_MS 20u
#define TOF_CALIBRATION_SAMPLE_DELAY_MS 50u

DFRobot_MatrixLidar_I2C tof(TOF_ARRAY_ADDRESS, &Wire1);
uint16_t buf[64];
uint32_t calibration[64];
bool active[64];
bool weight_detected = false;
bool wall_detected = false;

enum WeightType
{
    WEIGHT_NONE,
    WEIGHT_2X2,
    WEIGHT_2X3
};

WeightType weight_type = WEIGHT_NONE;


static bool tof_array_sees_wall();


bool weight_detection_init()
{
    // Check sensor starts and set to 8x8 mode
    if (tof.begin() == 0 && tof.setRangingMode(eMatrix_8X8) == 0)
    {
        fill_calibration_matrix();
        return true;
    }
    return false;
}

void weight_detection_task()
{
    static uint32_t last_sample_ms = 0;
    static uint8_t sample_count = 0;
    static uint32_t sum[64] = {0};
    static uint16_t temp[64];
    static bool sampling_active = false;

    if (!sampling_active)
    {
        for (int i = 0; i < 64; i++)
        {
            sum[i] = 0;
        }
        sample_count = 0;
        last_sample_ms = millis();
        sampling_active = true;
    }

    if (millis() - last_sample_ms < TOF_SAMPLE_INTERVAL_MS)
    {
        return;
    }

    last_sample_ms = millis();
    tof.getAllData(temp);

    for (int i = 0; i < 64; i++)
    {
        sum[i] += temp[i];
    }
    sample_count++;

    if (sample_count < 8)
    {
        return;
    }

    sampling_active = false;

    for (int i = 0; i < 64; i++)
    {
        buf[i] = sum[i] / 8;
    }
    int active_count = 0;
    for (int i = 0; i < 64; i++)
    {
        int16_t d = (int16_t)calibration[i] - (int16_t)buf[i];

        active[i] = (d > WEIGHT_MIN_DELTA && d < WEIGHT_MAX_DELTA);
        if (active[i])
        {
            active_count++;
        }
    }

    if (active_count > WEIGHT_MAX_ACTIVE_CELLS)
    {
        for (int i = 0; i < 64; i++)
        {
            active[i] = false;
        }
    }
    filter();
    wall_detected = tof_array_sees_wall();
    weight_detected = !wall_detected && detect_weight();

    if (weight_detected)
    {
        mission_report_weight_detected();
    }
} 

void drawToF_dithered_fast(U8G2& u8g2,
                           uint16_t d_max,
                           int x0, int y0)
{
    (void)d_max;

    const int cell_size = 8;

    if (weight_detected)
    {
        switch (weight_type){
            case WEIGHT_2X2:
            u8g2.drawStr(70, 10, "2X2");
            break;

            case WEIGHT_2X3:
            u8g2.drawStr(70, 10, "2X3");
            break;

            default:
            u8g2.drawStr(70, 10, "WEIGHT");
            break;
        }
    }
    else if (wall_detected)
    {
        u8g2.drawStr(70, 10, "WALL");
    }


    for (int cy = 0; cy < 8; cy++)
    {
        for (int cx = 0; cx < 8; cx++)
        {
            int index = cy * 8 + cx;

            int base_x = x0 + cx * cell_size;
            int base_y = y0 + cy * cell_size;

            if (active[index])
            {
                u8g2.drawBox(base_x, base_y, cell_size, cell_size);
            }
        }
    }
}


void filter() //filter out random pixels
{
    bool filtered[64] = {false};

    for (int cy = 0; cy < 8; cy++)
    {
        for (int cx = 0; cx < 8; cx++)
        {
            int index = cy * 8 + cx;

            if (!active[index])
            {
                continue;
            }

            int neighbours = 0;
            if (cx > 0 && active[index - 1])
            {
                neighbours++;
            }
            if (cx < 7 && active[index + 1])
            {
                neighbours++;
            }
            if (cy > 0 && active[index - 8])
            {
                neighbours++;
            }

            if (cy < 7 && active[index + 8])
            {
                neighbours++;
            }

            if (neighbours >= 2)
            {
                filtered[index] = true;
            }
        }
    }

    // Copy filtered result back
    for (int i = 0; i < 64; i++)
    {
        active[i] = filtered[i];
    }
}

static bool tof_array_sees_wall()
{
    lidar_scan* scan = get_scan();
    int adjacent_points = 0;
    float min_range = WALL_MAX_RANGE_M;
    float max_range = 0.0f;

    for (int i = 0; i < NUM_POINTS; i++)
    {
        const float range = scan->ranges[i];
        const bool close_front_return =
            fabsf(scan->angles[i]) <= WALL_FRONT_CONE_RAD &&
            range >= WALL_MIN_RANGE_M &&
            range <= WALL_MAX_RANGE_M;

        if (!close_front_return)
        {
            adjacent_points = 0;
            min_range = WALL_MAX_RANGE_M;
            max_range = 0.0f;
            continue;
        }

        adjacent_points++;
        if (range < min_range)
        {
            min_range = range;
        }
        if (range > max_range)
        {
            max_range = range;
        }

        if (adjacent_points >= WALL_MIN_ADJACENT_POINTS &&
            max_range - min_range <= WALL_MAX_RANGE_SPREAD_M)
        {
            return true;
        }
    }

    return false;
}

static bool is_active_cell(int x, int y)
{
    if (x < 0 || x >= 8 || y < 0 || y >= 8)
    {
        return false;
    }

    return active[y * 8 + x];
}


static bool is_exact_2x2(int x, int y)
{
    if (x < 0 || y < 0 || x + 1 >= 8 || y + 1 >= 8)
    {
        return false;
    }

    // Require the exact 2x2 block
    if (!is_active_cell(x, y) || !is_active_cell(x + 1, y) ||
        !is_active_cell(x, y + 1) || !is_active_cell(x + 1, y + 1))
    {
        return false;
    }

    // Block larger patterns like 2x3, 3x2, 2x4 etc.
    if (is_active_cell(x - 1, y) || is_active_cell(x + 2, y) ||
        is_active_cell(x - 1, y + 1) || 
        is_active_cell(x, y - 1) || is_active_cell(x + 1, y - 1) ||
        is_active_cell(x, y + 2))
    {
        return false;
    }

    return true;
}

static bool is_exact_2x3(int x, int y)
{
    if (x < 0 || y < 0 || x + 1 >= 8 || y + 2 >= 8)
    {
        return false;
    }

    // Require the exact 2x3 block
    if (!is_active_cell(x, y) || !is_active_cell(x + 1, y) ||
        !is_active_cell(x, y + 1) || !is_active_cell(x + 1, y + 1) ||
        !is_active_cell(x, y + 2) || !is_active_cell(x + 1, y + 2))
    {
        return false;
    }

    // Reject larger patterns like 2x4, 3x2, 3x3
    if (is_active_cell(x - 1, y) || is_active_cell(x + 2, y) ||
        is_active_cell(x - 1, y + 1) || is_active_cell(x + 2, y + 1) ||
        is_active_cell(x - 1, y + 2) || is_active_cell(x + 2, y + 2) || 
        is_active_cell(x, y + 3) || is_active_cell(x + 1, y + 3))
    {
        return false;
    }

    return true;
}

bool detect_weight()
{
    for (int y = 1; y < 7; y++)
    {
        for (int x = 1; x < 7; x++)

        {
            if (is_exact_2x2(x, y))
            {
                weight_type = WEIGHT_2X2;
                return true;
            }

            if (is_exact_2x3(x, y))
            {
                weight_type = WEIGHT_2X3;
                return true;
            }
        }
    }

    weight_type = WEIGHT_NONE;
    return false;
}
  

void draw_depth_data(U8G2& u8g2)
{
    // tof.getAllData(buf);

    drawToF_dithered_fast(u8g2, 200, 0, 0);
    if (read_button(A9) == LOW)
    {
        u8g2.drawStr(70, 10, "calibrating...");
        fill_calibration_matrix();
    }
}


void fill_calibration_matrix()
{
    // Reset calibration data
    for (size_t j = 0; j < 64; j++)
    {
        calibration[j] = 0;
    }

    uint32_t last_sample_ms = millis();
    for (size_t i = 0; i < 10; i++)
    {
        while (millis() - last_sample_ms < TOF_CALIBRATION_SAMPLE_DELAY_MS)
        {
            yield();
        }
        last_sample_ms = millis();

        tof.getAllData(buf);
        // Add data to calibration buffer
        for (size_t j = 0; j < 64; j++)
        {
            calibration[j] += buf[j];
        }
    }
    // Average data
    for (size_t j = 0; j < 64; j++)
    {
        calibration[j] = calibration[j] / 10;
    }
}



