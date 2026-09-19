//
// Created by Jasper Cusiel on 20/07/2026.
//

#include "display.h"

#include "ToF-Sensors.h"
#include "lidar-config.h"
#include "vfh.h"
#include "odometry.h"
#include "weight-detection.h"
#include "button.h"
#include "sensors.h"
#include "occupancy-grid.h"
#include "mapping.h"
#include "vfh.h"
#include "weight-pickup.h"
#include "ir-reflective.h"
#include "lift-motor.h"

#include <Arduino.h>
#include <Encoder.h>

#include "mission.h"
#include "navigation.h"
#include "coverage-planner.h"
#include "imu.h"
#include "U8g2lib.h"

// Encoder used to scroll through the display menu
#define ENCODER_A A11
#define ENCODER_B A1
#define SW_PIN A7

// Location of VFH histogram
#define HISTOGRAM_X 64
#define HISTOGRAM_Y 63

// Total number of menu items to show
#define MENU_ITEMS_COUNT 7

// Max number of lines to shows in scrolling list for sensor booting proccess
#define MAX_LINES 7

Encoder input_encoder(ENCODER_A, ENCODER_B);

// Used to keep track of boot status
bool boot_okay = true;

// Which sensor failed to start (if any)
const char* failed_sensor = nullptr;

// Pages that should be selectable via the menu.
const char* menuItems[] = {
    "VFH",
    "Debug",
    "Odometry",
    "8x8 ToF",
    "Map",
    "Mission"
    "Pickup State"
};

// Keep track of which page we are on
Page currentPage = PAGE_DEBUG;
int menuIndex = 0;

// Used to show scrolling list of sensor boot statuses.
static char lines[MAX_LINES][17];
static uint8_t lineCount = 0;

// Display interface, R2 = 180 degree display rotation
static U8G2_SSD1306_128X64_NONAME_F_2ND_HW_I2C display(U8G2_R2, U8X8_PIN_NONE);


void update_input()
{
    // Function reads encoder input to scroll menu and enter and exit pages.

    int32_t delta = -(input_encoder.readAndReset() / 2);
    // -ve inverts scroll direction to match display orientation.
    // Two pulses per detent -> map to scrolling one menu item index.
    // Only keep track of delta rather than absolute position (readAndReset).

    if (currentPage == PAGE_MENU)
    {
        if (delta != 0)
        {
            menuIndex += delta;
            // Stop at top and bottom of menu item list.
            if (menuIndex < 0)
                menuIndex = 0;

            if (menuIndex >= MENU_ITEMS_COUNT)
                menuIndex = MENU_ITEMS_COUNT - 1;
        }

        // Select page when rotary encode is pushed
        if (read_button(SW_PIN))
        {
            currentPage = (Page)(menuIndex + 1);
        }
    }
    else
    {
        // Return to menu if we are in a page
        if (read_button(SW_PIN))
        {
            currentPage = PAGE_MENU;
        }
    }
}

void draw_menu()
{
    // Draw the menu page
    for (int i = 0; i < MENU_ITEMS_COUNT; i++)
    {
        int y = (i + 1) * 12;

        // Invert the colors of the highlighted menu item (controlled by rotary encoder)
        if (i == menuIndex)
        {
            display.drawBox(0, y - 10, 128, 12);
            display.setDrawColor(0);
            display.drawStr(2, y, menuItems[i]);
            display.setDrawColor(1);
        }
        else
        {
            display.drawStr(2, y, menuItems[i]);
        }
    }
}

void display_init()
{
    // Starts display and input encoder.
    display.begin();
    display_log("BOOT OK");
    pinMode(SW_PIN, INPUT_PULLUP);
}

void display_draw()
{
    // Called periodically to draw the selected page/menu
    display.firstPage();

    do
    {
        display.setFont(u8g2_font_6x12_tr);
        switch (currentPage)
        {
        case PAGE_MENU: draw_menu();
            break;
        case PAGE_VFH: draw_vfh(vfh_get_histogram());
            break;
        case PAGE_ODOM: draw_odometry();
            break;
        case PAGE_DEBUG: draw_debug();
            break;
        case PAGE_8X8_TOF: draw_depth_data(display);
            break;
        case PAGE_MAP: draw_map();
            break;
        case PAGE_MISSION: draw_mission();
            break;
        case PAGE_BOOT_STATUS: draw_boot_status();
            break;
        case PAGE_WEIGHT_PICKUP: draw_weight_pickup();
            break;
        }
    }
    while (display.nextPage());
}

void display_log(const char* msg)
{
    // Logs and message to the scrolling boot screen
    if (lineCount >= MAX_LINES)
    {
        for (int i = 0; i < MAX_LINES - 1; i++)
        {
            strcpy(lines[i], lines[i + 1]);
        }
        strncpy(lines[MAX_LINES - 1], msg, 16);
        lines[MAX_LINES - 1][16] = '\0';
    }
    else
    {
        strncpy(lines[lineCount], msg, 16);
        lines[lineCount][16] = '\0';
        lineCount++;
    }
    display_draw();
}

void display_log_status(const char* name, bool ok)
{
    // Logs the boot status to the scrolling boot screen.
    char buffer[17];
    snprintf(buffer, sizeof(buffer), "%-10s %s", name, ok ? "OK" : "FAIL");
    display_log(buffer);
    display_draw();
}

void draw_debug()
{
    // Draws the debug page (most recent logs)
    for (uint8_t i = 0; i < lineCount; i++)
    {
        display.drawStr(0, 10 * i, lines[i]);
    }

    char buf[20];
    snprintf(buf, sizeof(buf), "L1:%ld", lifter_get_position1());
    display.drawStr(70, 44, buf);
    snprintf(buf, sizeof(buf), "L2:%ld", lifter_get_position2());
    display.drawStr(70, 56, buf);
}

int angle_to_u8g2(float angle)
{
    // u8g2 uses 0-255 to map 0-360 degree. This converts the angle in radians to 0-256.
    int a = 64 - (angle * 256.0f / (2.0f * PI));

    // wrap into 0-255
    while (a < 0) a += 256;
    while (a >= 256) a -= 256;

    return a;
}

void draw_angle_arrow(float angle, uint8_t radius)
{
    // Function draws arrow at specified angle and at radius from origin. Used to show target and steering direction on VFH page.
    if (!isfinite(angle))
    {
        return;
    }

    int cx = HISTOGRAM_X;
    int cy = HISTOGRAM_Y;


    int x0 = cx + (int)(sinf(angle) * (float)radius);
    int y0 = cy - (int)(cosf(angle) * (float)radius);


    uint8_t arrow_length = 8;

    int x1 = cx + (int)(sinf(angle) * (float)(radius + arrow_length));
    int y1 = cy - (int)(cosf(angle) * (float)(radius + arrow_length));


    display.drawLine(x0, y0, x1, y1);


    float head_angle = 0.5f;
    uint8_t head_length = 4;

    int xa = x1 - (int)(sinf(angle + head_angle) * (float)head_length);
    int ya = y1 + (int)(cosf(angle + head_angle) * (float)head_length);

    int xb = x1 - (int)(sinf(angle - head_angle) * (float)head_length);
    int yb = y1 + (int)(cosf(angle - head_angle) * (float)head_length);

    display.drawLine(x1, y1, xa, ya);
    display.drawLine(x1, y1, xb, yb);
}

void draw_robot_arrow(int cx, int cy, float angle, uint8_t length)
{
    // Draws arrow to represent robot pose at specified position
    int x1 = cx - (int)sinf(angle) * length;
    int y1 = cy - (int)cosf(angle) * length;

    // Draw main arrow shaft
    display.drawLine(cx, cy, x1, y1);

    // Arrow head
    float head_angle = 0.5f;
    uint8_t head_length = 4;

    int xa = x1 + (int)sinf(angle + head_angle) * head_length;
    int ya = y1 + (int)cosf(angle + head_angle) * head_length;

    int xb = x1 + (int)sinf(angle - head_angle) * head_length;
    int yb = y1 + (int)cosf(angle - head_angle) * head_length;

    display.drawLine(x1, y1, xa, ya);
    display.drawLine(x1, y1, xb, yb);
}

void draw_vfh(const float* histogram)
{
    // Function draws the VFH histogram output. Each sector is drawn as an arc.
    constexpr float histogram_display_scale = 1.0f / THRESHOLD;

    for (int i = 0; i < NUM_SECTORS; i++)
    {
        uint8_t max_radius = 40;
        uint8_t min_radius = 10;
        float angle_start = (float)(FOV_MAX - i * SECTOR_WIDTH);
        float angle_end = (float)(angle_start - SECTOR_WIDTH);

        uint8_t start = angle_to_u8g2(angle_start);
        uint8_t end = angle_to_u8g2(angle_end);


        float h = histogram[i] * histogram_display_scale;
        if (!isfinite(h))
        {
            h = 0.0f;
        }
        h = constrain(h, 0.0f, 1.0f);

        uint8_t radius = min_radius + (h * (max_radius - min_radius));

        display.drawArc(HISTOGRAM_X, HISTOGRAM_Y, radius, start, end);
    }

    display.drawArc(HISTOGRAM_X, HISTOGRAM_Y, 45, angle_to_u8g2(radians(60.0f)), angle_to_u8g2(radians(-60.0f)));
    draw_angle_arrow(-vfh_get_target_angle(), 45);
    draw_angle_arrow(-vfh_get_steering_angle(), 45 + 8);
}

void draw_odometry()
{
    // Function draws the odometry data page

    // Title
    display.drawStr(30, 0, "Odometry");

    // Get data to draw
    float x, y, theta;
    get_ekf_pose(&x, &y, &theta);

    float gyro_z, heading, vx, vy;
    get_sensor_data(&gyro_z, &heading, &vx, &vy);


    char buf[20];
    // Draw X Y Theta
    snprintf(buf, sizeof(buf), "X:%5.2f", x);
    display.drawStr(0, 12, buf);

    snprintf(buf, sizeof(buf), "Y:%5.2f", y);
    display.drawStr(0, 24, buf);

    snprintf(buf, sizeof(buf), "Theta:%5.2f", degrees(theta));
    display.drawStr(0, 36, buf);


    // Raw sensor data
    snprintf(buf, sizeof(buf), "Gyro Z:%5.2f", gyro_z);
    display.drawStr(0, 48, buf);

    snprintf(buf, sizeof(buf), "HDG:%5.2f", degrees(heading));
    display.drawStr(64, 12, buf);

    snprintf(buf, sizeof(buf), "vX:%5.2f", vx);
    display.drawStr(0, 60, buf);

    snprintf(buf, sizeof(buf), "vY:%5.2f", vy);
    display.drawStr(64, 60, buf);
}

void draw_boot_status()
{
    // Function displays the boot status after starting sensors.
    if (sensors_boot_okay())
    {
        display.drawStr(35, 10, "BOOT OKAY");
        display.drawStr(28, 30, "PUSH GO BTN");
        display.drawStr(38, 40, "TO START");
        // Draw inital heading
        char buf[20];
        snprintf(buf, sizeof(buf), "HD:%5.2f", degrees(imu_get_heading()));
        display.drawStr(38, 50, buf);
    }
    else
    {
        display.drawStr(35, 10, "BOOT FAIL");

        display.drawStr(10, 30, "SENSOR:");
        display.drawStr(55, 30, sensors_failed_sensor());
        display.drawStr(10, 40, "FAILED TO START");
    }
}

void display_set_page(const Page new_page)
{
    // Used to trigger page change by other modules.
    currentPage = new_page;
}

void draw_map()
{
    // Function draws the map to the display with the robots pose as an arrow.

    const int offset_x = 10; // centre 30px wide map
    const int offset_y = 7; // centre 50px tall map

    // Draw bounding box around map
    display.drawFrame(offset_x - 1, offset_y - 1, MAP_WIDTH, MAP_HEIGHT);
    // Draw robot pose
    pose_t current_pose;
    get_ekf_pose(&current_pose.x, &current_pose.y, &current_pose.theta);

    int rx, ry;
    if (!world_to_map(current_pose.x, current_pose.y, &rx, &ry))
    {
        return;
    }
    draw_robot_arrow(offset_x + rx, offset_y + (MAP_HEIGHT - 1 - ry), current_pose.theta, 10);

    // Draw occupancy grid
    for (int x = 0; x < MAP_WIDTH; x++)
    {
        for (int y = 0; y < MAP_HEIGHT; y++)
        {
            if (map_get_state(x, y) == OCCUPIED)
            {
                display.drawPixel(offset_x + x, offset_y + (MAP_HEIGHT - 1 - y)); // Flip Y axis
            }
        }
    }
}


void draw_mission()
{
    mission_state_t state = mission_get_state();
    display.drawStr(2, 10, "STATE: ");
    switch (state)
    {
    case MISSION_IDLE:
        display.drawStr(42, 10, "IDLE");
        break;
    case MISSION_EXPLORE:
        display.drawStr(42, 10, "EXPLORE");
        break;
    case MISSION_COMPLETE:
        display.drawStr(42, 10, "COMPLETE");
        break;
    case MISSION_RETURN_HOME:
        display.drawStr(42, 10, "RETURN HOME");
        break;
    case MISSION_STOPPED:
        display.drawStr(42, 10, "STOPPED");
        break;
    case MISSION_WEIGHT_DETECTED:
        display.drawStr(42, 10, "WEIGHT_DETECTED");
        break;
    default:
        break;
    }

    display.drawStr(2, 22, "NAV: ");
    switch (navigation_get_status())
    {
    case NAV_STATUS_IDLE:
        display.drawStr(32, 22, "IDLE/SCAN");
        break;
    case NAV_STATUS_PLANNING:
        display.drawStr(32, 22, "PLANNING");
        break;
    case NAV_STATUS_FOLLOWING_PATH:
        display.drawStr(32, 22, "FOLLOWING");
        break;
    case NAV_STATUS_GOAL_REACHED:
        display.drawStr(32, 22, "GOAL REACHED");
        break;
    case NAV_STATUS_EXPLORATION_COMPLETE:
        display.drawStr(32, 22, "EXP COMPLETE");
        break;
    case NAV_STATUS_PATH_FAILED:
        display.drawStr(32, 22, "PATH FAILED");
        break;
    default:
        break;
    }

    const navigation_goal_t goal = navigation_get_goal();
    display.drawStr(2, 34, "GOAL: ");
    switch (goal.type)
    {
    case NAV_GOAL_NONE:
        display.drawStr(38, 34, "NONE");
        break;
    case NAV_GOAL_FRONTIER:
        display.drawStr(38, 34, "FRONTIER");
        break;
    case NAV_GOAL_COVERAGE:
        display.drawStr(38, 34, "COVERAGE");
        break;
    case NAV_GOAL_BASE:
        display.drawStr(38, 34, "BASE");
        break;
    default:
        break;
    }

    char buf[22];
    snprintf(buf, sizeof(buf), "CELL:%02d,%02d", goal.cell.x, goal.cell.y);
    display.drawStr(2, 46, buf);
    snprintf(buf, sizeof(buf), "COV:%u/%u",
             coverage_planner_goal_index(),
             coverage_planner_goal_count());
    display.drawStr(2, 58, buf);
}


void draw_weight_pickup()
{
    // Draws the weight pickup state machine's current state
    display.drawStr(2, 10, "STATE:");

    switch (weight_pickup_get_state())
    {
    case PICKUP_STATUS_IDLE:
        display.drawStr(50, 10, "IDLE");
        break;
    case PICKUP_STATUS_ALIGNING:
        display.drawStr(50, 10, "ALIGNING");
        char buf[20];
        snprintf(buf, sizeof(buf), "A13:%d", analogRead(A13));
        display.drawStr(2, 24, buf);
        display.drawStr(2, 36, is_weight_detected_ir_reflective() ? "IR: DETECTED" : "IR: ---");
        break;
    case PICKUP_STATUS_WEIGHT_LOST:
        display.drawStr(50, 10, "FINDING");
        break;
    case PICKUP_STATUS_CHECKING_WEIGHT_TYPE:
        display.drawStr(50, 10, "CHECK TYPE");
        break;
    case PICKUP_STATUS_FAKE_WEIGHT_DETECTED:
        display.drawStr(50, 10, "FAKE DETECTED");
        break;
    case PICKUP_STATUS_FAKE_WEIGHT_LIFTING:
        display.drawStr(50, 10, "FAKE SERVO UP");
        break;
    case PICKUP_STATUS_FAKE_WEIGHT_MOVING:
        display.drawStr(50, 10, "FAKE REJECTING");
        break;
    case PICKUP_STATUS_FAKE_WEIGHT_CLEAR_DELAY:
        display.drawStr(50, 10, "FAKE REJECTING");
        break;
    case PICKUP_STATUS_LOWERING_RAILS:
        display.drawStr(50, 10, "LOWERING");
        break;
    case PICKUP_STATUS_LOADING_WEIGHT:
        display.drawStr(50, 10, "LOADING");
        break;
    case PICKUP_STATUS_LOADING_WEIGHT_CONFIRM_DELAY:
        display.drawStr(50, 10, "LOADING");
        break;
    case PICKUP_STATUS_LIFTING_WEIGHT:
        display.drawStr(50, 10, "LIFTING");
        break;
    default:
        display.drawStr(50, 10, "UNKNOWN");
        break;
    }
}