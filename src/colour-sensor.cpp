//
// Created by Jasper Cusiel on 21/07/2026.
//
#include <colour-sensor.h>
#include <Adafruit_TCS34725.h>

#define I2C_BUS &Wire1
#define I2C_ADDRESS 0x29

Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_614MS, TCS34725_GAIN_1X);

// Stores the color profile of the base
color_t base = {0, 0, 0, 0, COLOR_UNDEFINED};
color_t current = {0, 0, 0, 0, COLOR_UNDEFINED};

// Initialize sensor on I2C and set the base color (we assume the robot is always started in the colored home base square.
bool colour_sensor_init()
{
    for (int i = 0; i < 20; i++)
    {
        // 2 seconds max
        if (tcs.begin(I2C_ADDRESS, I2C_BUS) == true)
        {
            // Get base color
            update_base_color();
            return true;
        }
        delay(100);
    }

    return false;
}

static base_color_t classify_color(uint16_t red, uint16_t green, uint16_t blue, uint16_t clear)
{
    if (clear == 0)
    {
        return COLOR_UNDEFINED;
    }

    const float red_ratio = red / static_cast<float>(clear);
    const float green_ratio = green / static_cast<float>(clear);
    const float blue_ratio = blue / static_cast<float>(clear);

    // Green base tends to have a noticeably stronger green response than blue.
    // Blue base is the opposite. Use a normalized delta so the threshold is stable across lighting.
    const float green_delta = green_ratio - blue_ratio;
    const float blue_delta = blue_ratio - green_ratio;

    if (green_delta > 0.07f || (green > blue + 60 && red_ratio > 0.10f))
    {
        return COLOR_GREEN;
    }
    if (blue_delta > 0.07f || (blue > green + 60 && red_ratio > 0.10f))
    {
        return COLOR_BLUE;
    }

    return COLOR_UNDEFINED;
}

void update_base_color()
{
    // Get raw reading from colour sensor
    tcs.getRawData(&base.red, &base.green, &base.blue, &base.clear);
    base.base_color = classify_color(base.red, base.green, base.blue, base.clear);
}

base_color_t get_current_color()
{
    // Read the latest result without the integration delay.
    if ((tcs.read8(TCS34725_STATUS) &
        TCS34725_STATUS_AVALID) == 0)
    {
        current.base_color = COLOR_UNDEFINED;
        return current.base_color;
    }

    current.clear = tcs.read16(TCS34725_CDATAL);
    current.red = tcs.read16(TCS34725_RDATAL);
    current.green = tcs.read16(TCS34725_GDATAL);
    current.blue = tcs.read16(TCS34725_BDATAL);
    current.base_color = classify_color(current.red, current.green, current.blue, current.clear);

    return current.base_color;
}

base_color_t get_base_color()
{
    return base.base_color;
}
