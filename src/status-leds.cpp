#include "status-leds.h"

#include "mission.h"

#include <Arduino.h>
#include <FastLED.h>

#include "navigation.h"
#include "coverage-planner.h"

#define STATUS_LED_DATA_PIN A10

namespace
{
    constexpr uint8_t kOdomLedCount = 32;
    constexpr uint8_t kMissionLedCount = 32;
    constexpr uint8_t kTotalLedCount = kOdomLedCount + kMissionLedCount;
    constexpr uint8_t kBrightness = 128;

    CRGB leds[kTotalLedCount];
    mission_state_t last_state = MISSION_IDLE;
    bool last_coverage_started = false;
    bool initialized = false;

    CRGB mission_color(mission_state_t state)
    {
        switch (state)
        {
        case MISSION_IDLE:
            return CRGB::White;
        case MISSION_EXPLORE:
            if (coverage_planner_started())
            {
                return CRGB::Purple;
            }
            else
            {
                return CRGB::Green;
            }

        case MISSION_WEIGHT_DETECTED:
            return CRGB::Orange;
        case MISSION_RECOVERING:
            return CRGB::Yellow;
        case MISSION_RETURN_HOME:
            return CRGB::Blue;
        case MISSION_COMPLETE:
            return CRGB::DarkBlue;
        case MISSION_STOPPED:
            return CRGB::Red;
        }

        return CRGB::Black;
    }

    void render_mission_status(mission_state_t state)
    {
        fill_solid(leds, kOdomLedCount, CRGB::White);
        fill_solid(leds + kOdomLedCount, kMissionLedCount, mission_color(state));
    }
} // namespace

void status_leds_init()
{
    FastLED.addLeds<WS2812, STATUS_LED_DATA_PIN, GRB>(leds, kTotalLedCount);
    FastLED.setBrightness(kBrightness);

    last_state = mission_get_state();
    last_coverage_started = coverage_planner_started();
    render_mission_status(last_state);
    FastLED.show();
    initialized = true;
}

void status_leds_task()
{
    if (!initialized)
    {
        return;
    }

    const mission_state_t state = mission_get_state();
    const bool coverage_started = coverage_planner_started();
    if (state == last_state && coverage_started == last_coverage_started)
    {
        return;
    }

    last_state = state;
    last_coverage_started = coverage_started;
    render_mission_status(state);
    FastLED.show();
}
