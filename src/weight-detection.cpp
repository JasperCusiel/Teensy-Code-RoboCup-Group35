#include "weight-detection.h"
#include "DFRobot_MatrixLidar.h"
#include "button.h"
#include "mission.h"
#include <Arduino.h>
#include <math.h>
#include <string.h>

DFRobot_MatrixLidar_I2C tof(0x33, &Wire1);
uint16_t buf[64] = {};
uint32_t calibration[64] = {};
bool active[64] = {};
bool weight_detected = false;
bool wall_detected = false; // broad foreground / wall-like shape, not proven wall

enum WeightType { WEIGHT_NONE, WEIGHT_2X2, WEIGHT_2X3 };

WeightType weight_type = WEIGHT_NONE; // legacy symbol; UI now uses actual blob size

namespace
{
    constexpr uint32_t kSampleMs = 50, kStaleMs = 300, kCalibrationTimeoutMs = 5000;
    constexpr int kCalFrames = 16, kMinCalSamples = 12;
    constexpr int kMinRangeMm = 40, kMaxRangeMm = 2000;
    constexpr int kMinDeltaMm = 18, kMaxCalibrationNoiseMm = 12;
    constexpr int kMinUsableCells = 32;
    constexpr int kMaxSceneShiftMm = 20, kMaxForegroundCells = 24;
    constexpr int kMinCells = 3, kMaxCells = 20, kMaxExtent = 5;
    constexpr int kMaxDepthSpreadMm = 90, kBorderContrastMm = 10;
    constexpr int kConfirmFrames = 2, kReleaseFrames = 5;
    constexpr bool kDebug = true;

    bool ready = false, calibrated = false, calibrating = false, calFailed = false;
    bool calTimerStarted = false;
    bool usable[64] = {};
    uint16_t noise[64] = {}, samples[kCalFrames][64] = {};
    uint8_t calCount = 0, hits = 0, misses = 0;
    bool reported = false, tracking = false;
    uint32_t lastReadMs = 0, lastGoodMs = 0, calStartMs = 0, lastLogMs = 0;
    int foregroundCount = 0, sceneShift = 0;
    int validCells = 0, peakDelta = 0;
    const char* rejectReason = "none";
    uint32_t readTimeUs = 0;

    struct Blob
    {
        int count = 0, width = 0, height = 0, range = 0;
        float x = 0, y = 0;
    };

    Blob candidate, previous;

    bool valid(uint16_t v) { return v >= kMinRangeMm && v <= kMaxRangeMm; }

    int median(int* a, int n)
    {
        for (int i = 1; i < n; ++i)
        {
            int v = a[i], j = i;
            while (j > 0 && a[j - 1] > v)
            {
                a[j] = a[j - 1];
                --j;
            }
            a[j] = v;
        }
        return n ? a[n / 2] : 0;
    }

    void resetDetection(bool rearm)
    {
        memset(active, 0, sizeof(active));
        weight_detected = false;
        wall_detected = false;
        weight_type = WEIGHT_NONE;
        hits = 0;
        misses = 0;
        tracking = false;
        candidate = Blob{};
        if (rearm) reported = false;
    }

    void finishCalibration()
    {
        uint32_t nextBase[64] = {};
        uint16_t nextNoise[64] = {};
        bool nextUsable[64] = {};
        int good = 0;
        for (int i = 0; i < 64; ++i)
        {
            int a[kCalFrames], n = 0;
            for (int f = 0; f < kCalFrames; ++f)
                if (valid(samples[f][i])) a[n++] = samples[f][i];
            if (n < kMinCalSamples) continue;
            int base = median(a, n);
            for (int j = 0; j < n; ++j) a[j] = abs(a[j] - base);
            int mad = median(a, n);
            if (mad > kMaxCalibrationNoiseMm) continue;
            nextBase[i] = base;
            nextNoise[i] = mad;
            nextUsable[i] = true;
            ++good;
        }
        calibrating = false;
        // A failed recalibration leaves old values intact, but detection disabled
        // until a successful explicit calibration, rather than using a suspect view.
        calFailed = good < kMinUsableCells;
        calibrated = !calFailed;
        if (calibrated)
        {
            memcpy(calibration, nextBase, sizeof(calibration));
            memcpy(noise, nextNoise, sizeof(noise));
            memcpy(usable, nextUsable, sizeof(usable));
        }
        resetDetection(true);
        if (kDebug) Serial.printf("WEIGHT CAL: %s usable=%d/64\n", calibrated ? "OK" : "FAIL", good);
    }

    bool findCandidate()
    {
        candidate = Blob{};
        foregroundCount = 0;
        wall_detected = false;
        validCells = 0;
        peakDelta = 0;
        sceneShift = 0;
        rejectReason = "no-foreground";
        memset(active, 0, sizeof(active));
        int delta[64] = {}, shifts[64], n = 0;
        bool good[64] = {}, foreground[64] = {}, visited[64] = {};
        for (int i = 0; i < 64; ++i)
        {
            good[i] = usable[i] && valid(buf[i]);
            if (good[i])
            {
                delta[i] = int(calibration[i]) - int(buf[i]);
                shifts[n++] = delta[i];
            }
        }
        validCells = n;
        if (n < kMinUsableCells)
        {
            rejectReason = "too-few-valid";
            return false;
        }
        sceneShift = median(shifts, n);
        // Small global floor-distance shifts may be vibration; large shifts are
        // ambiguous (pitch, floor change, or a broad obstacle): fail closed.
        if (abs(sceneShift) > kMaxSceneShiftMm)
        {
            wall_detected = true;
            rejectReason = "scene-shift";
            return false;
        }
        for (int i = 0; i < 64; ++i)
        {
            delta[i] -= sceneShift;
            // Noise adds at most 15 mm: old threshold could reach 78 mm.
            const int extra = 2 * int(noise[i]);
            const int threshold = kMinDeltaMm + (extra < 15 ? extra : 15);
            if (good[i] && delta[i] > peakDelta) peakDelta = delta[i];
            foreground[i] = good[i] && delta[i] >= threshold;
            foregroundCount += foreground[i];
        }
        if (foregroundCount > kMaxForegroundCells)
        {
            wall_detected = true;
            rejectReason = "broad-foreground";
            return false;
        }
        // Eight-connected components, without first eroding their boundaries.
        // Do NOT impose a maximum delta: that can carve a wall into small blobs.
        for (int seed = 0; seed < 64; ++seed)
        {
            if (!foreground[seed] || visited[seed]) continue;
            int queue[64], head = 0, tail = 0;
            queue[tail++] = seed;
            visited[seed] = true;
            int minX = 7, maxX = 0, minY = 7, maxY = 0, minD = 65535, maxD = 0, sx = 0, sy = 0;
            int depths[64], deltas[64];
            while (head < tail)
            {
                const int i = queue[head++], x = i % 8, y = i / 8;
                minX = x < minX ? x : minX;
                maxX = x > maxX ? x : maxX;
                minY = y < minY ? y : minY;
                maxY = y > maxY ? y : maxY;
                minD = buf[i] < minD ? buf[i] : minD;
                maxD = buf[i] > maxD ? buf[i] : maxD;
                sx += x;
                sy += y;
                depths[head - 1] = buf[i];
                deltas[head - 1] = delta[i];
                for (int dy = -1; dy <= 1; ++dy)
                    for (int dx = -1; dx <= 1; ++dx)
                    {
                        int xx = x + dx, yy = y + dy;
                        if (xx < 0 || xx > 7 || yy < 0 || yy > 7) continue;
                        int j = yy * 8 + xx;
                        if (foreground[j] && !visited[j])
                        {
                            visited[j] = true;
                            queue[tail++] = j;
                        }
                    }
            }
            int w = maxX - minX + 1, h = maxY - minY + 1;
            if (w > kMaxExtent || h > kMaxExtent || tail > kMaxCells)
            {
                wall_detected = true;
                rejectReason = "large-blob";
                continue;
            }
            if (tail < kMinCells || w < 2 || h < 2 || tail * 100 < 45 * w * h)
            {
                rejectReason = "small-sparse-blob";
                continue;
            }
            if (maxD - minD > kMaxDepthSpreadMm)
            {
                rejectReason = "depth-spread";
                continue;
            }
            // Require background on any TWO sides, allowing partial edge objects. Compare
            // calibrated residuals (not raw floor ranges, which vary with row).
            const int objectDelta = median(deltas, tail);
            int clearSides = 0;
            for (int side = 0; side < 4; ++side)
            {
                int total = (side < 2 ? w : h), validN = 0, clearN = 0;
                for (int k = 0; k < total; ++k)
                {
                    int x = side < 2 ? minX + k : (side == 2 ? minX - 1 : maxX + 1);
                    int y = side < 2 ? (side == 0 ? minY - 1 : maxY + 1) : minY + k;
                    if (x < 0 || x > 7 || y < 0 || y > 7) continue;
                    int j = y * 8 + x;
                    if (!good[j]) continue;
                    ++validN;
                    if (!foreground[j] && objectDelta - delta[j] >= kBorderContrastMm) ++clearN;
                }
                if (validN * 2 >= total && clearN * 2 >= total) ++clearSides;
            }
            if (clearSides < 2)
            {
                rejectReason = "no-boundary";
                continue;
            }
            Blob b;
            b.count = tail;
            b.width = w;
            b.height = h;
            b.range = median(depths, tail);
            b.x = float(sx) / tail;
            b.y = float(sy) / tail;
            // Prefer the tracked object; otherwise prefer the largest candidate.
            bool match = tracking && fabsf(b.x - previous.x) <= 1.5f &&
                fabsf(b.y - previous.y) <= 1.5f && abs(b.range - previous.range) <= 100;
            bool oldMatch = tracking && candidate.count && fabsf(candidate.x - previous.x) <= 1.5f &&
                fabsf(candidate.y - previous.y) <= 1.5f && abs(candidate.range - previous.range) <= 100;
            if (!candidate.count || (match && !oldMatch) || (match == oldMatch && b.count > candidate.count))
            {
                candidate = b;
                memset(active, 0, sizeof(active));
                for (int k = 0; k < tail; ++k) active[queue[k]] = true;
            }
        }
        if (candidate.count) rejectReason = "candidate";
        return candidate.count != 0;
    }

    void updateConfirmation(bool found)
    {
        weight_detected = false;
        if (!found)
        {
            hits = 0;
            tracking = false;
            if (misses < kReleaseFrames) ++misses;
            if (misses >= kReleaseFrames) reported = false;
            return;
        }
        misses = 0;
        bool same = tracking && fabsf(candidate.x - previous.x) <= 1.5f &&
            fabsf(candidate.y - previous.y) <= 1.5f && abs(candidate.range - previous.range) <= 100;
        hits = same ? (hits < kConfirmFrames ? hits + 1 : hits) : 1;
        previous = candidate;
        tracking = true;
        weight_detected = hits >= kConfirmFrames;
        if (weight_detected && !reported)
        {
            reported = true;
            mission_report_weight_detected();
        }
    }
}

void fill_calibration_matrix()
{
    if (!ready || calibrating) return;
    calibrating = true;
    calibrated = false;
    calFailed = false;
    calTimerStarted = false;
    calCount = 0;
    memset(samples, 0, sizeof(samples));
    resetDetection(true);
}

bool weight_detection_init()
{
    ready = false;
    calibrated = false;
    calibrating = false;
    resetDetection(true);
    if (tof.begin() != 0 || tof.setRangingMode(eMatrix_8X8) != 0) return false;
    ready = true;
    lastReadMs = millis();
    lastGoodMs = lastReadMs;
    fill_calibration_matrix();
    return true;
}

void weight_detection_task()
{
    if (!ready) return;
    uint32_t now = millis();
    // Calibration can be requested during setup, before the scheduler starts.
    // Do not count that idle setup/GO-button time against its timeout.
    if (calibrating && !calTimerStarted)
    {
        calStartMs = now;
        calTimerStarted = true;
    }
    if (calibrating && now - calStartMs > kCalibrationTimeoutMs)
    {
        calibrating = false;
        calibrated = false;
        calFailed = true;
        resetDetection(true);
        if (kDebug)
            Serial.printf("WEIGHT CAL: timeout frames=%u/%d\n", unsigned(calCount), kCalFrames);
    }
    if (now - lastGoodMs > kStaleMs) resetDetection(false);
    if (now - lastReadMs < kSampleMs) return;
    lastReadMs = now;
    uint16_t frame[64] = {};
    uint32_t started = micros();
    const bool ok = tof.getAllData(frame) == 0;
    readTimeUs = micros() - started;
    if (!ok)
    {
        if (kDebug && millis() - lastLogMs >= 250)
        {
            lastLogMs = millis();
            Serial.println("WEIGHT reason=read-failed");
        }
        resetDetection(false); // communication failures cannot rearm the event
        return;
    }
    lastGoodMs = millis();
    memcpy(buf, frame, sizeof(buf));
    if (calibrating)
    {
        memcpy(samples[calCount++], buf, sizeof(buf));
        if (calCount == kCalFrames) finishCalibration();
        return;
    }
    if (!calibrated) return;
    // A long read/scheduling gap cannot count toward consecutive confirmation.
    if (readTimeUs > kStaleMs * 1000u)
    {
        resetDetection(false);
        return;
    }
    updateConfirmation(findCandidate());
    if (kDebug && millis() - lastLogMs >= 250)
    {
        lastLogMs = millis();
        Serial.printf(
            "WEIGHT valid=%d peak=%d reason=%s fg=%d shift=%d wallLike=%d blob=%d size=%dx%d range=%d hits=%u/%d detected=%d read_us=%lu\n",
            validCells, peakDelta, rejectReason, foregroundCount, sceneShift, wall_detected, candidate.count,
            candidate.width,
            candidate.height, candidate.range, unsigned(hits), kConfirmFrames,
            weight_detected, (unsigned long)readTimeUs);
    }
}

// Compatibility with existing header declarations; task owns confirmation.
bool detect_weight() { return weight_detected; }

// Returns true if the system needs to stop for weight detection calibration
// Continues normal tasks while wall is detected, stops when wall clears for calibration
bool weight_detection_requires_stop()
{
    // If already calibrated and no issues, no need to stop
    if (calibrated && !calFailed) return false;
    
    // If calibration is in progress, stop to allow it to complete
    if (calibrating) return true;
    
    // If calibration failed, stop to allow retry
    if (calFailed) return true;
    
    // If wall IS detected, continue normal tasks (don't stop)
    if (wall_detected) return false;
    
    // Wall is not detected and not calibrated yet, stop for calibration
    return true;
}

void filter()
{
} // No erosion: component analysis replaces the old filter.

void drawToF_dithered_fast(U8G2& u8g2, uint16_t d_max, int x0, int y0)
{
    (void)d_max;
    const char* label = calibrating
                            ? "CAL..."
                            : calFailed
                            ? "CAL FAIL"
                            : !calibrated
                            ? "NOT READY"
                            : weight_detected
                            ? "WEIGHT"
                            : candidate.count
                            ? "CANDIDATE"
                            : wall_detected
                            ? "WALL-LIKE"
                            : "CLEAR";
    u8g2.drawStr(70, 10, label);
    for (int y = 0; y < 8; ++y)
        for (int x = 0; x < 8; ++x)
            if (active[y * 8 + x]) u8g2.drawBox(x0 + x * 8, y0 + y * 8, 8, 8);
}

void draw_depth_data(U8G2& u8g2)
{
    static bool wasPressed = false;
    bool pressed = read_button(A9) == LOW;
    if (pressed && !wasPressed) fill_calibration_matrix();
    wasPressed = pressed;
    drawToF_dithered_fast(u8g2, 200, 0, 0);
}
