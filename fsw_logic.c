#include <stdint.h>
#include <stdbool.h>
#include <math.h>
#include <stdio.h>

#define ALT_FILTER_SIZE        5u
#define PAD_CAL_SAMPLES        20u

#define ASCENT_CONFIRM         4u
#define APOGEE_CONFIRM         4u
#define DESCENT_CONFIRM        4u
#define RELEASE_CONFIRM        4u
#define LANDED_CONFIRM         15u

#define ASCENT_MARGIN_M        5.0f
#define APOGEE_DROP_M          3.0f
#define DESCENT_DROP_M         10.0f
#define RELEASE_FRACTION       0.90f
#define RELEASE_HYST_M         2.0f
#define LANDED_SPEED_MPS       0.5f
#define LANDED_MAX_ALT_M       25.0f
#define VSPEED_FILTER_ALPHA    0.2f
#define MAX_ALT_RATE_MPS       250.0f

#define SEA_LEVEL_PRESSURE_PA  101325.0f
#define BARO_EXPONENT          0.190295f
#define BARO_EXPONENT_INV      5.25501f

typedef enum {
    STATE_LAUNCH_PAD = 0,
    STATE_ASCENT,
    STATE_APOGEE,
    STATE_DESCENT,
    STATE_PQ_RELEASE,
    STATE_LANDED
} FswStateId;

typedef struct {
    uint8_t state;
    float pad_pressure_pa;
    float maximum_height_m;
} FswNvmRecord;

typedef struct {
    FswStateId state;
    float pad_pressure_pa;
    float maximum_height_m;
    float previous_altitude_m;
    float previous_filtered_altitude_m;
    float filtered_altitude_m;
    float vertical_speed_mps;
    float altitude_window[ALT_FILTER_SIZE];
    uint8_t window_index;
    uint8_t window_count;

    float calib_sum_pa;
    uint16_t calib_count;

    uint16_t ascent_count;
    uint16_t apogee_count;
    uint16_t descent_count;
    uint16_t release_count;
    uint16_t landed_count;

    bool armed;
    bool simulation;
    bool sim_valid;
    float sim_pressure_pa;

    bool active;
    bool release_command;
    bool landed;
} FswState;

static float pressure_to_altitude_m(float pressure_pa)
{
    if (pressure_pa <= 0.0f) return 0.0f;
    return 44330.0f * (1.0f - powf(pressure_pa / SEA_LEVEL_PRESSURE_PA, BARO_EXPONENT));
}

static float relative_altitude_m(const FswState *s, float pressure_pa)
{
    return pressure_to_altitude_m(pressure_pa) - pressure_to_altitude_m(s->pad_pressure_pa);
}

static void reset_transition_counters(FswState *s)
{
    s->ascent_count = 0;
    s->apogee_count = 0;
    s->descent_count = 0;
    s->release_count = 0;
    s->landed_count = 0;
}

void FswInit(FswState *s)
{
    s->state = STATE_LAUNCH_PAD;
    s->pad_pressure_pa = SEA_LEVEL_PRESSURE_PA;
    s->maximum_height_m = 0.0f;
    s->previous_altitude_m = 0.0f;
    s->previous_filtered_altitude_m = 0.0f;
    s->filtered_altitude_m = 0.0f;
    s->vertical_speed_mps = 0.0f;
    s->window_index = 0;
    s->window_count = 0;
    for (uint8_t i = 0; i < ALT_FILTER_SIZE; i++) s->altitude_window[i] = 0.0f;
    s->calib_sum_pa = 0.0f;
    s->calib_count = 0;
    reset_transition_counters(s);
    s->armed = false;
    s->simulation = false;
    s->sim_valid = false;
    s->sim_pressure_pa = SEA_LEVEL_PRESSURE_PA;
    s->active = true;
    s->release_command = false;
    s->landed = false;
}

void FswStartPadCalibration(FswState *s)
{
    s->calib_sum_pa = 0.0f;
    s->calib_count = 0;
}

bool FswFeedPadCalibration(FswState *s, float pressure_pa)
{
    s->calib_sum_pa += pressure_pa;
    s->calib_count++;
    if (s->calib_count < PAD_CAL_SAMPLES) return false;
    s->pad_pressure_pa = s->calib_sum_pa / (float)s->calib_count;
    s->previous_altitude_m = 0.0f;
    s->previous_filtered_altitude_m = 0.0f;
    s->filtered_altitude_m = 0.0f;
    s->vertical_speed_mps = 0.0f;
    s->maximum_height_m = 0.0f;
    s->window_index = 0;
    s->window_count = 0;
    for (uint8_t i = 0; i < ALT_FILTER_SIZE; i++) s->altitude_window[i] = 0.0f;
    return true;
}

void FswSetArmed(FswState *s, bool armed)
{
    s->armed = armed;
}

void FswEnableSimulation(FswState *s, bool enable)
{
    s->simulation = enable;
    if (!enable) s->sim_valid = false;
}

void FswSetSimulatedPressure(FswState *s, float pressure_pa)
{
    s->sim_pressure_pa = pressure_pa;
    s->sim_valid = true;
}

static float filter_altitude(FswState *s, float altitude_m)
{
    s->altitude_window[s->window_index] = altitude_m;
    s->window_index = (uint8_t)((s->window_index + 1u) % ALT_FILTER_SIZE);
    if (s->window_count < ALT_FILTER_SIZE) s->window_count++;

    float sum = 0.0f;
    for (uint8_t i = 0; i < s->window_count; i++) sum += s->altitude_window[i];
    return sum / (float)s->window_count;
}

void FswUpdate(FswState *s, float pressure_pa, float dt)
{
    if (!s->active || dt <= 0.0f) return;

    float used_pressure = pressure_pa;
    if (s->simulation && s->sim_valid) used_pressure = s->sim_pressure_pa;

    float altitude_m = relative_altitude_m(s, used_pressure);
    if (!isfinite(altitude_m)) return;

    float max_step = MAX_ALT_RATE_MPS * dt;
    if (fabsf(altitude_m - s->previous_altitude_m) > max_step) {
        altitude_m = s->previous_altitude_m;
    }
    s->previous_altitude_m = altitude_m;

    float alt = filter_altitude(s, altitude_m);
    float raw_vspeed = (alt - s->previous_filtered_altitude_m) / dt;
    s->vertical_speed_mps += VSPEED_FILTER_ALPHA * (raw_vspeed - s->vertical_speed_mps);
    s->previous_filtered_altitude_m = alt;
    s->filtered_altitude_m = alt;

    if (alt > s->maximum_height_m) s->maximum_height_m = alt;

    switch (s->state) {
    case STATE_LAUNCH_PAD:
        if (s->armed && alt > ASCENT_MARGIN_M) {
            if (++s->ascent_count > ASCENT_CONFIRM) {
                s->state = STATE_ASCENT;
                s->ascent_count = 0;
            }
        } else {
            s->ascent_count = 0;
        }
        break;

    case STATE_ASCENT:
        if (alt < (s->maximum_height_m - APOGEE_DROP_M)) {
            if (++s->apogee_count > APOGEE_CONFIRM) {
                s->state = STATE_APOGEE;
                s->apogee_count = 0;
            }
        } else {
            s->apogee_count = 0;
        }
        break;

    case STATE_APOGEE:
        if (alt < (s->maximum_height_m - DESCENT_DROP_M)) {
            if (++s->descent_count > DESCENT_CONFIRM) {
                s->state = STATE_DESCENT;
                s->descent_count = 0;
            }
        } else {
            s->descent_count = 0;
        }
        break;

    case STATE_DESCENT: {
        float release_altitude_m = RELEASE_FRACTION * s->maximum_height_m;
        if (alt <= (release_altitude_m - RELEASE_HYST_M)) {
            if (++s->release_count > RELEASE_CONFIRM) {
                s->state = STATE_PQ_RELEASE;
                s->release_command = true;
                s->release_count = 0;
            }
        } else {
            s->release_count = 0;
        }
        break;
    }

    case STATE_PQ_RELEASE: {
        if (fabsf(s->vertical_speed_mps) < LANDED_SPEED_MPS && alt <= LANDED_MAX_ALT_M) {
            if (++s->landed_count > LANDED_CONFIRM) {
                s->state = STATE_LANDED;
                s->landed = true;
                s->active = false;
            }
        } else {
            s->landed_count = 0;
        }
        break;
    }

    case STATE_LANDED:
    default:
        break;
    }
}

const char *FswStateName(FswStateId state)
{
    static const char *names[] = {
        "LAUNCH_PAD", "ASCENT", "APOGEE", "DESCENT", "PQ_RELEASE", "LANDED"
    };
    if ((uint8_t)state >= (uint8_t)(sizeof(names) / sizeof(names[0]))) return "UNKNOWN";
    return names[state];
}

bool FswTakeReleaseCommand(FswState *s)
{
    bool cmd = s->release_command;
    s->release_command = false;
    return cmd;
}

void FswSaveToNvm(const FswState *s, void (*write)(const FswNvmRecord *))
{
    FswNvmRecord rec;
    rec.state = (uint8_t)s->state;
    rec.pad_pressure_pa = s->pad_pressure_pa;
    rec.maximum_height_m = s->maximum_height_m;
    write(&rec);
}

bool FswLoadFromNvm(FswState *s, bool (*read)(FswNvmRecord *))
{
    FswNvmRecord rec;
    if (!read(&rec)) return false;
    if (rec.state > (uint8_t)STATE_LANDED) return false;
    s->state = (FswStateId)rec.state;
    s->pad_pressure_pa = rec.pad_pressure_pa;
    s->maximum_height_m = rec.maximum_height_m;
    s->previous_altitude_m = rec.maximum_height_m;
    s->previous_filtered_altitude_m = rec.maximum_height_m;
    s->filtered_altitude_m = rec.maximum_height_m;
    s->vertical_speed_mps = 0.0f;
    s->armed = (s->state != STATE_LAUNCH_PAD);
    s->active = (s->state != STATE_LANDED);
    return true;
}

#ifdef FSW_DEMO
static float altitude_to_pressure_m(float altitude_m)
{
    float ratio = 1.0f - altitude_m / 44330.0f;
    if (ratio < 0.0f) ratio = 0.0f;
    return SEA_LEVEL_PRESSURE_PA * powf(ratio, BARO_EXPONENT_INV);
}

int main(void)
{
    FswState fsw;
    FswInit(&fsw);

    const float dt = 0.1f;
    FswStateId last = fsw.state;

    for (uint16_t i = 0; i < PAD_CAL_SAMPLES; i++) {
        FswFeedPadCalibration(&fsw, altitude_to_pressure_m(0.0f) + 3.0f);
    }
    FswSetArmed(&fsw, true);

    float profile_alt = 0.0f;
    int phase = 0;
    for (int step = 0; step < 1000; step++) {
        if (phase == 0) {
            profile_alt += 45.0f * dt;
            if (profile_alt >= 670.0f) phase = 1;
        } else if (phase == 1) {
            static int hold = 0;
            if (++hold > 20) phase = 2;
        } else if (phase == 2) {
            profile_alt -= 10.0f * dt;
            if (profile_alt <= 0.0f) {
                profile_alt = 0.0f;
                phase = 3;
            }
        }

        float noise = ((step % 7) - 3) * 0.15f;
        float pressure = altitude_to_pressure_m(profile_alt + noise);
        FswUpdate(&fsw, pressure, dt);

        if (fsw.state != last) {
            printf("t=%.1fs  ->  %s  (alt=%.1fm, peak=%.1fm)\n",
                   0.1f * (float)step, FswStateName(fsw.state),
                   fsw.filtered_altitude_m, fsw.maximum_height_m);
            last = fsw.state;
        }
    }

    printf("final state: %s\n", FswStateName(fsw.state));
    return 0;
}
#endif
