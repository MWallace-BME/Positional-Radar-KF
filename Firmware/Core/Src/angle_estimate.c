/*
 * angle_estimate.c
 *
 *  Created on: Oct 26, 2026
 *      Author: julian
 */

#include "angle_estimate.h"
#include <math.h>
#include <stdlib.h>

#define PI_F      3.14159265359f
#define TWO_PI_F  6.28318530718f

/* Wrap an angle into [-pi, pi]. Cheaper than atan2f(sinf(x), cosf(x)). */
static float wrap_pi(float x)
{
    while (x >  PI_F) x -= TWO_PI_F;
    while (x < -PI_F) x += TWO_PI_F;
    return x;
}

/* Phase of X1 * conj(X2), i.e. how far Rx2 lags Rx1. */
static float phase_diff(const float *x1, const float *x2)
{
    float re = x1[0] * x2[0] + x1[1] * x2[1];
    float im = x1[1] * x2[0] - x1[0] * x2[1];
    return atan2f(im, re);
}

float Phase_Extract(const float *x, uint32_t bin)
{
    return atan2f(x[2 * bin + 1], x[2 * bin]);
}

float AoA_CalibratePhase(const float *rx1_bin, const float *rx2_bin)
{
    /* Reflector at boresight should give 0 phase difference; whatever we
     * measure instead is the fixed channel mismatch (cables, op-amps, ADC). */
    return phase_diff(rx1_bin, rx2_bin);
}

bool AoA_Estimate(const float *rx1_bin, const float *rx2_bin,
                  float phase_cal_rad, float *angle_rad)
{
    if (rx1_bin == 0 || rx2_bin == 0 || angle_rad == 0) return false;

    /* 1. Phase difference between receivers, minus the channel mismatch. */
    float dphi = wrap_pi(phase_diff(rx1_bin, rx2_bin) - phase_cal_rad);

    /* 2. dphi = 2*pi*d*sin(theta)/lambda  ->  solve for sin(theta). */
    const float lambda = SPEED_OF_LIGHT / RADAR_FC_HZ;
    float s = (dphi * lambda) / (TWO_PI_F * RX_SPACING_M);

    /* 3. Safety clamp so asinf never sees a value outside [-1, 1]. */
    if (s >  1.0f) s =  1.0f;
    if (s < -1.0f) s = -1.0f;

    /* 4. Convert to angle and reject anything outside the trusted field of view. */
    float theta = asinf(s);
    if (fabsf(theta) > AOA_MAX_RAD) return false;

    *angle_rad = theta;
    return true;
}

RadarMeasurement Measurement_Pack(float range_m, float velocity_mps,
                                  float angle_rad, uint32_t timestamp_ms)
{
    RadarMeasurement m;
    m.range_m      = range_m;
    m.velocity_mps = velocity_mps;
    m.angle_rad    = angle_rad;
    m.timestamp_ms = timestamp_ms;
    m.valid        = true;
    return m;
}

/* NOTE: beyond ~45 deg (asin(lambda/2d)) the phase wraps and a target aliases to a
 * smaller angle that looks valid. Software cannot detect that; the antenna beam
 * pattern (+/-40 deg) is what keeps real targets out of that region.
 *
 * Build a synthetic Rx pair for a target at theta, with a known channel
 * phase offset, then check that AoA_Estimate recovers theta. */
int AoA_SelfTest(void)
{
    const float lambda = SPEED_OF_LIGHT / RADAR_FC_HZ;
    const float cal    = 0.35f;             /* pretend channel mismatch, rad */
    int failures = 0;

    for (int deg = -45; deg <= 45; deg += 5) {
        float theta = (float)deg * PI_F / 180.0f;
        float delta = TWO_PI_F * RX_SPACING_M * sinf(theta) / lambda + cal;

        float rx1[2] = { 1.0f, 0.0f };
        float rx2[2] = { cosf(-delta), sinf(-delta) };   /* rx1 * e^(-j*delta) */

        float est;
        bool ok = AoA_Estimate(rx1, rx2, cal, &est);
        bool should_pass = (abs(deg) <= 40);

        if (should_pass) {
            if (!ok || fabsf(est - theta) > 0.01f) failures++;
        } else {
            if (ok) failures++;     /* 41..45 deg: unambiguous but outside trusted FOV */
        }
    }
    return failures;
}

