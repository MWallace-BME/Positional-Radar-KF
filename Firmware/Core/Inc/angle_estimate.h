/*
 * angle_estimate.h
 *
 *  Created on: Oct 26, 2026
 *      Author: julian
 */

#ifndef ANGLE_ESTIMATE_H
#define ANGLE_ESTIMATE_H

#include <stdint.h>
#include <stdbool.h>
#include "RadarMeasurement.h"   /* shared struct contract (shared/RadarMeasurement.h) */

/* ---------- Radar / antenna constants ---------- */
#define RADAR_FC_HZ     24.125e9f       /* center of K-LC7 24-24.25 GHz band        */
#define SPEED_OF_LIGHT  299792458.0f
#define RX_SPACING_M    0.008763f       /* Rx1-Rx2 spacing, K-LC7 datasheet (thesis) */

/* K-LC7 azimuth beamwidth is 80 deg -> +/-40 deg. With d/lambda ~ 0.70 the
 * unambiguous limit is about +/-45 deg, so anything past 40 deg is rejected. */
#define AOA_MAX_RAD     0.7000f         /* ~40.1 degrees (small margin for float rounding) */

/* Complex bins are passed as pointers to {real, imag}, the same layout
 * CMSIS-DSP arm_cfft_f32 uses (interleaved re, im, re, im, ...). */

/* Phase (rad, -pi..pi) of one complex bin in an interleaved array. */
float Phase_Extract(const float *cplx_interleaved, uint32_t bin);

/* Measures the Rx1/Rx2 phase offset with a reflector at boresight (0 deg).
 * Store the result and pass it to AoA_Estimate() as phase_cal_rad. */
float AoA_CalibratePhase(const float *rx1_bin, const float *rx2_bin);

/* Phase-comparison angle of arrival.
 * Returns true and writes *angle_rad (radians, 0 = boresight) on success.
 * Returns false if the angle is outside +/-AOA_MAX_RAD or inputs are bad. */
bool  AoA_Estimate(const float *rx1_bin, const float *rx2_bin,
                   float phase_cal_rad, float *angle_rad);

/* Packages range, velocity, angle, timestamp into the shared struct. */
RadarMeasurement Measurement_Pack(float range_m, float velocity_mps,
                                  float angle_rad, uint32_t timestamp_ms);

/* Synthetic-data self test. Returns number of failed checks (0 = pass). */
int AoA_SelfTest(void);

#endif /* ANGLE_ESTIMATE_H */
