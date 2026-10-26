/*
 * RadarMeasurement.h
 *
 *  Created on: Oct 26, 2026
 *      Author: julia
 */

#ifndef RADAR_MEASUREMENT_H
#define RADAR_MEASUREMENT_H

#include <stdint.h>
#include <stdbool.h>

typedef struct {
    float    range_m;        /* target range, meters            */
    float    velocity_mps;   /* radial velocity, m/s            */
    float    angle_rad;      /* angle of arrival, radians       */
    uint32_t timestamp_ms;   /* HAL_GetTick() at measurement    */
    bool     valid;          /* true if this is a real detection */
} RadarMeasurement;

#endif
