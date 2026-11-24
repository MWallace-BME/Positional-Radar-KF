/*
 * radar.h - Convergence Radar signal chain for STM32F446RE + RFbeam K-LC7
 *
 * Chirp:    TIM2 at 64 kHz triggers the DAC (ramp) and ADC1+ADC2 (dual simultaneous).
 *           64 samples per 1 ms chirp, 64 chirps per 64 ms frame.
 * Capture:  ADC1 rank1 = I1 (PA0), rank2 = Q1 (PA1)
 *           ADC2 rank1 = I2 (PC0), rank2 = Q2 (PC1)
 *           DMA2 (ADC1, DMA access mode 2) packs each pair as one 32-bit word:
 *           low half = ADC1, high half = ADC2.
 * Output:   one text line per frame on USART2 (115200 8N1), see radar.c header.
 *
 * Everything you are likely to tune on the bench is in the CONFIG block below.
 */
#ifndef RADAR_H
#define RADAR_H

#include <stdint.h>

/* ======================= CONFIG (tune on the bench) ======================= */

/* Metres per range bin. Theory: c / (2 * sweep bandwidth) ~ 0.75 m for ~200 MHz.
 * Calibrate: put a flat target at a measured distance D, read the reported
 * range R, then set this to RADAR_RANGE_RES_M * D / R. */
#define RADAR_RANGE_RES_M      0.75f

/* If targets show up at the wrong range, or moving away looks the same as
 * nothing, the I/Q sign is flipped: set this to 1 (uses the negative-frequency
 * half of the range FFT). Check with spectrum mode ('s'). */
#define RADAR_BEAT_NEGATIVE    0

/* Set to -1.0f if left and right come out swapped. */
#define RADAR_ANGLE_SIGN       1.0f

/* Receiver phase offset (radians). Calibrate: target straight ahead (0 deg),
 * read the dphi_mrad field in track mode, enter it here divided by 1000. */
#define RADAR_PHASE_CAL_RAD    0.0f

/* Search window in range bins (bin 1 ~ 0.75 m, bin 12 ~ 9 m). */
#define RADAR_MIN_BIN          1
#define RADAR_MAX_BIN          12

/* Doppler bins around zero speed that are ignored (static clutter residue).
 * 1 = ignore only exactly-zero speed. */
#define RADAR_MIN_DOPPLER      1

/* Detection threshold above the average noise floor, in dB. */
#define RADAR_THRESH_DB        12.0f

/* 1 = remove static reflections (walls, furniture). A person standing
 * perfectly still fades out. 0 = keep everything (useful for range checks
 * against a wall or a metal plate). */
#define RADAR_USE_MTI          1

/* Position smoothing, 0..1. Lower = smoother but slower to follow. */
#define RADAR_SMOOTH_ALPHA     0.35f

/* Frames without a detection before the smoothed track is dropped (~64 ms each). */
#define RADAR_TRACK_TIMEOUT    16

/* DAC ramp limits in counts (0..4095 = 0..3.3 V): 248 = 0.2 V, 3351 = 2.7 V. */
#define RADAR_RAMP_LO          248
#define RADAR_RAMP_HI          3351

/* ========================== fixed geometry ========================== */
#define RADAR_NS        64      /* samples per chirp                    */
#define RADAR_NC        64      /* chirps per frame                     */
#define RADAR_NR        32      /* range bins kept per chirp            */
#define RADAR_TC_S      0.001f  /* chirp period                         */
#define RADAR_LAMBDA_M  0.012435f /* wavelength at 24.125 GHz           */
#define RADAR_ANGLE_K   0.2257f /* lambda / (2*pi*d), d = 8.763 mm      */

typedef struct {
    uint8_t  valid;          /* 1 if a detection passed the threshold   */
    uint8_t  ndet;           /* number of peaks above threshold         */
    float    range_m;
    float    az_rad;         /* + = right of boresight (if sign correct)*/
    float    vel_mps;        /* radial velocity, sign per I/Q convention*/
    float    snr_db;
    float    dphi_rad;       /* Rx1-Rx2 phase before calibration offset  */
    float    x_m, y_m;       /* y = straight ahead                      */
    uint16_t rbin, dbin;
} radar_det_t;

/* Call once after all MX_..._Init() calls (USER CODE BEGIN 2). */
void radar_start(void);

/* Call every pass of the while(1) loop (USER CODE BEGIN 3). Sleeps (WFI)
 * until a frame is ready, processes it, sends telemetry, refreshes IWDG. */
void radar_loop(void);

/* Exposed for testing. */
void radar_process_chirp(const uint32_t *words);
void radar_process_frame(uint8_t cube_idx, radar_det_t *out);
void radar_dsp_init(void);

#endif
