/*
 * radar.c - Convergence Radar signal chain (STM32F446RE + K-LC7)
 *
 * Data flow
 *   TIM2 (64 kHz TRGO) -> DAC ramp via DMA1, and ADC1+ADC2 dual simultaneous via DMA2.
 *   DMA2 ping-pong buffer holds 2 chirps. Each half-transfer / transfer-complete
 *   interrupt runs the range FFT for one chirp (both receivers) and stores 32
 *   range bins into a "radar cube". After 64 chirps the cube is handed to the
 *   main loop (double-buffered), which runs clutter removal, Doppler FFT,
 *   detection, angle, smoothing and telemetry. The CPU sleeps (WFI) in between.
 *
 * Serial output (USART2, 115200 8N1, plain text, one line per frame)
 *   $B,<reset cause>,<firmware version>                          at boot
 *   $F,t_ms,frame,ndet,range_mm,az_cdeg,vel_mms,snr_dbx10,x_mm,y_mm,
 *      trk,xs_mm,ys_mm,dphi_mrad,overruns                         track mode
 *   $S,frame,v0,...,v31     range spectrum of Rx1 in dB x10       spectrum mode
 *   $R,i1,q1,i2,q2,...      one raw chirp (64 x 4 ADC counts)     raw mode (1/s)
 *   All numbers are integers (no float printf needed).
 *
 * Serial commands (single characters typed into the terminal / sent by the viewer)
 *   t = track mode (default)   s = spectrum mode   r = raw mode   ? = settings
 */
#include "radar.h"
#include <math.h>
#include <string.h>
#include <stdio.h>

#ifndef RADAR_HOST_TEST
#include "main.h"
extern ADC_HandleTypeDef  hadc1;
extern ADC_HandleTypeDef  hadc2;
extern DAC_HandleTypeDef  hdac;
extern TIM_HandleTypeDef  htim2;
extern UART_HandleTypeDef huart2;
extern IWDG_HandleTypeDef hiwdg;
#endif

#define FW_VERSION "1.0"
#define PI_F 3.14159265f

#define NS RADAR_NS
#define NC RADAR_NC
#define NR RADAR_NR

/* ---------------- buffers ---------------- */
static uint16_t ramp[NS];
/* 2 chirps x 64 ticks x 2 ranks, one 32-bit word per rank (ADC2<<16 | ADC1) */
static uint32_t adc_buf[2 * NS * 2];
/* double-buffered radar cube: [buffer][rx][chirp][range bin][re,im] = 64 KB */
static float cube[2][2][NC][NR][2];
static float pmap[NR][NC];          /* power map after Doppler FFT */
static float win[NS];
static float tw_re[NS / 2], tw_im[NS / 2];
static uint8_t brev[NS];

/* ---------------- shared state (ISR <-> main) ---------------- */
static volatile uint8_t  fill_idx = 0;
static volatile uint16_t chirp_idx = 0;
static volatile uint8_t  frame_ready = 0;
static volatile uint8_t  proc_idx = 0;
static volatile uint32_t frame_count = 0;
static volatile uint32_t overruns = 0;
static volatile char     mode = 't';
static volatile uint8_t  raw_req = 0, raw_ready = 0, help_req = 0, rx_rearm = 0;
static uint32_t raw_snap[2 * NS];
static uint8_t  rx_byte;

/* track smoothing */
static uint8_t trk_valid = 0;
static uint16_t trk_miss = 0;
static float xs = 0, ys = 0;

/* ================================================================ */
/*                              DSP                                 */
/* ================================================================ */
void radar_dsp_init(void)
{
    for (int n = 0; n < NS; n++)
        win[n] = 0.5f - 0.5f * cosf(2.0f * PI_F * (float)n / (float)(NS - 1));
    for (int k = 0; k < NS / 2; k++) {
        tw_re[k] = cosf(2.0f * PI_F * (float)k / (float)NS);
        tw_im[k] = -sinf(2.0f * PI_F * (float)k / (float)NS);
    }
    for (int i = 0; i < NS; i++) {          /* 6-bit reversal for N = 64 */
        int r = 0;
        for (int b = 0; b < 6; b++) if (i & (1 << b)) r |= 1 << (5 - b);
        brev[i] = (uint8_t)r;
    }
}

/* In-place forward complex FFT, N = 64, interleaved re/im. */
static void fft64(float *x)
{
    for (int i = 0; i < NS; i++) {
        int j = brev[i];
        if (j > i) {
            float tr = x[2*i], ti = x[2*i+1];
            x[2*i] = x[2*j]; x[2*i+1] = x[2*j+1];
            x[2*j] = tr;     x[2*j+1] = ti;
        }
    }
    for (int len = 2; len <= NS; len <<= 1) {
        int half = len >> 1, step = NS / len;
        for (int i = 0; i < NS; i += len) {
            for (int k = 0; k < half; k++) {
                float wr = tw_re[k * step], wi = tw_im[k * step];
                int a = i + k, b = a + half;
                float tr = x[2*b] * wr - x[2*b+1] * wi;
                float ti = x[2*b] * wi + x[2*b+1] * wr;
                x[2*b]   = x[2*a]   - tr;
                x[2*b+1] = x[2*a+1] - ti;
                x[2*a]   += tr;
                x[2*a+1] += ti;
            }
        }
    }
}

/* Range FFT for one chirp. words[2n] = I1 | I2<<16, words[2n+1] = Q1 | Q2<<16.
 * Runs in the DMA interrupt (~tens of microseconds). */
void radar_process_chirp(const uint32_t *words)
{
    static float b1[2 * NS], b2[2 * NS];
    float sI1 = 0, sQ1 = 0, sI2 = 0, sQ2 = 0;

    for (int n = 0; n < NS; n++) {
        uint32_t wi = words[2*n], wq = words[2*n+1];
        b1[2*n]   = (float)(wi & 0xFFFF); b2[2*n]   = (float)(wi >> 16);
        b1[2*n+1] = (float)(wq & 0xFFFF); b2[2*n+1] = (float)(wq >> 16);
        sI1 += b1[2*n]; sQ1 += b1[2*n+1]; sI2 += b2[2*n]; sQ2 += b2[2*n+1];
    }
    sI1 /= NS; sQ1 /= NS; sI2 /= NS; sQ2 /= NS;
    for (int n = 0; n < NS; n++) {          /* remove DC (1.65 V bias), window */
        b1[2*n]   = (b1[2*n]   - sI1) * win[n];
        b1[2*n+1] = (b1[2*n+1] - sQ1) * win[n];
        b2[2*n]   = (b2[2*n]   - sI2) * win[n];
        b2[2*n+1] = (b2[2*n+1] - sQ2) * win[n];
    }
    fft64(b1);
    fft64(b2);

    uint8_t f = fill_idx;
    uint16_t c = chirp_idx;
    for (int k = 0; k < NR; k++) {
        int src = RADAR_BEAT_NEGATIVE ? ((NS - k) % NS) : k;
        cube[f][0][c][k][0] = b1[2*src]; cube[f][0][c][k][1] = b1[2*src+1];
        cube[f][1][c][k][0] = b2[2*src]; cube[f][1][c][k][1] = b2[2*src+1];
    }

    if (raw_req && c == 0) {                /* one raw chirp for raw mode */
        memcpy(raw_snap, words, sizeof(raw_snap));
        raw_req = 0;
        raw_ready = 1;
    }

    c++;
    if (c >= NC) {
        c = 0;
        frame_count++;
        if (!frame_ready) {                 /* hand the full cube to main */
            proc_idx = f;
            fill_idx = f ^ 1;
            frame_ready = 1;
        } else {
            overruns++;                     /* main still busy: overwrite this cube */
        }
    }
    chirp_idx = c;
}

static float wrap_pi(float a)
{
    while (a >  PI_F) a -= 2.0f * PI_F;
    while (a < -PI_F) a += 2.0f * PI_F;
    return a;
}

static int dsigned(int d) { return d < NC / 2 ? d : d - NC; }

static int cell_ok(int r, int d)
{
    int ds = dsigned(d);
    if (r < RADAR_MIN_BIN || r > RADAR_MAX_BIN || r >= NR) return 0;
    if (RADAR_USE_MTI && (ds > -RADAR_MIN_DOPPLER && ds < RADAR_MIN_DOPPLER)) return 0;
    return 1;
}

/* Clutter removal + Doppler FFT + detection + angle, on one finished cube. */
void radar_process_frame(uint8_t p, radar_det_t *out)
{
    static float col[2 * NC];
    memset(out, 0, sizeof(*out));

    for (int rx = 0; rx < 2; rx++) {
        for (int r = 0; r < NR; r++) {
            float mr = 0, mi = 0;
            for (int c = 0; c < NC; c++) {
                col[2*c]   = cube[p][rx][c][r][0];
                col[2*c+1] = cube[p][rx][c][r][1];
                mr += col[2*c]; mi += col[2*c+1];
            }
            if (RADAR_USE_MTI) { mr /= NC; mi /= NC; } else { mr = 0; mi = 0; }
            for (int c = 0; c < NC; c++) {
                col[2*c]   = (col[2*c]   - mr) * win[c];
                col[2*c+1] = (col[2*c+1] - mi) * win[c];
            }
            fft64(col);
            for (int d = 0; d < NC; d++) {  /* write back in place: index = Doppler bin */
                cube[p][rx][d][r][0] = col[2*d];
                cube[p][rx][d][r][1] = col[2*d+1];
            }
        }
    }

    /* power map and noise floor */
    double noise = 0; int ncell = 0;
    float best = 0; int br = -1, bd = -1;
    for (int r = 0; r < NR; r++) {
        for (int d = 0; d < NC; d++) {
            float a = cube[p][0][d][r][0], b = cube[p][0][d][r][1];
            float c2 = cube[p][1][d][r][0], e = cube[p][1][d][r][1];
            float pw = a*a + b*b + c2*c2 + e*e;
            pmap[r][d] = pw;
            if (cell_ok(r, d)) {
                noise += pw; ncell++;
                if (pw > best) { best = pw; br = r; bd = d; }
            }
        }
    }
    if (ncell == 0 || br < 0) return;
    float nf = (float)(noise / ncell) + 1e-12f;
    float thr = nf * powf(10.0f, RADAR_THRESH_DB / 10.0f);

    /* count separate peaks above threshold (local maxima, 3x3) */
    for (int r = RADAR_MIN_BIN; r <= RADAR_MAX_BIN && r < NR; r++) {
        for (int d = 0; d < NC; d++) {
            if (!cell_ok(r, d) || pmap[r][d] < thr) continue;
            int peak = 1;
            for (int dr = -1; dr <= 1 && peak; dr++)
                for (int dd = -1; dd <= 1; dd++) {
                    int rr = r + dr, d2 = (d + dd + NC) % NC;
                    if ((dr || dd) && rr >= 0 && rr < NR && pmap[rr][d2] > pmap[r][d]) { peak = 0; break; }
                }
            if (peak && out->ndet < 255) out->ndet++;
        }
    }

    out->snr_db = 10.0f * log10f(best / nf);
    out->rbin = (uint16_t)br;
    out->dbin = (uint16_t)bd;
    if (best < thr) return;

    /* range with parabolic interpolation across neighbouring range bins */
    float delta = 0;
    if (br > 0 && br < NR - 1) {
        float pm = pmap[br-1][bd], p0 = pmap[br][bd], pp = pmap[br+1][bd];
        float den = pm - 2.0f * p0 + pp;
        if (den < 0) delta = 0.5f * (pm - pp) / den;
        if (delta > 0.5f) delta = 0.5f;
        if (delta < -0.5f) delta = -0.5f;
    }
    out->range_m = ((float)br + delta) * RADAR_RANGE_RES_M;
    out->vel_mps = (float)dsigned(bd) * RADAR_LAMBDA_M / (2.0f * NC * RADAR_TC_S);

    /* angle from Rx1/Rx2 phase difference: arg(X1 * conj(X2)) */
    float a = cube[p][0][bd][br][0], b = cube[p][0][bd][br][1];
    float c = cube[p][1][bd][br][0], d = cube[p][1][bd][br][1];
    float re = a*c + b*d, im = b*c - a*d;
    out->dphi_rad = atan2f(im, re);
    float s = RADAR_ANGLE_SIGN * RADAR_ANGLE_K * wrap_pi(out->dphi_rad - RADAR_PHASE_CAL_RAD);
    if (s > 1.0f) s = 1.0f;
    if (s < -1.0f) s = -1.0f;
    out->az_rad = asinf(s);
    out->x_m = out->range_m * sinf(out->az_rad);
    out->y_m = out->range_m * cosf(out->az_rad);
    out->valid = 1;
}

/* ================================================================ */
/*                         STM32 integration                        */
/* ================================================================ */
#ifndef RADAR_HOST_TEST

static void uart_send(const char *s, int n)
{
    HAL_UART_Transmit(&huart2, (uint8_t *)s, (uint16_t)n, 200);
}

static void send_help(void)
{
    char buf[256];
    int n = snprintf(buf, sizeof(buf),
        "$H,fw=%s,range_res_mm=%ld,beat_neg=%d,mti=%d,thresh_db=%ld,bins=%d-%d,"
        "phase_cal_mrad=%ld,modes=t/s/r\r\n",
        FW_VERSION, (long)(RADAR_RANGE_RES_M * 1000.0f), RADAR_BEAT_NEGATIVE, RADAR_USE_MTI,
        (long)RADAR_THRESH_DB, RADAR_MIN_BIN, RADAR_MAX_BIN, (long)(RADAR_PHASE_CAL_RAD * 1000.0f));
    uart_send(buf, n);
}

void radar_start(void)
{
    radar_dsp_init();
    for (int i = 0; i < NS; i++)
        ramp[i] = (uint16_t)(RADAR_RAMP_LO + ((RADAR_RAMP_HI - RADAR_RAMP_LO) * i) / (NS - 1));

    /* report why we booted (watchdog reset vs normal) */
    const char *cause = "POR";
    if (__HAL_RCC_GET_FLAG(RCC_FLAG_IWDGRST)) cause = "IWDG";
    else if (__HAL_RCC_GET_FLAG(RCC_FLAG_PINRST)) cause = "PIN";
    __HAL_RCC_CLEAR_RESET_FLAGS();
    char buf[64];
    int n = snprintf(buf, sizeof(buf), "$B,%s,fw=%s\r\n", cause, FW_VERSION);
    uart_send(buf, n);
    send_help();

    __HAL_DBGMCU_FREEZE_IWDG();             /* watchdog pauses at debugger breakpoints */

    /* Arm both DMA streams first, then start the timer, so sample n lines up with ramp step n. */
    HAL_DAC_Start_DMA(&hdac, DAC_CHANNEL_1, (uint32_t *)ramp, NS, DAC_ALIGN_12B_R);
    HAL_ADC_Start(&hadc2);
    HAL_ADCEx_MultiModeStart_DMA(&hadc1, adc_buf, 2 * NS * 2);
    HAL_TIM_Base_Start(&htim2);

    HAL_UART_Receive_IT(&huart2, &rx_byte, 1);
    HAL_IWDG_Refresh(&hiwdg);
}

static void send_track(const radar_det_t *det)
{
    char buf[200];
    int n = snprintf(buf, sizeof(buf),
        "$F,%lu,%lu,%u,%ld,%ld,%ld,%ld,%ld,%ld,%u,%ld,%ld,%ld,%lu\r\n",
        (unsigned long)HAL_GetTick(), (unsigned long)frame_count, (unsigned)det->ndet,
        (long)(det->range_m * 1000.0f), (long)(det->az_rad * 18000.0f / PI_F),
        (long)(det->vel_mps * 1000.0f), (long)(det->snr_db * 10.0f),
        (long)(det->x_m * 1000.0f), (long)(det->y_m * 1000.0f),
        (unsigned)trk_valid, (long)(xs * 1000.0f), (long)(ys * 1000.0f),
        (long)(det->dphi_rad * 1000.0f), (unsigned long)overruns);
    uart_send(buf, n);
}

/* Rx1 range spectrum (dB x10), averaged over the frame's chirps, before clutter removal. */
static void send_spectrum(uint8_t p)
{
    char buf[400];
    int n = snprintf(buf, sizeof(buf), "$S,%lu", (unsigned long)frame_count);
    for (int r = 0; r < NR && n < (int)sizeof(buf) - 12; r++) {
        float acc = 0;
        for (int c = 0; c < NC; c++) {
            float a = cube[p][0][c][r][0], b = cube[p][0][c][r][1];
            acc += sqrtf(a*a + b*b);
        }
        n += snprintf(buf + n, sizeof(buf) - n, ",%ld", (long)(200.0f * log10f(acc / NC + 1e-3f)));
    }
    n += snprintf(buf + n, sizeof(buf) - n, "\r\n");
    uart_send(buf, n);
}

static void send_raw(void)
{
    static char buf[1700];
    int n = snprintf(buf, sizeof(buf), "$R");
    for (int i = 0; i < NS && n < (int)sizeof(buf) - 30; i++) {
        uint32_t wi = raw_snap[2*i], wq = raw_snap[2*i+1];
        n += snprintf(buf + n, sizeof(buf) - n, ",%u,%u,%u,%u",
                      (unsigned)(wi & 0xFFFF), (unsigned)(wq & 0xFFFF),
                      (unsigned)(wi >> 16), (unsigned)(wq >> 16));
    }
    n += snprintf(buf + n, sizeof(buf) - n, "\r\n");
    uart_send(buf, n);
}

void radar_loop(void)
{
    static uint32_t last_raw = 0;

    if (rx_rearm) {                          /* re-arm UART receive if the ISR couldn't */
        if (HAL_UART_Receive_IT(&huart2, &rx_byte, 1) == HAL_OK) rx_rearm = 0;
    }
    if (help_req) { help_req = 0; send_help(); }
    if (mode == 'r' && HAL_GetTick() - last_raw >= 1000) { last_raw = HAL_GetTick(); raw_req = 1; }
    if (raw_ready) { raw_ready = 0; send_raw(); }

    if (!frame_ready) {
        __WFI();                             /* sleep until the next interrupt */
        return;
    }

    uint8_t p = proc_idx;
    if (mode == 's') send_spectrum(p);       /* uses the cube before Doppler processing */

    radar_det_t det;
    radar_process_frame(p, &det);
    frame_ready = 0;                         /* cube p may now be refilled */

    if (det.valid) {
        if (!trk_valid) { xs = det.x_m; ys = det.y_m; trk_valid = 1; }
        else { xs += RADAR_SMOOTH_ALPHA * (det.x_m - xs); ys += RADAR_SMOOTH_ALPHA * (det.y_m - ys); }
        trk_miss = 0;
    } else if (trk_valid && ++trk_miss > RADAR_TRACK_TIMEOUT) {
        trk_valid = 0;
    }

    if (mode == 't') send_track(&det);

    if ((frame_count & 7) == 0) BSP_LED_Toggle(LED2);   /* heartbeat ~2 Hz */
    HAL_IWDG_Refresh(&hiwdg);                /* once per completed frame */
}

/* ---------------- HAL callbacks ---------------- */
void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef *h)
{
    if (h->Instance == ADC1) radar_process_chirp(&adc_buf[0]);
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *h)
{
    if (h->Instance == ADC1) radar_process_chirp(&adc_buf[2 * NS]);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *h)
{
    if (h->Instance != USART2) return;
    switch (rx_byte) {
        case 't': case 's': case 'r': mode = (char)rx_byte; break;
        case '?': help_req = 1; break;
        default: break;
    }
    if (HAL_UART_Receive_IT(&huart2, &rx_byte, 1) != HAL_OK) rx_rearm = 1;
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *h)
{
    if (h->Instance == USART2) rx_rearm = 1;
}

#endif /* RADAR_HOST_TEST */
