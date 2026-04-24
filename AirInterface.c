#include "iio.h"
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <stdint.h>
#include <string.h>

#define MHZ(x) ((long long)(x * 1000000.0 + .5))
#define BUFFER_SIZE (1024 * 256)

/* ------- CRC-24 for ADS-B (polynomial 0x1FFF409) ------- */
static uint32_t crc24_table[256];

static void crc24_init(void) {
    const uint32_t poly = 0x1FFF409;
    for (int i = 0; i < 256; i++) {
        uint32_t crc = (uint32_t)i << 16;
        for (int j = 0; j < 8; j++) {
            crc <<= 1;
            if (crc & 0x1000000)
                crc ^= poly;
        }
        crc24_table[i] = crc & 0xFFFFFF;
    }
}

static uint32_t crc24_compute(const uint8_t *data, int len) {
    uint32_t crc = 0;
    for (int i = 0; i < len; i++)
        crc = crc24_table[((crc >> 16) ^ data[i]) & 0xFF] ^ (crc << 8) & 0xFFFFFF;
    return crc;
}

/* ------- Debug counters ------- */
static int preamble_count = 0;
static int crc_fail_count = 0;
static int df_reject_count = 0;
static int valid_count = 0;
static int buffer_count = 0;

/*
 * ADS-B Preamble timing (in microseconds):
 *
 *   Pulse 1: 0.0 us
 *   Pulse 2: 1.0 us
 *   Pulse 3: 3.5 us
 *   Pulse 4: 4.5 us
 *   Data starts at 8.0 us
 *   Each data bit = 1.0 us
 *   Total frame = 120.0 us (8.0 + 112 * 1.0)
 *
 * These will be converted to samples based on actual sampling rate.
 */

#define ADSB_BITS   112
#define ADSB_BYTES  14

/* Preamble timing in microseconds */
#define PRE_PULSE1_US   0.0
#define PRE_PULSE2_US   1.0
#define PRE_PULSE3_US   3.5
#define PRE_PULSE4_US   4.5
#define DATA_START_US   8.0
#define BIT_DURATION_US 1.0

/* Adaptive timing structure */
typedef struct {
    long long sampling_rate;
    double samples_per_us;
    int data_start_sample;
    int frame_len_samples;
    double samples_per_bit;
    /* Preamble pulse positions (in samples) */
    int pulse1_sample;
    int pulse2_sample;
    int pulse3_sample;
    int pulse4_sample;
    /* Gap sample positions for noise estimation */
    int gap_samples[6];
    int gap_count;
} TimingConfig;

/* Calculate adaptive timing based on sampling rate */
static TimingConfig* calculate_timing(long long sr) {
    TimingConfig *cfg = (TimingConfig *)malloc(sizeof(TimingConfig));
    if (!cfg) return NULL;
    
    cfg->sampling_rate = sr;
    cfg->samples_per_us = sr / 1e6;
    
    /* Calculate sample positions based on microsecond timings */
    cfg->pulse1_sample = (int)(PRE_PULSE1_US * cfg->samples_per_us + 0.5);
    cfg->pulse2_sample = (int)(PRE_PULSE2_US * cfg->samples_per_us + 0.5);
    cfg->pulse3_sample = (int)(PRE_PULSE3_US * cfg->samples_per_us + 0.5);
    cfg->pulse4_sample = (int)(PRE_PULSE4_US * cfg->samples_per_us + 0.5);
    
    cfg->data_start_sample = (int)(DATA_START_US * cfg->samples_per_us + 0.5);
    cfg->samples_per_bit = BIT_DURATION_US * cfg->samples_per_us;
    
    /* Frame length: 8.0 us (preamble+gap) + 112 * 1.0 us (data) = 120.0 us */
    cfg->frame_len_samples = (int)(120.0 * cfg->samples_per_us + 0.5);
    
    /* Gap sample positions for noise estimation (between pulses) */
    /* At 2.5 MSPS gaps were at 1, 4, 5, 6, 7, 10 (in sample units) */
    /* Translate to microsecond positions and recalculate for actual SR */
    cfg->gap_samples[0] = (int)(0.4 * cfg->samples_per_us + 0.5);    /* 0.4 us */
    cfg->gap_samples[1] = (int)(1.6 * cfg->samples_per_us + 0.5);    /* 1.6 us */
    cfg->gap_samples[2] = (int)(2.0 * cfg->samples_per_us + 0.5);    /* 2.0 us */
    cfg->gap_samples[3] = (int)(2.4 * cfg->samples_per_us + 0.5);    /* 2.4 us */
    cfg->gap_samples[4] = (int)(2.8 * cfg->samples_per_us + 0.5);    /* 2.8 us */
    cfg->gap_samples[5] = (int)(4.0 * cfg->samples_per_us + 0.5);    /* 4.0 us */
    cfg->gap_count = 6;
    
    return cfg;
}

int main(void) {
    crc24_init();

    /* Get sampling rate from environment variable, default to 2.5 MSPS */
    const char *sr_env = getenv("ADSB_SR");
    double target_sr_msps = sr_env ? atof(sr_env) : 2.5;
    long long target_sr = (long long)(target_sr_msps * 1e6 + 0.5);
    
    if (target_sr < 2000000 || target_sr > 10000000) {
        fprintf(stderr, "Error: ADSB_SR must be 2-10 MSPS, got %.1f MSPS\n", target_sr_msps);
        return -1;
    }

    fprintf(stderr, "Using sampling rate: %.1f MSPS (%lld Hz)\n", target_sr_msps, target_sr);

    /* 1. Connect to ADALM-PLUTO */
    struct iio_context *ctx = iio_create_context_from_uri("ip:192.168.2.1");
    if (!ctx) {
        fprintf(stderr, "Error: Unable to connect to PlutoSDR.\n");
        return -1;
    }

    /* 2. Configure PHY */
    struct iio_device *phy = iio_context_find_device(ctx, "ad9361-phy");
    if (!phy) {
        fprintf(stderr, "Error: Could not find ad9361-phy device.\n");
        iio_context_destroy(ctx);
        return -1;
    }

    struct iio_channel *rx_lo = iio_device_find_channel(phy, "altvoltage0", true);
    if (!rx_lo) {
        fprintf(stderr, "Error: Could not find RX LO channel.\n");
        iio_context_destroy(ctx);
        return -1;
    }
    iio_channel_attr_write_longlong(rx_lo, "frequency", MHZ(1090));

    struct iio_channel *rx_cfg = iio_device_find_channel(phy, "voltage0", false);
    if (!rx_cfg) {
        fprintf(stderr, "Error: Could not find RX config channel.\n");
        iio_context_destroy(ctx);
        return -1;
    }

    iio_channel_attr_write_longlong(rx_cfg, "sampling_frequency", target_sr);
    iio_channel_attr_write_longlong(rx_cfg, "rf_bandwidth", target_sr);
    iio_channel_attr_write(rx_cfg, "gain_control_mode", "manual");
    iio_channel_attr_write_double(rx_cfg, "hardwaregain", 73.0);

    /* Readback to verify */
    double actual_gain = 0;
    long long actual_freq = 0, actual_sr = 0;
    iio_channel_attr_read_double(rx_cfg, "hardwaregain", &actual_gain);
    iio_channel_attr_read_longlong(rx_lo, "frequency", &actual_freq);
    iio_channel_attr_read_longlong(rx_cfg, "sampling_frequency", &actual_sr);
    fprintf(stderr, "Config: LO=%lld Hz, SR=%lld Hz, Gain=%.1f dB\n",
            actual_freq, actual_sr, actual_gain);

    /* Validate sampling rate is within supported range */
    if (actual_sr < 2000000 || actual_sr > 10000000) {
        fprintf(stderr, "ERROR: Sampling rate must be 2-10 MSPS, got %.3f MSPS\n", actual_sr / 1e6);
        iio_context_destroy(ctx);
        return -1;
    }

    /* Calculate adaptive timing configuration */
    TimingConfig *timing = calculate_timing(actual_sr);
    if (!timing) {
        fprintf(stderr, "Error: Failed to calculate timing configuration.\n");
        iio_context_destroy(ctx);
        return -1;
    }

    fprintf(stderr, "Timing config: %lld MSPS, samples/us=%.2f, data_start=%d, frame_len=%d, samples/bit=%.2f\n",
            actual_sr / 1000000, timing->samples_per_us, timing->data_start_sample,
            timing->frame_len_samples, timing->samples_per_bit);

    printf("SAMPLING_RATE: %lld Hz (%.1f MSPS)\n", actual_sr, actual_sr / 1e6);

    /* 3. Setup RX streaming device */
    struct iio_device *rx = iio_context_find_device(ctx, "cf-ad9361-lpc");
    if (!rx) {
        fprintf(stderr, "Error: Could not find RX streaming device.\n");
        iio_context_destroy(ctx);
        return -1;
    }

    struct iio_channel *rx_i = iio_device_find_channel(rx, "voltage0", false);
    struct iio_channel *rx_q = iio_device_find_channel(rx, "voltage1", false);
    if (!rx_i || !rx_q) {
        fprintf(stderr, "Error: Could not find I/Q channels.\n");
        iio_context_destroy(ctx);
        return -1;
    }
    iio_channel_enable(rx_i);
    iio_channel_enable(rx_q);

    struct iio_buffer *rxbuf = iio_device_create_buffer(rx, BUFFER_SIZE, false);
    if (!rxbuf) {
        fprintf(stderr, "Error: Could not create RX buffer.\n");
        iio_context_destroy(ctx);
        return -1;
    }

    double *mag = (double *)malloc(BUFFER_SIZE * sizeof(double));
    if (!mag) {
        fprintf(stderr, "Error: malloc failed.\n");
        iio_buffer_destroy(rxbuf);
        iio_context_destroy(ctx);
        return -1;
    }

    fprintf(stderr, "\nADS-B receiver running @ %.1f MSPS on 1090 MHz\n\n",
            actual_sr / 1e6);

    /* 4. Main receive loop */
    while (1) {
        ssize_t nbytes_rx = iio_buffer_refill(rxbuf);
        if (nbytes_rx < 0) {
            fprintf(stderr, "Warning: buffer refill error %zd\n", nbytes_rx);
            continue;
        }

        buffer_count++;

        char *ptr;
        char *p_end = (char *)iio_buffer_end(rxbuf);
        ptrdiff_t step = iio_buffer_step(rxbuf);

        int sample_count = 0;
        double peak_mag = 0;

        /* Compute magnitude */
        for (ptr = (char *)iio_buffer_first(rxbuf, rx_i);
             ptr < p_end && sample_count < BUFFER_SIZE;
             ptr += step, sample_count++) {
            int16_t iv = ((int16_t *)ptr)[0];
            int16_t qv = ((int16_t *)ptr)[1];
            mag[sample_count] = sqrt((double)iv * iv + (double)qv * qv);
            if (mag[sample_count] > peak_mag) peak_mag = mag[sample_count];
        }

        /* Debug: print stats every 10 buffers */
        if (buffer_count % 10 == 0) {
            fprintf(stderr, "[buf %d] samples=%d peak=%.1f | preambles=%d df_rej=%d crc_fail=%d valid=%d\n",
                    buffer_count, sample_count, peak_mag,
                    preamble_count, df_reject_count, crc_fail_count, valid_count);
        }

        /* 5. Scan for preambles with adaptive timing */
        for (int s = 0; s <= sample_count - timing->frame_len_samples; s++) {

            /*
             * Noise estimate from gap samples (positions between preamble pulses)
             */
            double noise = 0;
            for (int g = 0; g < timing->gap_count; g++) {
                int gap_idx = s + timing->gap_samples[g];
                if (gap_idx < sample_count)
                    noise += mag[gap_idx];
            }
            noise /= timing->gap_count;
            double thr = (noise < 1.0) ? 1.0 : noise * 2.0;

            /*
             * Check four preamble pulses.
             * Since pulses may land between samples, check candidate samples around each pulse position.
             */
            double p0_pos = s + timing->pulse1_sample;
            double p1_pos = s + timing->pulse2_sample;
            double p2_pos = s + timing->pulse3_sample;
            double p3_pos = s + timing->pulse4_sample;
            
            int p0_idx = (int)p0_pos;
            int p1_idx = (int)p1_pos;
            int p2_idx = (int)p2_pos;
            int p3_idx = (int)p3_pos;

            if (p0_idx >= sample_count || p1_idx + 1 >= sample_count || 
                p2_idx + 1 >= sample_count || p3_idx + 1 >= sample_count)
                continue;

            /* Take max of two candidate samples near each pulse */
            double p0 = mag[p0_idx];
            double p1 = (mag[p1_idx] > mag[p1_idx + 1]) ? mag[p1_idx] : mag[p1_idx + 1];
            double p2 = (mag[p2_idx] > mag[p2_idx + 1]) ? mag[p2_idx] : mag[p2_idx + 1];
            double p3 = (mag[p3_idx] > mag[p3_idx + 1]) ? mag[p3_idx] : mag[p3_idx + 1];

            if (p0 < thr || p1 < thr || p2 < thr || p3 < thr)
                continue;

            /* Extra: pulse average must be well above noise */
            double pulse_avg = (p0 + p1 + p2 + p3) / 4.0;
            if (pulse_avg < noise * 2.5)
                continue;

            preamble_count++;

            /* 6. Demodulate 112 bits via PPM with adaptive timing
             *
             * Each bit has duration BIT_DURATION_US (1.0 us)
             * Each bit occupies timing->samples_per_bit samples.
             * PPM decode: compare first half vs second half.
             */
            uint8_t frame[ADSB_BYTES];
            memset(frame, 0, sizeof(frame));

            for (int b = 0; b < ADSB_BITS; b++) {
                double bit_start_sample = s + timing->data_start_sample + (b * timing->samples_per_bit);
                int bit_start_idx = (int)bit_start_sample;
                int half_point = bit_start_idx + (int)(timing->samples_per_bit / 2.0);

                if (half_point + 1 >= sample_count)
                    break;

                /* PPM: compare first half vs second half */
                double first_half_max = 0;
                for (int i = bit_start_idx; i < half_point && i < sample_count; i++)
                    if (mag[i] > first_half_max) first_half_max = mag[i];

                double second_half_max = 0;
                for (int i = half_point; i < bit_start_idx + (int)timing->samples_per_bit && i < sample_count; i++)
                    if (mag[i] > second_half_max) second_half_max = mag[i];

                int bit = (first_half_max > second_half_max) ? 1 : 0;
                frame[b / 8] = (frame[b / 8] << 1) | bit;
            }

            /* 7. Check Downlink Format (DF) */
            uint8_t df = (frame[0] >> 3) & 0x1F;

            int expected_bits;
            if (df == 0 || df == 4 || df == 5 || df == 11) {
                expected_bits = 56;
            } else if (df == 16 || df == 17 || df == 18 || df == 19 || df == 20 || df == 21) {
                expected_bits = 112;
            } else {
                df_reject_count++;
                continue;
            }

            int expected_bytes = expected_bits / 8;

            /* 8. CRC-24 check */
            uint32_t crc = crc24_compute(frame, expected_bytes - 3);
            uint32_t pi  = ((uint32_t)frame[expected_bytes - 3] << 16)
                         | ((uint32_t)frame[expected_bytes - 2] << 8)
                         |  (uint32_t)frame[expected_bytes - 1];

            if (df == 11 || df == 17 || df == 18) {
                if ((crc ^ pi) != 0) {
                    crc_fail_count++;
                    continue;
                }
            }

            /* 9. Output the validated hex frame */
            valid_count++;
            putchar('*');
            for (int j = 0; j < expected_bytes; j++)
                printf("%02X", frame[j]);
            printf(";\n");
            fflush(stdout);

            s += timing->frame_len_samples - 1;
        }
    }

    free(timing);
    free(mag);
    iio_buffer_destroy(rxbuf);
    iio_context_destroy(ctx);
    return 0;
}