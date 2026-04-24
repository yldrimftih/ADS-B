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
 * ADS-B Preamble at 2.5 MSPS (1 sample = 0.4 us):
 *
 *   Pulse 1: 0.0 us  -> sample  0.0  -> check sample 0
 *   Pulse 2: 1.0 us  -> sample  2.5  -> check samples 2,3
 *   Pulse 3: 3.5 us  -> sample  8.75 -> check samples 8,9
 *   Pulse 4: 4.5 us  -> sample 11.25 -> check samples 11,12
 *
 *   Gap samples (should be LOW): 1, 4, 5, 6, 7, 10
 *
 *   Data starts at 8.0 us -> sample 20
 *   Each data bit = 1.0 us = 2.5 samples
 *
 *   Total frame: 20 + 112*2.5 = 300 samples
 */

#define DATA_START  20
#define ADSB_BITS   112
#define ADSB_BYTES  14
#define FRAME_LEN   300

int main(void) {
    crc24_init();

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

    iio_channel_attr_write_longlong(rx_cfg, "sampling_frequency", MHZ(2.5));
    iio_channel_attr_write_longlong(rx_cfg, "rf_bandwidth", MHZ(2.5));
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

    if (actual_sr < 2400000 || actual_sr > 2600000) {
        fprintf(stderr, "ERROR: Expected ~2.5 MSPS, got %.3f MSPS\n", actual_sr / 1e6);
        fprintf(stderr, "Preamble timing will be wrong. Aborting.\n");
        iio_context_destroy(ctx);
        return -1;
    }

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

        /* 5. Scan for preambles (2.5 MSPS) */
        for (int s = 0; s <= sample_count - FRAME_LEN; s++) {

            /*
             * Noise estimate from gap samples (positions that should be LOW):
             * Gaps at: 1, 4, 5, 6, 7, 10 (between the four preamble pulses)
             */
            double noise = (mag[s + 1] + mag[s + 4] + mag[s + 5]
                          + mag[s + 6] + mag[s + 7] + mag[s + 10]) / 6.0;
            double thr = (noise < 1.0) ? 1.0 : noise * 2.0;

            /*
             * Check four preamble pulses.
             * Since pulses land between samples at 2.5 MSPS,
             * take the MAX of the two candidate samples for each pulse.
             */
            double p0 = mag[s + 0];
            double p1 = (mag[s + 2] > mag[s + 3])   ? mag[s + 2]  : mag[s + 3];
            double p2 = (mag[s + 8] > mag[s + 9])   ? mag[s + 8]  : mag[s + 9];
            double p3 = (mag[s + 11] > mag[s + 12]) ? mag[s + 11] : mag[s + 12];

            if (p0 < thr || p1 < thr || p2 < thr || p3 < thr)
                continue;

            /* Extra: pulse average must be well above noise */
            double pulse_avg = (p0 + p1 + p2 + p3) / 4.0;
            if (pulse_avg < noise * 2.5)
                continue;

            preamble_count++;

            /* 6. Demodulate 112 bits via PPM at 2.5 MSPS
             *
             * Each bit occupies 2.5 samples starting at DATA_START.
             * Bit b starts at sample: DATA_START + floor(b * 5 / 2)
             *
             * PPM decode: compare first half vs second half.
             *   Even bits (b%2==0): bit starts at integer sample
             *     first = mag[si], second = mag[si+1]
             *   Odd bits (b%2==1): bit starts at half-sample boundary
             *     first = max(mag[si], mag[si+1]), second = mag[si+2]
             */
            uint8_t frame[ADSB_BYTES];
            memset(frame, 0, sizeof(frame));

            for (int b = 0; b < ADSB_BITS; b++) {
                int si = s + DATA_START + (b * 5) / 2;
                double first, second;

                if (b % 2 == 0) {
                    first  = mag[si];
                    second = mag[si + 1];
                } else {
                    first  = (mag[si] > mag[si + 1]) ? mag[si] : mag[si + 1];
                    second = mag[si + 2];
                }

                int bit = (first > second) ? 1 : 0;
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

            s += FRAME_LEN - 1;
        }
    }

    free(mag);
    iio_buffer_destroy(rxbuf);
    iio_context_destroy(ctx);
    return 0;
}



// TODO: min sampling rate 2 MSPS, max sampling rate 10MSPS. 
// TODO: the interface must be adaptive