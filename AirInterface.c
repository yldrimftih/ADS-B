#include "iio.h"
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <stdint.h>
#include <string.h>

#define MHZ(x) ((long long)(x * 1000000.0 + .5))
#define BUFFER_SIZE (1024 * 512)

/* ADS-B at 2 MSPS:
 *   1 chip = 0.5 us = 1 sample
 *   1 bit  = 1.0 us = 2 samples
 *
 * Preamble pattern (chips): 1 1 0 0 0 0 1 0 0 1 0 0 0 0 1 1
 * Pulse starts at chips:    0,    4,       9,        14
 * At 2 MSPS each chip = 1 sample, so pulse peaks at samples 0, 4, 14, 18 — BUT
 * each pulse is 2 chips (1 us) wide, so we check the first sample of each pulse.
 *
 * Correction: The standard preamble is 8 us with pulses at 0, 1, 3.5, 4.5 us
 *   At 2 MSPS: samples 0, 2, 7, 9  (each pulse = 1 sample wide at 2 MSPS)
 *
 * Actually at 2 MSPS, 1 sample = 0.5 us.
 *   Pulse at 0.0 us -> sample 0
 *   Pulse at 1.0 us -> sample 2
 *   Pulse at 3.5 us -> sample 7
 *   Pulse at 4.5 us -> sample 9
 * Each pulse is 0.5 us = 1 sample wide.
 *
 * Data starts at 8.0 us -> sample 16.
 * Each data bit = 1.0 us = 2 samples (PPM: compare first vs second half).
 */

#define PREAMBLE_P0  0
#define PREAMBLE_P1  2
#define PREAMBLE_P2  7
#define PREAMBLE_P3  9

#define DATA_START   16
#define SAMPLES_PER_BIT 2
#define ADSB_BITS    112
#define ADSB_BYTES   14

/* Minimum samples needed from a candidate preamble start to end of a long frame */
#define FRAME_LEN (DATA_START + ADSB_BITS * SAMPLES_PER_BIT)

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

    /* RX LO — note: LO channels are OUTPUT (true) */
    struct iio_channel *rx_lo = iio_device_find_channel(phy, "altvoltage0", true);
    if (!rx_lo) {
        fprintf(stderr, "Error: Could not find RX LO channel.\n");
        iio_context_destroy(ctx);
        return -1;
    }
    iio_channel_attr_write_longlong(rx_lo, "frequency", MHZ(1090));

    /* RX baseband config */
    struct iio_channel *rx_cfg = iio_device_find_channel(phy, "voltage0", false);
    if (!rx_cfg) {
        fprintf(stderr, "Error: Could not find RX config channel.\n");
        iio_context_destroy(ctx);
        return -1;
    }
    iio_channel_attr_write_longlong(rx_cfg, "sampling_frequency", MHZ(2));
    iio_channel_attr_write_longlong(rx_cfg, "rf_bandwidth", MHZ(2));

    /* Set gain mode BEFORE writing gain value */
    iio_channel_attr_write(rx_cfg, "gain_control_mode", "manual");
    iio_channel_attr_write_double(rx_cfg, "hardwaregain", 58.0);

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

    fprintf(stderr, "ADS-B receiver running on 1090 MHz @ 2 MSPS ...\n");

    /* 4. Main receive loop */
    while (1) {
        ssize_t nbytes_rx = iio_buffer_refill(rxbuf);
        if (nbytes_rx < 0) {
            fprintf(stderr, "Warning: buffer refill error %zd\n", nbytes_rx);
            continue;
        }

        char *ptr;
        char *p_end = (char *)iio_buffer_end(rxbuf);
        ptrdiff_t step = iio_buffer_step(rxbuf);

        int sample_count = 0;

        /* Compute magnitude for every I/Q pair (avoid overflow with cast) */
        for (ptr = (char *)iio_buffer_first(rxbuf, rx_i);
             ptr < p_end && sample_count < BUFFER_SIZE;
             ptr += step, sample_count++) {
            int16_t iv = ((int16_t *)ptr)[0];
            int16_t qv = ((int16_t *)ptr)[1];
            mag[sample_count] = sqrt((double)iv * iv + (double)qv * qv);
        }

        /* 5. Scan for preambles */
        for (int s = 0; s <= sample_count - FRAME_LEN; s++) {

            /* Local noise estimate from gaps between preamble pulses */
            double noise = (mag[s + 1] + mag[s + 3] + mag[s + 4]
                          + mag[s + 5] + mag[s + 6] + mag[s + 8]) / 6.0;
            double thr = (noise < 1.0) ? 1.0 : noise * 3.0;

            /* Check four preamble pulses above threshold */
            if (mag[s + PREAMBLE_P0] < thr) continue;
            if (mag[s + PREAMBLE_P1] < thr) continue;
            if (mag[s + PREAMBLE_P2] < thr) continue;
            if (mag[s + PREAMBLE_P3] < thr) continue;

            /* 6. Demodulate 112 bits via PPM */
            uint8_t frame[ADSB_BYTES];
            memset(frame, 0, sizeof(frame));

            for (int b = 0; b < ADSB_BITS; b++) {
                int si = s + DATA_START + b * SAMPLES_PER_BIT;
                int bit = (mag[si] > mag[si + 1]) ? 1 : 0;
                frame[b / 8] = (frame[b / 8] << 1) | bit;
            }

            /* 7. Basic sanity: check Downlink Format (DF) */
            uint8_t df = (frame[0] >> 3) & 0x1F;

            int expected_bits;
            if (df == 0 || df == 4 || df == 5 || df == 11) {
                expected_bits = 56;   /* short squitter */
            } else if (df == 16 || df == 17 || df == 18 || df == 19 || df == 20 || df == 21) {
                expected_bits = 112;  /* long squitter / extended */
            } else {
                continue; /* unknown DF, skip */
            }

            int expected_bytes = expected_bits / 8;

            /* 8. CRC-24 check (last 3 bytes carry the PI/AP field) */
            uint32_t crc = crc24_compute(frame, expected_bytes - 3);
            uint32_t pi  = ((uint32_t)frame[expected_bytes - 3] << 16)
                         | ((uint32_t)frame[expected_bytes - 2] << 8)
                         |  (uint32_t)frame[expected_bytes - 1];

            if (df == 11 || df == 17 || df == 18) {
                /* For these DFs the PI field should equal the CRC remainder = 0 */
                if ((crc ^ pi) != 0)
                    continue;
            }
            /* For DF 0/4/5/16/20/21 the PI is XORed with the ICAO address,
               so we can't reject solely on CRC — pass them through. */

            /* 9. Output the validated hex frame */
            putchar('*');
            for (int j = 0; j < expected_bytes; j++)
                printf("%02X", frame[j]);
            printf(";\n");
            fflush(stdout);

            /* Skip past this frame to avoid re-triggering */
            s += DATA_START + expected_bits * SAMPLES_PER_BIT - 1;
        }
    }

    free(mag);
    iio_buffer_destroy(rxbuf);
    iio_context_destroy(ctx);
    return 0;
}