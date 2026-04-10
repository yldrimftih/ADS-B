/*
 * TestAirInterface.c — Offline test harness for the ADS-B signal processing
 *
 * This is the same preamble detector, PPM demodulator, and CRC checker
 * from adsb_receiver.c, but reads I/Q samples from a binary file instead
 * of from the ADALM-PLUTO SDR hardware. No libiio dependency.
 *
 * Compile:
 *     gcc -o test_adsb TestAirInterface.c -lm
 *
 * Usage:
 *     python GenerateIQTest.py                # creates test_adsb_iq.bin
 *     ./TestAirInterface.exe TestADS-BIQ.bin  # run the test
 *
 * Expected: prints *HEX; for each injected ADS-B message.
 */

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <stdint.h>
#include <string.h>

#define PREAMBLE_P0  0
#define PREAMBLE_P1  2
#define PREAMBLE_P2  7
#define PREAMBLE_P3  9

#define DATA_START      16
#define SAMPLES_PER_BIT 2
#define ADSB_BITS       112
#define ADSB_BYTES      14

#define FRAME_LEN (DATA_START + ADSB_BITS * SAMPLES_PER_BIT)

/* ---------- CRC-24 (identical to adsb_receiver.c) ---------- */

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

/* ---------- Main ---------- */

int main(int argc, char *argv[]) {
    if (argc < 2) {
        fprintf(stderr, "Usage: %s <iq_file.bin>\n", argv[0]);
        fprintf(stderr, "  iq_file.bin: raw int16 interleaved I/Q at 2 MSPS\n");
        fprintf(stderr, "\nGenerate test data with:\n");
        fprintf(stderr, "  python3 generate_iq_test.py\n");
        return 1;
    }

    crc24_init();

    /* Open raw I/Q file */
    FILE *fp = fopen(argv[1], "rb");
    if (!fp) {
        fprintf(stderr, "Error: Cannot open '%s'\n", argv[1]);
        return 1;
    }

    /* Get file size to determine sample count */
    fseek(fp, 0, SEEK_END);
    long file_size = ftell(fp);
    fseek(fp, 0, SEEK_SET);

    /* Each sample = 4 bytes (int16 I + int16 Q) */
    int sample_count = (int)(file_size / 4);
    if (sample_count < FRAME_LEN) {
        fprintf(stderr, "Error: File too small (%ld bytes, need at least %d samples)\n",
                file_size, FRAME_LEN);
        fclose(fp);
        return 1;
    }

    fprintf(stderr, "Reading %d I/Q samples from '%s' (%.1f ms at 2 MSPS)\n",
            sample_count, argv[1], sample_count / 2000.0);

    /* Read all samples */
    int16_t *iq_raw = (int16_t *)malloc(sample_count * 2 * sizeof(int16_t));
    if (!iq_raw) {
        fprintf(stderr, "Error: malloc failed for I/Q buffer\n");
        fclose(fp);
        return 1;
    }
    fread(iq_raw, 4, sample_count, fp);
    fclose(fp);

    /* Compute magnitude*/
    double *mag = (double *)malloc(sample_count * sizeof(double));
    if (!mag) {
        fprintf(stderr, "Error: malloc failed for magnitude buffer\n");
        free(iq_raw);
        return 1;
    }

    for (int i = 0; i < sample_count; i++) {
        int16_t iv = iq_raw[i * 2];
        int16_t qv = iq_raw[i * 2 + 1];
        mag[i] = sqrt((double)iv * iv + (double)qv * qv);
    }

    free(iq_raw); 

    /* ---------- Preamble detection + demodulation ----------
     * This is IDENTICAL to the main loop in AirInterface.c
     */

    int detected = 0;

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

        /* Demodulate 112 bits via PPM */
        uint8_t frame[ADSB_BYTES];
        memset(frame, 0, sizeof(frame));

        for (int b = 0; b < ADSB_BITS; b++) {
            int si = s + DATA_START + b * SAMPLES_PER_BIT;
            int bit = (mag[si] > mag[si + 1]) ? 1 : 0;
            frame[b / 8] = (frame[b / 8] << 1) | bit;
        }

        /* Check Downlink Format */
        uint8_t df = (frame[0] >> 3) & 0x1F;

        int expected_bits;
        if (df == 0 || df == 4 || df == 5 || df == 11) {
            expected_bits = 56;
        } else if (df == 16 || df == 17 || df == 18 || df == 19 || df == 20 || df == 21) {
            expected_bits = 112;
        } else {
            continue;
        }

        int expected_bytes = expected_bits / 8;

        /* CRC-24 check */
        uint32_t crc = crc24_compute(frame, expected_bytes - 3);
        uint32_t pi  = ((uint32_t)frame[expected_bytes - 3] << 16)
                     | ((uint32_t)frame[expected_bytes - 2] << 8)
                     |  (uint32_t)frame[expected_bytes - 1];

        if (df == 11 || df == 17 || df == 18) {
            if ((crc ^ pi) != 0)
                continue;
        }

        /* Output validated frame */
        detected++;
        putchar('*');
        for (int j = 0; j < expected_bytes; j++)
            printf("%02X", frame[j]);
        printf(";\n");
        fflush(stdout);

        /* Skip past this frame */
        s += DATA_START + expected_bits * SAMPLES_PER_BIT - 1;
    }

    /* Summary */
    fprintf(stderr, "\n--- Results ---\n");
    fprintf(stderr, "Frames detected: %d\n", detected);
    if (detected > 0) {
        fprintf(stderr, "STATUS: PASS\n");
    } else {
        fprintf(stderr, "STATUS: FAIL (no frames detected)\n");
    }

    free(mag);
    return (detected > 0) ? 0 : 1;
}