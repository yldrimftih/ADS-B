#!/usr/bin/env python3
"""
generate_iq_test.py — Generate synthetic ADS-B I/Q test data for adsb_receiver.c

Produces a raw binary file of interleaved int16 I/Q samples at 2 MSPS,
exactly matching the ADALM-PLUTO's output format. The C code can read
this file instead of connecting to the SDR hardware.

Usage:
    python3 generate_iq_test.py
    -> Creates test_adsb_iq.bin

Then compile and run the test harness:
    gcc -o test_adsb test_adsb_receiver.c -lm
    ./test_adsb test_adsb_iq.bin
"""

import struct
import random
import math

# ---- Configuration ----
SAMPLE_RATE = 2_000_000       # 2 MSPS
NOISE_AMPLITUDE = 30          # Background noise (int16 scale)
SIGNAL_AMPLITUDE = 2000       # Pulse amplitude (strong enough vs 3x threshold)
GAP_SAMPLES = 500             # Silence between injected packets

# ---- Known good ADS-B hex messages (CRC-verified) ----
TEST_MESSAGES = [
    # (hex_string, description)
    ("8D4840D6202CC371C32CE0576098", "Callsign KLM1023 (TC=4)"),
    ("8D40621D58C382D690C8AC2863A7", "Position even 40621D @ 38000ft (TC=11)"),
    ("8D40621D58C386435CC412692AD6", "Position odd 40621D (TC=11)"),
    ("8D485020994409940838175B284F", "Velocity 485020 GS=159kt (TC=19)"),
    ("8DA05F219B06B6AF189400CBC33F", "Velocity A05F21 TAS=376kt (TC=19)"),
    ("8DABA1A023101331C38D205B04E5", "Callsign DAL1084 (TC=4)"),
    ("8DABA1A0990C950B509C042E09F3", "Velocity ABA1A0 GS=173kt (TC=19)"),
    ("8DA48E3A5831D5652AB9D932A61B", "Position A48E3A @ 8925ft (TC=11)"),
]


def hex_to_bits(hex_str):
    """Convert hex string to list of 0/1 bits."""
    return [int(b) for b in bin(int(hex_str, 16))[2:].zfill(len(hex_str) * 4)]


def generate_adsb_baseband(hex_msg, amplitude):
    """
    Generate baseband magnitude samples for one ADS-B frame at 2 MSPS.

    ADS-B structure:
      - 8 us preamble (16 samples)
      - 112 us data (224 samples) for long squitter

    Preamble pattern (1 us pulses at 0, 1, 3.5, 4.5 us):
      At 2 MSPS → samples 0,1  2,3  7,8  9,10 are HIGH
      (each pulse is 0.5 us on, but at 2 MSPS that's 1 sample per chip)
      Preamble chips: [1,0,1,0,0,0,0,1,0,1,0,0,0,0,0,0]

    PPM data: each bit = 2 samples
      bit=1 → [HIGH, LOW]
      bit=0 → [LOW, HIGH]
    """
    # Preamble: 16 chips at 0.5 us each = 8 us = 16 samples
    # Pulses at 0.0us, 1.0us, 3.5us, 4.5us
    # -> sample indices 0, 2, 7, 9 are HIGH
    preamble = [0] * 16
    preamble[0] = 1
    preamble[2] = 1
    preamble[7] = 1
    preamble[9] = 1

    # Data bits -> PPM samples
    bits = hex_to_bits(hex_msg)
    data_samples = []
    for bit in bits:
        if bit == 1:
            data_samples.extend([1, 0])  # HIGH then LOW
        else:
            data_samples.extend([0, 1])  # LOW then HIGH

    # Combine
    all_chips = preamble + data_samples

    # Convert to amplitude levels
    samples = []
    for chip in all_chips:
        if chip:
            mag = amplitude
        else:
            mag = 0
        # Convert magnitude to I/Q: put all energy in I channel for simplicity
        # Add slight Q component to make it realistic
        angle = random.uniform(0, 2 * math.pi)
        i_val = int(mag * math.cos(angle))
        q_val = int(mag * math.sin(angle))
        samples.append((i_val, q_val))

    return samples


def add_noise(iq_pair, noise_amp):
    """Add Gaussian noise to an I/Q pair."""
    i, q = iq_pair
    i += int(random.gauss(0, noise_amp))
    q += int(random.gauss(0, noise_amp))
    # Clamp to int16 range
    i = max(-32768, min(32767, i))
    q = max(-32768, min(32767, q))
    return (i, q)


def generate_noise_samples(count, noise_amp):
    """Generate pure noise I/Q samples."""
    samples = []
    for _ in range(count):
        i = int(random.gauss(0, noise_amp))
        q = int(random.gauss(0, noise_amp))
        i = max(-32768, min(32767, i))
        q = max(-32768, min(32767, q))
        samples.append((i, q))
    return samples


def main():
    random.seed(42)  # Reproducible output

    all_samples = []

    # Leading noise (simulate receiver running before first packet)
    all_samples.extend(generate_noise_samples(1000, NOISE_AMPLITUDE))

    for hex_msg, description in TEST_MESSAGES:
        print(f"Encoding: {description}")
        print(f"  Hex:  {hex_msg}")
        print(f"  Bits: {len(hex_msg) * 4}")

        # Generate clean baseband
        frame_iq = generate_adsb_baseband(hex_msg, SIGNAL_AMPLITUDE)

        # Add noise to every sample
        noisy_frame = [add_noise(s, NOISE_AMPLITUDE) for s in frame_iq]

        all_samples.extend(noisy_frame)

        # Gap between frames (noise only)
        all_samples.extend(generate_noise_samples(GAP_SAMPLES, NOISE_AMPLITUDE))

    # Trailing noise
    all_samples.extend(generate_noise_samples(1000, NOISE_AMPLITUDE))

    # Write as raw int16 interleaved I/Q (same format as PlutoSDR)
    output_file = "test_adsb_iq.bin"
    with open(output_file, "wb") as f:
        for i_val, q_val in all_samples:
            f.write(struct.pack("<hh", i_val, q_val))

    total_bytes = len(all_samples) * 4
    duration_ms = len(all_samples) / SAMPLE_RATE * 1000

    print(f"\nGenerated: {output_file}")
    print(f"  Samples:  {len(all_samples)}")
    print(f"  Size:     {total_bytes} bytes ({total_bytes/1024:.1f} KB)")
    print(f"  Duration: {duration_ms:.1f} ms at 2 MSPS")
    print(f"  Format:   int16 interleaved I/Q (little-endian)")
    print(f"  Messages: {len(TEST_MESSAGES)}")
    print(f"\nExpected output from test_adsb:")
    for hex_msg, desc in TEST_MESSAGES:
        print(f"  *{hex_msg.upper()};   <- {desc}")


if __name__ == "__main__":
    main()