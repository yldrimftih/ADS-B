#!/usr/bin/env python3
"""
generate_iq_test.py — Generate synthetic ADS-B I/Q test data

Builds valid ADS-B hex frames from aircraft parameters (ICAO, callsign,
altitude, lat/lon, speed, heading, etc.), then modulates them into raw
int16 I/Q samples at 2 MSPS — the exact format the ADALM-PLUTO outputs.

Usage:
    python generate_iq_test.py              # creates test_adsb_iq.bin
    .\\test_adsb.exe test_adsb_iq.bin        # verify C demodulator
    .\\test_adsb.exe test_adsb_iq.bin 2>$null | python adsb_terminal.py  # full pipeline

Edit TEST_AIRCRAFT below to define your own scenario.
"""

import struct
import random
import math

# ============================================================================
#  USER-DEFINED TEST SCENARIO — Edit this array freely
# ============================================================================
# Each entry defines one aircraft. The generator will create the appropriate
# ADS-B message types for whatever fields you provide:
#
#   "icao"      : 6-char hex ICAO address (required)
#   "callsign"  : up to 8 chars, A-Z 0-9 space (optional -> TC=4 identification msg)
#   "altitude"  : barometric altitude in feet (optional -> TC=11 position msg)
#   "lat"       : latitude in degrees (optional, needs lon too)
#   "lon"       : longitude in degrees (optional, needs lat too)
#   "speed"     : ground speed in knots (optional -> TC=19 velocity msg)
#   "heading"   : track angle in degrees 0-360 (optional, needs speed too)
#   "vr"        : vertical rate in ft/min (optional, default 0)
#
# Position messages always emit an EVEN+ODD CPR pair so the decoder can
# resolve lat/lon. Provide lat+lon+altitude together for best results.

TEST_AIRCRAFT = [
    {
        "icao": "4BA8E3",
        "callsign": "THY6AJ",
        "altitude": 37000,
        "lat": 40.9800,
        "lon": 28.8200,
        "speed": 465,
        "heading": 270.0,
        "vr": 0,
    },
    {
        "icao": "4BB1C5",
        "callsign": "THY1923",
        "altitude": 12500,
        "lat": 41.2750,
        "lon": 28.7500,
        "speed": 280,
        "heading": 55.0,
        "vr": -1200,
    },
    {
        "icao": "A12345",
        "callsign": "UAL472",
        "altitude": 41000,
        "lat": 52.3676,
        "lon": 4.9041,
        "speed": 510,
        "heading": 185.0,
        "vr": 0,
    },
    {
        "icao": "C0FFEE",
        "callsign": "DLH9K",
        "altitude": 2200,
        "lat": 50.0379,
        "lon": 8.5622,
        "speed": 160,
        "heading": 120.5,
        "vr": 2400,
    },
    {
        # Minimal example: only callsign, no position or velocity
        "icao": "ABCDEF",
        "callsign": "TEST1",
    },
    {
        # Only position, no callsign
        "icao": "789ABC",
        "altitude": 28000,
        "lat": 35.1500,
        "lon": -90.0500,
    },
]


# ============================================================================
#  RF / MODULATION CONFIG
# ============================================================================
SAMPLE_RATE      = 2_000_000    # 2 MSPS
NOISE_AMPLITUDE  = 30           # Gaussian noise sigma (int16 scale)
SIGNAL_AMPLITUDE = 2000         # Pulse amplitude
GAP_SAMPLES      = 500          # Silence between packets
OUTPUT_FILE      = "TestADS-BIQ.bin"


# ============================================================================
#  CRC-24 ENCODER (polynomial 0x1FFF409)
# ============================================================================

def crc24(data_bytes):
    """Compute CRC-24 over bytes, return 3-byte remainder."""
    POLY = 0x1FFF409
    crc = 0
    for byte in data_bytes:
        crc ^= byte << 16
        for _ in range(8):
            crc <<= 1
            if crc & 0x1000000:
                crc ^= POLY
    return crc & 0xFFFFFF


def build_frame_with_crc(df, ca, icao_int, me_56bits):
    """
    Assemble a 112-bit ADS-B frame and append valid CRC-24.
    Returns 28-char hex string.

    df:         Downlink Format (5 bits, usually 17)
    ca:         Capability (3 bits, usually 5)
    icao_int:   24-bit ICAO address as integer
    me_56bits:  56-bit ME payload as integer
    """
    # First 88 bits: DF(5) + CA(3) + ICAO(24) + ME(56)
    header_88 = ((df & 0x1F) << 83
               | (ca & 0x07) << 80
               | (icao_int & 0xFFFFFF) << 56
               | (me_56bits & 0xFFFFFFFFFFFFFF))

    # Convert to 11 bytes
    data_bytes = header_88.to_bytes(11, 'big')

    # CRC over those 11 bytes
    pi = crc24(data_bytes)

    # Full 14-byte frame
    frame = data_bytes + pi.to_bytes(3, 'big')
    return frame.hex().upper()


# ============================================================================
#  ADS-B MESSAGE BUILDERS
# ============================================================================

CALLSIGN_CHARSET = " ABCDEFGHIJKLMNOPQRSTUVWXYZ                     0123456789      "


def build_identification_msg(icao_hex, callsign):
    """Build TC=4 Aircraft Identification message. Returns hex string."""
    icao_int = int(icao_hex, 16)

    # Pad callsign to 8 chars
    cs = callsign.upper().ljust(8)[:8]

    # TC=4 (category A0)
    tc = 4
    cat = 0

    # Encode 8 chars x 6 bits = 48 bits
    char_bits = 0
    for ch in cs:
        idx = CALLSIGN_CHARSET.find(ch)
        if idx < 0:
            idx = 0  # space for unknown chars
        char_bits = (char_bits << 6) | (idx & 0x3F)

    # ME = TC(5) + CA(3) + 48 bits of callsign = 56 bits
    me = ((tc & 0x1F) << 51) | ((cat & 0x07) << 48) | (char_bits & 0xFFFFFFFFFFFF)

    return build_frame_with_crc(17, 5, icao_int, me)


def encode_altitude_12bit(alt_ft):
    """Encode barometric altitude into 12-bit field (Q-bit = 1, 25ft resolution)."""
    n = (alt_ft + 1000) // 25
    # Layout: b11..b5 [Q=1] b3..b0
    high7 = (n >> 4) & 0x7F
    low4  = n & 0x0F
    return (high7 << 5) | (1 << 4) | low4


def cpr_nl_func(lat):
    """Number of longitude zones at given latitude."""
    if abs(lat) >= 87.0:
        return 1
    return int(math.floor(
        2.0 * math.pi / math.acos(
            1.0 - (1.0 - math.cos(math.pi / (2.0 * 15))) /
            (math.cos(math.pi / 180.0 * abs(lat)) ** 2)
        )
    ))


def cpr_encode(lat, lon, odd):
    """
    Encode lat/lon into 17-bit CPR format.
    odd: 0 for even frame, 1 for odd frame.
    Returns (lat_cpr_17bit, lon_cpr_17bit).
    """
    NZ = 15
    d_lat = 360.0 / (4 * NZ - odd)

    # Normalize latitude to [0, 360)
    lat_norm = lat % 360.0
    if lat_norm < 0:
        lat_norm += 360.0

    yz = int(math.floor(131072.0 * (lat_norm % d_lat) / d_lat + 0.5)) & 0x1FFFF

    # Compute NL for this latitude
    rlat = d_lat * (yz / 131072.0 + int(lat_norm / d_lat))
    if rlat >= 270.0:
        rlat -= 360.0

    nl = cpr_nl_func(rlat)
    d_lon = 360.0 / max(nl - odd, 1)

    lon_norm = lon % 360.0
    if lon_norm < 0:
        lon_norm += 360.0

    xz = int(math.floor(131072.0 * (lon_norm % d_lon) / d_lon + 0.5)) & 0x1FFFF

    return yz, xz


def build_position_msg(icao_hex, alt_ft, lat, lon, odd):
    """
    Build TC=11 Airborne Position message.
    odd: 0 for even CPR frame, 1 for odd CPR frame.
    Returns hex string.
    """
    icao_int = int(icao_hex, 16)

    tc = 11
    ss = 0      # surveillance status
    saf = 0     # single antenna flag
    t_flag = 0  # UTC synced

    alt_enc = encode_altitude_12bit(alt_ft)
    lat_cpr, lon_cpr = cpr_encode(lat, lon, odd)

    # ME: TC(5) + SS(2) + SAF(1) + ALT(12) + T(1) + F(1) + LAT(17) + LON(17) = 56 bits
    me = 0
    me |= (tc & 0x1F)          << 51
    me |= (ss & 0x03)          << 49
    me |= (saf & 0x01)         << 48
    me |= (alt_enc & 0xFFF)    << 36
    me |= (t_flag & 0x01)      << 35
    me |= (odd & 0x01)         << 34
    me |= (lat_cpr & 0x1FFFF)  << 17
    me |= (lon_cpr & 0x1FFFF)

    return build_frame_with_crc(17, 5, icao_int, me)


def build_velocity_msg(icao_hex, speed_kt, heading_deg, vr_ftmin):
    """
    Build TC=19 subtype 1 (ground speed) Airborne Velocity message.
    Returns hex string.
    """
    icao_int = int(icao_hex, 16)

    tc = 19
    subtype = 1

    # Decompose ground speed into E/W and N/S components
    hdg_rad = math.radians(heading_deg)
    vx = speed_kt * math.sin(hdg_rad)  # east-west
    vy = speed_kt * math.cos(hdg_rad)  # north-south

    ew_sign = 1 if vx < 0 else 0
    ew_vel = min(int(abs(vx) + 0.5) + 1, 1023)
    ns_sign = 1 if vy < 0 else 0
    ns_vel = min(int(abs(vy) + 0.5) + 1, 1023)

    # Vertical rate
    vr_sign = 1 if vr_ftmin < 0 else 0
    vr_enc = min(int(abs(vr_ftmin) / 64.0 + 0.5) + 1, 511)

    # ME: TC(5) + ST(3) + IC(1) + IFR(1) + NUCr(3) +
    #     EWs(1) + EWv(10) + NSs(1) + NSv(10) +
    #     VrSrc(1) + VrSign(1) + Vr(9) + Rsrvd(2) +
    #     DaltSign(1) + Dalt(7) = 56 bits
    me = 0
    me |= (tc & 0x1F)       << 51
    me |= (subtype & 0x07)  << 48
    me |= (0)               << 47   # intent change
    me |= (0)               << 46   # IFR capability
    me |= (0)               << 43   # NUCr
    me |= (ew_sign & 0x01)  << 42
    me |= (ew_vel & 0x3FF)  << 32
    me |= (ns_sign & 0x01)  << 31
    me |= (ns_vel & 0x3FF)  << 21
    me |= (0)               << 20   # VR source (baro)
    me |= (vr_sign & 0x01)  << 19
    me |= (vr_enc & 0x1FF)  << 10
    me |= (0)               << 8    # reserved
    me |= (0)               << 7    # dalt sign
    me |= (0)                        # dalt

    return build_frame_with_crc(17, 5, icao_int, me)


# ============================================================================
#  I/Q MODULATION
# ============================================================================

def hex_to_bits(hex_str):
    """Convert hex string to list of 0/1 bits."""
    return [int(b) for b in bin(int(hex_str, 16))[2:].zfill(len(hex_str) * 4)]


def generate_adsb_baseband(hex_msg, amplitude):
    """
    Generate baseband I/Q samples for one ADS-B frame at 2 MSPS.

    Preamble: pulses at samples 0, 2, 7, 9 (8 us = 16 samples)
    Data: PPM, each bit = 2 samples (bit=1 -> [HIGH,LOW], bit=0 -> [LOW,HIGH])
    """
    preamble = [0] * 16
    preamble[0] = 1
    preamble[2] = 1
    preamble[7] = 1
    preamble[9] = 1

    bits = hex_to_bits(hex_msg)
    data_samples = []
    for bit in bits:
        if bit == 1:
            data_samples.extend([1, 0])
        else:
            data_samples.extend([0, 1])

    all_chips = preamble + data_samples

    samples = []
    for chip in all_chips:
        mag = amplitude if chip else 0
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
    return (max(-32768, min(32767, i)), max(-32768, min(32767, q)))


def generate_noise_samples(count, noise_amp):
    """Generate pure noise I/Q samples."""
    return [
        (max(-32768, min(32767, int(random.gauss(0, noise_amp)))),
         max(-32768, min(32767, int(random.gauss(0, noise_amp)))))
        for _ in range(count)
    ]


# ============================================================================
#  MAIN
# ============================================================================

def main():
    random.seed(42)

    # Step 1: Build hex messages from aircraft definitions
    hex_messages = []  # list of (hex_str, description)

    for ac in TEST_AIRCRAFT:
        icao = ac["icao"].upper()
        callsign = ac.get("callsign")
        altitude = ac.get("altitude")
        lat = ac.get("lat")
        lon = ac.get("lon")
        speed = ac.get("speed")
        heading = ac.get("heading")
        vr = ac.get("vr", 0)

        # Identification message
        if callsign:
            msg = build_identification_msg(icao, callsign)
            hex_messages.append((msg, f"{icao} Callsign: {callsign}"))

        # Position messages (even + odd pair for CPR decode)
        if altitude is not None and lat is not None and lon is not None:
            msg_even = build_position_msg(icao, altitude, lat, lon, odd=0)
            msg_odd  = build_position_msg(icao, altitude, lat, lon, odd=1)
            hex_messages.append((msg_even, f"{icao} Position EVEN: {lat:.4f}, {lon:.4f} @ {altitude} ft"))
            hex_messages.append((msg_odd,  f"{icao} Position ODD:  {lat:.4f}, {lon:.4f} @ {altitude} ft"))
        elif altitude is not None:
            # Altitude only — single even frame
            msg = build_position_msg(icao, altitude, 0.0, 0.0, odd=0)
            hex_messages.append((msg, f"{icao} Altitude: {altitude} ft (no coords)"))

        # Velocity message
        if speed is not None and heading is not None:
            msg = build_velocity_msg(icao, speed, heading, vr)
            hex_messages.append((msg, f"{icao} Velocity: {speed} kt, {heading:.1f} deg, VR {vr:+d}"))

    # Step 2: Display summary
    print("=" * 65)
    print("  ADS-B Test I/Q Generator")
    print("=" * 65)
    print(f"\n  Aircraft defined:  {len(TEST_AIRCRAFT)}")
    print(f"  Messages to emit:  {len(hex_messages)}\n")

    for i, (msg, desc) in enumerate(hex_messages):
        print(f"  [{i+1:2d}] {desc}")
        print(f"       *{msg};")

    # Step 3: Modulate into I/Q samples
    all_samples = []
    all_samples.extend(generate_noise_samples(1000, NOISE_AMPLITUDE))

    for hex_msg, _ in hex_messages:
        frame_iq = generate_adsb_baseband(hex_msg, SIGNAL_AMPLITUDE)
        noisy_frame = [add_noise(s, NOISE_AMPLITUDE) for s in frame_iq]
        all_samples.extend(noisy_frame)
        all_samples.extend(generate_noise_samples(GAP_SAMPLES, NOISE_AMPLITUDE))

    all_samples.extend(generate_noise_samples(1000, NOISE_AMPLITUDE))

    # Step 4: Write binary file
    with open(OUTPUT_FILE, "wb") as f:
        for i_val, q_val in all_samples:
            f.write(struct.pack("<hh", i_val, q_val))

    total_bytes = len(all_samples) * 4
    duration_ms = len(all_samples) / SAMPLE_RATE * 1000

    print(f"\n  Output:    {OUTPUT_FILE}")
    print(f"  Samples:   {len(all_samples)}")
    print(f"  Size:      {total_bytes} bytes ({total_bytes/1024:.1f} KB)")
    print(f"  Duration:  {duration_ms:.1f} ms at 2 MSPS")
    print(f"\n  Test commands:")
    print(f"    .\\test_adsb.exe {OUTPUT_FILE}")
    print(f"    .\\test_adsb.exe {OUTPUT_FILE} 2>$null | python adsb_terminal.py")
    print()


if __name__ == "__main__":
    main()