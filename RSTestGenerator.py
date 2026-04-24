#!/usr/bin/env python3
"""
ADSB_SignalGen.py — Generate ADS-B signals via R&S SMBV100B

NO VISA REQUIRED — uses raw TCP socket (port 5025) for SCPI control.

Requirements:
    pip install numpy
    pip install RsWaveform   (optional, manual .wv creation as fallback)

Usage:
    python ADSB_SignalGen.py --ip 169.254.147.45
    python ADSB_SignalGen.py --ip 169.254.147.45 --power -30
    python ADSB_SignalGen.py --no-upload
"""

import numpy as np
import math
import struct
import argparse
import datetime
import os
import socket
import time

# ============================================================================
#  USER-DEFINED TEST SCENARIO
# ============================================================================
TEST_AIRCRAFT = [
    {
        "icao": "4BA8E3",
        "callsign": "MUT33AJ",
        "altitude": 33600,
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
        "callsign": "HACTOR",
        "altitude": 41000,
        "lat": 52.3676,
        "lon": 4.9041,
        "speed": 510,
        "heading": 185.0,
        "vr": 0,
    },
    {
        "icao": "C0FFEE",
        "callsign": "ASIL",
        "altitude": 2200,
        "lat": 50.0379,
        "lon": 8.5622,
        "speed": 160,
        "heading": 120.5,
        "vr": 2400,
    },
    {
        "icao": "DEADBE",
        "callsign": "GUMELI",
        "altitude": 5500,
        "lat": 48.8566,
        "lon": 2.3522,
        "speed": 420,
        "heading": 95.0,
        "vr": 1500,
    },
    {
        "icao": "CAFE01",
        "callsign": "UAL456",
        "altitude": 35000,
        "lat": 51.5074,
        "lon": -0.1278,
        "speed": 480,
        "heading": 270.0,
        "vr": 0,
    },
    {
        "icao": "BEEF02",
        "callsign": "BA789",
        "altitude": 25000,
        "lat": 45.4642,
        "lon": -73.5673,
        "speed": 420,
        "heading": 180.0,
        "vr": -800,
    },
    {
        "icao": "ACE123",
        "callsign": "LAX567",
        "altitude": 10000,
        "lat": 34.0522,
        "lon": -118.2437,
        "speed": 280,
        "heading": 0.0,
        "vr": 1000,
    },
]

# ============================================================================
#  WAVEFORM CONFIG
# ============================================================================
DEFAULT_ARB_CLOCK = 5_000_000  # 5 MSPS
GAP_US = 2000
RF_FREQ = 1090e6
DEFAULT_POWER = -30


# ============================================================================
#  CRC-24
# ============================================================================
def crc24(data_bytes):
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
    header_88 = ((df & 0x1F) << 83
               | (ca & 0x07) << 80
               | (icao_int & 0xFFFFFF) << 56
               | (me_56bits & 0xFFFFFFFFFFFFFF))
    data_bytes = header_88.to_bytes(11, 'big')
    pi = crc24(data_bytes)
    frame = data_bytes + pi.to_bytes(3, 'big')
    return frame.hex().upper()


# ============================================================================
#  ADS-B MESSAGE BUILDERS
# ============================================================================
CALLSIGN_CHARSET = " ABCDEFGHIJKLMNOPQRSTUVWXYZ                     0123456789      "

def build_identification_msg(icao_hex, callsign):
    icao_int = int(icao_hex, 16)
    cs = callsign.upper().ljust(8)[:8]
    tc, cat = 4, 0
    char_bits = 0
    for ch in cs:
        idx = CALLSIGN_CHARSET.find(ch)
        if idx < 0: idx = 0
        char_bits = (char_bits << 6) | (idx & 0x3F)
    me = ((tc & 0x1F) << 51) | ((cat & 0x07) << 48) | (char_bits & 0xFFFFFFFFFFFF)
    return build_frame_with_crc(17, 5, icao_int, me)

def cpr_nl_func(lat):
    if abs(lat) >= 87.0: return 1
    return int(math.floor(
        2.0 * math.pi / math.acos(
            1.0 - (1.0 - math.cos(math.pi / 30.0)) /
            (math.cos(math.pi / 180.0 * abs(lat)) ** 2))))

def cpr_encode(lat, lon, odd):
    NZ = 15
    d_lat = 360.0 / (4 * NZ - odd)
    lat_norm = lat % 360.0
    if lat_norm < 0: lat_norm += 360.0
    yz = int(math.floor(131072.0 * (lat_norm % d_lat) / d_lat + 0.5)) & 0x1FFFF
    rlat = d_lat * (yz / 131072.0 + int(lat_norm / d_lat))
    if rlat >= 270.0: rlat -= 360.0
    nl = cpr_nl_func(rlat)
    d_lon = 360.0 / max(nl - odd, 1)
    lon_norm = lon % 360.0
    if lon_norm < 0: lon_norm += 360.0
    xz = int(math.floor(131072.0 * (lon_norm % d_lon) / d_lon + 0.5)) & 0x1FFFF
    return yz, xz

def encode_altitude_12bit(alt_ft):
    n = (alt_ft + 1000) // 25
    high7 = (n >> 4) & 0x7F
    low4 = n & 0x0F
    return (high7 << 5) | (1 << 4) | low4

def build_position_msg(icao_hex, alt_ft, lat, lon, odd):
    icao_int = int(icao_hex, 16)
    alt_enc = encode_altitude_12bit(alt_ft)
    lat_cpr, lon_cpr = cpr_encode(lat, lon, odd)
    me = (11 << 51) | (alt_enc & 0xFFF) << 36 | (odd & 1) << 34 | (lat_cpr & 0x1FFFF) << 17 | (lon_cpr & 0x1FFFF)
    return build_frame_with_crc(17, 5, icao_int, me)

def build_velocity_msg(icao_hex, speed_kt, heading_deg, vr_ftmin):
    icao_int = int(icao_hex, 16)
    hdg_rad = math.radians(heading_deg)
    vx = speed_kt * math.sin(hdg_rad)
    vy = speed_kt * math.cos(hdg_rad)
    ew_sign = 1 if vx < 0 else 0
    ew_vel = min(int(abs(vx) + 0.5) + 1, 1023)
    ns_sign = 1 if vy < 0 else 0
    ns_vel = min(int(abs(vy) + 0.5) + 1, 1023)
    vr_sign = 1 if vr_ftmin < 0 else 0
    vr_enc = min(int(abs(vr_ftmin) / 64.0 + 0.5) + 1, 511)
    me = (19 << 51) | (1 << 48) | (ew_sign << 42) | (ew_vel << 32) | (ns_sign << 31) | (ns_vel << 21) | (vr_sign << 19) | (vr_enc << 10)
    return build_frame_with_crc(17, 5, icao_int, me)


# ============================================================================
#  ADS-B BASEBAND MODULATION
# ============================================================================

def hex_to_bits(hex_str):
    return [int(b) for b in bin(int(hex_str, 16))[2:].zfill(len(hex_str) * 4)]

def generate_adsb_pulse(hex_msg, arb_clock):
    """
    ADS-B preamble (ICAO Annex 10):
      4 pulses, each 1.0 us wide:
        0.0-1.0 us,  1.0-2.0 us,  3.5-4.5 us,  4.5-5.5 us
      In 0.5 us chips: [1,1, 1,1, 0,0,0, 1,1, 1,1, 0,0,0,0,0]
    Data: 112 bits PPM, bit=1 -> [1,0], bit=0 -> [0,1] (each chip 0.5 us)
    """
    samples_per_us = arb_clock / 1e6
    half_us = int(samples_per_us * 0.5)

    preamble_chips = [1,1, 1,1, 0,0,0, 1,1, 1,1, 0,0,0,0,0]

    bits = hex_to_bits(hex_msg)
    data_chips = []
    for bit in bits:
        data_chips.extend([1, 0] if bit else [0, 1])

    all_chips = preamble_chips + data_chips
    samples = np.zeros(len(all_chips) * half_us)
    for i, chip in enumerate(all_chips):
        if chip:
            samples[i * half_us : (i + 1) * half_us] = 1.0

    return samples.astype(np.complex128)

def generate_silence(duration_us, arb_clock):
    samples_per_us = arb_clock / 1e6
    return np.zeros(int(duration_us * samples_per_us), dtype=np.complex128)


# ============================================================================
#  .WV FILE CREATION
# ============================================================================

def create_wv_file(iq_data, clock_rate, filename):
    try:
        import RsWaveform
        wv = RsWaveform.RsWaveform()
        wv.data[0] = iq_data
        wv.meta[0].update({
            'type': 'SMU-WV',
            'copyright': 'ADS-B Test Generator',
            'date': datetime.datetime.now(),
            'clock': float(clock_rate),
            'comment': f'ADS-B test signal, {len(TEST_AIRCRAFT)} aircraft',
        })
        wv.save(filename)
        print(f"  Created {filename} using RsWaveform library")
        return True
    except ImportError:
        print("  RsWaveform not installed, creating .wv manually...")
        return create_wv_file_manual(iq_data, clock_rate, filename)
    except Exception as e:
        print(f"  RsWaveform error: {e}, falling back to manual...")
        return create_wv_file_manual(iq_data, clock_rate, filename)

def create_wv_file_manual(iq_data, clock_rate, filename):
    max_val = np.max(np.abs(iq_data))
    if max_val > 0:
        iq_norm = iq_data / max_val
    else:
        iq_norm = iq_data

    i_s = (np.real(iq_norm) * 32767.0).astype(np.int16)
    q_s = (np.imag(iq_norm) * 32767.0).astype(np.int16)

    n_samples = len(iq_data)
    interleaved = np.empty(n_samples * 2, dtype=np.int16)
    interleaved[0::2] = i_s
    interleaved[1::2] = q_s
    binary_data = interleaved.tobytes()

    date_str = datetime.datetime.now().strftime("%Y-%m-%d;%H:%M:%S")
    data_len = len(binary_data)
    data_len_str = str(data_len)
    digits = len(data_len_str)

    header  = "{TYPE: SMU-WV,0}\r\n"
    header += "{COMMENT: ADS-B Test Signal}\r\n"
    header += f"{{DATE: {date_str}}}\r\n"
    header += f"{{CLOCK: {clock_rate:.1f}}}\r\n"
    header += "{LEVEL OFFS: 0,0}\r\n"
    header += f"{{SAMPLES: {n_samples}}}\r\n"
    header += f"{{WAVEFORM-{digits}: #{digits}{data_len_str}"

    with open(filename, 'wb') as f:
        f.write(header.encode('ascii'))
        f.write(binary_data)
        f.write(b'}')

    print(f"  Created {filename} ({os.path.getsize(filename)} bytes, {n_samples} samples)")
    return True


# ============================================================================
#  RAW SCPI VIA TCP SOCKET (no VISA needed)
# ============================================================================

class ScpiSocket:
    """Raw TCP SCPI communication on port 5025."""

    def __init__(self, ip, port=5025, timeout=15):
        self.ip = ip
        self.port = port
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.settimeout(timeout)
        self.sock.connect((ip, port))

    def write(self, cmd):
        """Send a SCPI command (text)."""
        if isinstance(cmd, str):
            cmd = cmd.encode('ascii')
        if not cmd.endswith(b'\n'):
            cmd += b'\n'
        self.sock.sendall(cmd)

    def read(self, bufsize=4096):
        """Read response."""
        data = b''
        try:
            while True:
                chunk = self.sock.recv(bufsize)
                if not chunk:
                    break
                data += chunk
                if data.endswith(b'\n'):
                    break
        except socket.timeout:
            pass
        return data.decode('ascii', errors='replace').strip()

    def query(self, cmd):
        """Send command and return response."""
        self.write(cmd)
        return self.read()

    def write_binary_block(self, cmd_header, binary_data):
        """
        Send SCPI command with IEEE 488.2 definite-length binary block.
        Format: COMMAND #<digits><length><binary_data>\n
        """
        data_len = len(binary_data)
        data_len_str = str(data_len)
        digits = len(data_len_str)
        block_header = f"{cmd_header}#{digits}{data_len_str}"
        payload = block_header.encode('ascii') + binary_data + b'\n'

        # Send in chunks to avoid socket buffer issues
        chunk_size = 1024 * 256
        sent = 0
        while sent < len(payload):
            end = min(sent + chunk_size, len(payload))
            self.sock.sendall(payload[sent:end])
            sent = end

    def opc(self):
        """Wait for operation complete."""
        return self.query('*OPC?')

    def close(self):
        self.sock.close()


# ============================================================================
#  INSTRUMENT CONTROL
# ============================================================================

def upload_and_play(ip, wv_filename, power_dbm, loop=True):
    """Upload waveform to SMBV100B and start RF output via raw socket."""

    print(f"\n  Connecting to {ip}:5025 (raw SCPI socket)...")

    try:
        scpi = ScpiSocket(ip)
    except Exception as e:
        print(f"  Connection failed: {e}")
        print(f"  Check: ping {ip}")
        return False

    # Identify
    idn = scpi.query('*IDN?')
    print(f"  Connected: {idn}")

    # Reset
    print("  Resetting instrument...")
    scpi.write('*RST')
    scpi.write('*CLS')
    time.sleep(2)
    scpi.opc()

    # Read waveform file
    with open(wv_filename, 'rb') as f:
        wv_data = f.read()

    # Upload to instrument filesystem
    instr_path = '/var/user/adsb_test.wv'
    print(f"  Uploading {wv_filename} ({len(wv_data)} bytes)...")
    scpi.write_binary_block(f"MMEM:DATA '{instr_path}',", wv_data)
    time.sleep(1)
    scpi.opc()
    print("  Upload complete.")

    # Verify file exists
    file_list = scpi.query("MMEM:CAT? '/var/user/'")
    if 'adsb_test.wv' in file_list:
        print("  File verified on instrument.")
    else:
        print(f"  WARNING: File not found in catalog. Trying anyway...")

    # Select waveform in ARB
    print("  Configuring ARB...")
    scpi.write(f"SOUR:BB:ARB:WAV:SEL '{instr_path}'")
    time.sleep(1)
    scpi.opc()

    # Trigger mode
    if loop:
        scpi.write("SOUR:BB:ARB:TRIG:SOUR AUTO")
    else:
        scpi.write("SOUR:BB:ARB:TRIG:SOUR SING")

    # Enable ARB
    scpi.write("SOUR:BB:ARB:STAT ON")
    time.sleep(1)
    scpi.opc()

    # Verify ARB state
    arb_state = scpi.query("SOUR:BB:ARB:STAT?")
    print(f"  ARB state: {arb_state}")

    # Configure RF output
    scpi.write(f"SOUR:FREQ:CW {RF_FREQ:.0f}")
    scpi.write(f"SOUR:POW:LEV:IMM:AMPL {power_dbm}")
    scpi.opc()

    # Read back
    actual_freq = scpi.query("SOUR:FREQ:CW?")
    actual_pow = scpi.query("SOUR:POW:LEV:IMM:AMPL?")
    print(f"  RF freq:  {float(actual_freq)/1e6:.1f} MHz")
    print(f"  RF power: {actual_pow} dBm")

    # RF ON
    scpi.write("OUTP:STAT ON")
    scpi.opc()

    outp_state = scpi.query("OUTP:STAT?")
    print(f"  RF output: {'ON' if '1' in outp_state else 'OFF'}")

    # Error check
    errors = scpi.query("SYST:ERR:ALL?")
    if '0,"No error"' in errors or errors.startswith('0,'):
        print("  No instrument errors.")
    else:
        print(f"  INSTRUMENT ERRORS: {errors}")

    print(f"\n  >>> SMBV100B transmitting ADS-B on 1090 MHz @ {power_dbm} dBm <<<")
    print(f"  >>> Mode: {'Continuous loop' if loop else 'Single burst'} <<<")

    scpi.close()
    return True


# ============================================================================
#  MAIN
# ============================================================================

def main():
    parser = argparse.ArgumentParser(description="ADS-B Signal Generator for R&S SMBV100B")
    parser.add_argument("--ip", default="169.254.147.45", help="SMBV100B IP address")
    parser.add_argument("--sr", type=float, default=10.0, help="Sampling rate in MSPS (2-10, default: 10)")
    parser.add_argument("--power", type=float, default=DEFAULT_POWER, help=f"RF power dBm (default: {DEFAULT_POWER})")
    parser.add_argument("--loop", action="store_true", default=True, help="Continuous loop")
    parser.add_argument("--once", action="store_true", help="Single burst")
    parser.add_argument("--no-upload", action="store_true", help="Generate .wv file only")
    args = parser.parse_args()

    # Validate sampling rate
    if args.sr < 2.0 or args.sr > 10.0:
        print(f"Error: Sampling rate must be 2-10 MSPS, got {args.sr}")
        return
    
    arb_clock = int(args.sr * 1_000_000)

    if args.once:
        args.loop = False

    print("=" * 65)
    print("  ADS-B Signal Generator — R&S SMBV100B")
    print(f"  Sampling Rate: {args.sr} MSPS ({arb_clock} Hz)")
    print("=" * 65)

    # Step 1: Build messages
    hex_messages = []
    for ac in TEST_AIRCRAFT:
        icao = ac["icao"].upper()
        cs = ac.get("callsign")
        alt = ac.get("altitude")
        lat = ac.get("lat")
        lon = ac.get("lon")
        spd = ac.get("speed")
        hdg = ac.get("heading")
        vr = ac.get("vr", 0)

        if cs:
            hex_messages.append((build_identification_msg(icao, cs), f"{icao} Callsign: {cs}"))
        if alt is not None and lat is not None and lon is not None:
            hex_messages.append((build_position_msg(icao, alt, lat, lon, 0), f"{icao} Pos EVEN @ {alt} ft"))
            hex_messages.append((build_position_msg(icao, alt, lat, lon, 1), f"{icao} Pos ODD"))
        if spd is not None and hdg is not None:
            hex_messages.append((build_velocity_msg(icao, spd, hdg, vr), f"{icao} Vel: {spd} kt"))

    print(f"\n  Aircraft: {len(TEST_AIRCRAFT)}")
    print(f"  Messages: {len(hex_messages)}")
    print(f"  ARB clock: {arb_clock/1e6:.1f} MSPS")
    print(f"  RF freq:   {RF_FREQ/1e6:.1f} MHz")
    print(f"  RF power:  {args.power} dBm\n")

    for i, (msg, desc) in enumerate(hex_messages):
        print(f"  [{i+1:2d}] {desc}")
        print(f"       *{msg};")

    # Step 2: Generate baseband
    print(f"\n  Generating baseband waveform...")
    all_iq = [generate_silence(100, arb_clock)]
    for hex_msg, _ in hex_messages:
        all_iq.append(generate_adsb_pulse(hex_msg, arb_clock))
        all_iq.append(generate_silence(GAP_US, arb_clock))
    all_iq.append(generate_silence(100, arb_clock))

    waveform = np.concatenate(all_iq)
    print(f"  Samples:  {len(waveform)}")
    print(f"  Duration: {len(waveform) / arb_clock * 1000:.1f} ms")

    # Step 3: Create .wv
    wv_file = f"adsb_test_{args.sr:.0f}MSPS.wv"
    print(f"\n  Creating {wv_file}...")
    create_wv_file(waveform, arb_clock, wv_file)

    # Step 4: Upload
    if args.no_upload:
        print(f"\n  File created: {wv_file}")
        print(f"  Transfer manually to SMBV100B -> Baseband > ARB > Select Waveform")
    else:
        upload_and_play(args.ip, wv_file, args.power, args.loop)

    print("\n  Done.")
    print("=" * 65)

if __name__ == "__main__":
    main()