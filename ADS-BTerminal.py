import subprocess
import sys
import math
import os

# ---------- CRC-24 (kept for standalone use / validation) ----------
CRC_POLY = 0x1FFF409

def crc24(hex_str):
    """CRC-24 over raw bytes; returns remainder."""
    data = bytes.fromhex(hex_str)
    crc = 0
    for byte in data[:-3]:  # everything except last 3 bytes (PI field)
        crc ^= byte << 16
        for _ in range(8):
            crc <<= 1
            if crc & 0x1000000:
                crc ^= CRC_POLY
    crc &= 0xFFFFFF
    pi = int.from_bytes(data[-3:], 'big')
    return crc ^ pi


# ---------- ADS-B Character Set (6-bit index → char) ----------
ADSB_CHARSET = (
    " ABCDEFGHIJKLMNOPQRSTUVWXYZ                     0123456789      "
)
#  0 = space, 1-26 = A-Z, 48-57 = 0-9, rest unused


# ---------- CPR Position Decoding ----------
NZ = 15  # Number of latitude zones for ADS-B

def cpr_nl(lat):
    """Number of longitude zones at a given latitude."""
    if abs(lat) >= 87.0:
        return 1
    return int(math.floor(
        2.0 * math.pi / math.acos(
            1.0 - (1.0 - math.cos(math.pi / (2.0 * NZ))) /
            (math.cos(math.pi / 180.0 * abs(lat)) ** 2)
        )
    ))


def decode_cpr_global(lat0, lon0, lat1, lon1, t):
    """
    Globally unambiguous CPR decode.
    (lat0, lon0) = even frame CPR values (0..1 normalized)
    (lat1, lon1) = odd frame CPR values  (0..1 normalized)
    t = 0 if most recent frame is even, 1 if odd.
    Returns (lat, lon) in degrees or None on failure.
    """
    d_lat0 = 360.0 / (4 * NZ)
    d_lat1 = 360.0 / (4 * NZ - 1)

    j = int(math.floor(59.0 * lat0 - 60.0 * lat1 + 0.5))

    lat_even = d_lat0 * ((j % 60) + lat0)
    lat_odd  = d_lat1 * ((j % 59) + lat1)

    if lat_even >= 270.0:
        lat_even -= 360.0
    if lat_odd >= 270.0:
        lat_odd -= 360.0

    # Check zone consistency
    if cpr_nl(lat_even) != cpr_nl(lat_odd):
        return None

    if t == 0:
        lat = lat_even
        nl = cpr_nl(lat)
        ni = max(nl, 1)
        m = int(math.floor(lon0 * (nl - 1) - lon1 * nl + 0.5))
        lon = (360.0 / ni) * ((m % ni) + lon0)
    else:
        lat = lat_odd
        nl = cpr_nl(lat)
        ni = max(nl - 1, 1)
        m = int(math.floor(lon0 * (nl - 1) - lon1 * nl + 0.5))
        lon = (360.0 / ni) * ((m % ni) + lon1)

    if lon >= 180.0:
        lon -= 360.0

    return lat, lon


# ---------- State: track even/odd CPR frames per ICAO ----------
cpr_cache = {}  # icao -> {"even": (lat, lon, tc), "odd": (lat, lon, tc), "counter": int}
msg_counter = 0


def decode_position(icao, me_bits, frame_counter):
    """Decode airborne position (TC 9-18). Returns string."""
    # Altitude
    q_bit = int(me_bits[15])
    if q_bit == 1:
        n = int(me_bits[8:15] + me_bits[16:20], 2)
        altitude = n * 25 - 1000
    else:
        altitude = None  # Gillham coded, rare

    # CPR encoded lat/lon
    cpr_flag = int(me_bits[21])  # 0 = even, 1 = odd
    raw_lat = int(me_bits[22:39], 2) / 131072.0
    raw_lon = int(me_bits[39:56], 2) / 131072.0

    # Cache this frame
    if icao not in cpr_cache:
        cpr_cache[icao] = {"even": None, "odd": None}

    key = "odd" if cpr_flag else "even"
    cpr_cache[icao][key] = (raw_lat, raw_lon, frame_counter)

    # Try global decode if we have both frames
    pos_str = ""
    entry = cpr_cache[icao]
    if entry["even"] and entry["odd"]:
        lat0, lon0, c0 = entry["even"]
        lat1, lon1, c1 = entry["odd"]

        # Use the more recent frame as reference
        t = 0 if c0 > c1 else 1
        result = decode_cpr_global(lat0, lon0, lat1, lon1, t)
        if result:
            lat, lon = result
            pos_str = f"Lat: {lat:.5f} Lon: {lon:.5f} | "

    alt_str = f"Alt: {altitude} ft" if altitude is not None else "Alt: (Gillham)"
    return pos_str + alt_str


def decode_velocity(me_bits):
    """Decode airborne velocity (TC 19, subtypes 1-2 = ground speed)."""
    subtype = int(me_bits[5:8], 2)

    if subtype in (1, 2):
        # Ground speed (normal / supersonic)
        ew_sign = int(me_bits[13])
        ew_vel  = int(me_bits[14:24], 2) - 1
        ns_sign = int(me_bits[24])
        ns_vel  = int(me_bits[25:35], 2) - 1

        if subtype == 2:  # supersonic: multiply by 4
            ew_vel *= 4
            ns_vel *= 4

        vx = -ew_vel if ew_sign else ew_vel
        vy = -ns_vel if ns_sign else ns_vel

        speed = math.sqrt(vx ** 2 + vy ** 2)
        heading = (math.degrees(math.atan2(vx, vy)) + 360) % 360

        # Vertical rate
        vr_sign = int(me_bits[36])
        vr_val  = (int(me_bits[37:46], 2) - 1) * 64
        vr = -vr_val if vr_sign else vr_val

        return f"GS: {speed:.0f} kt Hdg: {heading:.1f}° VR: {vr:+d} ft/min"

    elif subtype in (3, 4):
        # Airspeed (normal / supersonic)
        hdg_avail = int(me_bits[13])
        if hdg_avail:
            heading = int(me_bits[14:24], 2) * 360.0 / 1024.0
        else:
            heading = None

        as_type = "TAS" if int(me_bits[24]) else "IAS"
        airspeed = int(me_bits[25:35], 2)
        if subtype == 4:
            airspeed *= 4

        hdg_str = f"Hdg: {heading:.1f}°" if heading is not None else "Hdg: N/A"
        return f"{as_type}: {airspeed} kt {hdg_str}"

    return "Velocity (unknown subtype)"


def decode_callsign(me_bits):
    """Decode aircraft identification (TC 1-4)."""
    callsign = ""
    for i in range(8):
        idx = int(me_bits[8 + i * 6 : 14 + i * 6], 2)
        if 0 <= idx < len(ADSB_CHARSET):
            callsign += ADSB_CHARSET[idx]
    return callsign.strip()


# ---------- Main Decoder ----------

def decode_adsb(hex_str):
    """Parses a validated hex frame and prints decoded info."""
    global msg_counter
    msg_counter += 1

    n_hex = len(hex_str)
    if n_hex == 14:
        n_bits = 56
    elif n_hex == 28:
        n_bits = 112
    else:
        return

    bin_str = bin(int(hex_str, 16))[2:].zfill(n_bits)
    df = int(bin_str[0:5], 2)

    # DF 17/18 = Extended Squitter (112 bits)
    if df in (17, 18) and n_bits == 112:
        # Optional: verify CRC in Python too
        if crc24(hex_str) != 0:
            return

        icao = hex_str[2:8].upper()
        me_bits = bin_str[32:88]
        tc = int(me_bits[0:5], 2)

        output = f"[DF{df}] ICAO: {icao} | "

        if 1 <= tc <= 4:
            cs = decode_callsign(me_bits)
            output += f"Callsign: {cs}"
        elif 9 <= tc <= 18:
            output += decode_position(icao, me_bits, msg_counter)
        elif tc == 19:
            output += decode_velocity(me_bits)
        elif 20 <= tc <= 22:
            output += "Surface Position"
        else:
            output += f"TC: {tc}"

        print(output)

    # DF 11 = All-Call Reply (short squitter, 56 bits)
    elif df == 11 and n_bits == 14:
        icao = hex_str[2:8].upper()
        print(f"[DF11] ICAO: {icao} | All-Call Reply")


def main():
    # Determine executable name based on platform
    if os.name == "nt":
        exe_name = "pluto_adsb.exe"
    else:
        exe_name = "./pluto_adsb"

    print(f"Starting ADS-B decoder (C interface: {exe_name}) ...")

    try:
        process = subprocess.Popen(
            [exe_name],
            stdout=subprocess.PIPE,
            stderr=None,        # let C stderr print to terminal
            text=True,
            bufsize=1,          # line-buffered for real-time output
        )
    except FileNotFoundError:
        print(f"Error: {exe_name} not found. Compile the C code first.")
        return

    print("Listening on 1090 MHz ... (Ctrl+C to quit)")
    print("-" * 60)

    try:
        for line in iter(process.stdout.readline, ''):
            raw = line.strip()

            # Strip *...; wrapper from C output
            if raw.startswith('*') and raw.endswith(';'):
                raw = raw[1:-1]

            if not all(c in '0123456789ABCDEFabcdef' for c in raw):
                continue

            decode_adsb(raw)

    except KeyboardInterrupt:
        print("\nShutting down...")
        process.terminate()
        process.wait(timeout=3)


if __name__ == "__main__":
    main()
    #print("=" * 60)
    #print("ADS-B DECODER — TEST SUITE (all CRC-verified)")
    #print("=" * 60)
 
    ## Test 1: Callsign — KLM1023 (mode-s.org textbook example)
    ## ICAO: 4840D6 (Fokker 70, KLM Cityhopper)
    #print("\n[Test 1] Callsign — expected: ICAO 4840D6, KLM1023")
    #decode_adsb("8D4840D6202CC371C32CE0576098")
 
    ## Test 2: Airborne Position Even (TC=11) — altitude only (no pair yet)
    ## ICAO: 40621D, expected: 38000 ft
    #print("\n[Test 2] Position Even — expected: 40621D @ 38000 ft")
    #cpr_cache.clear(); msg_counter = 10
    #decode_adsb("8D40621D58C382D690C8AC2863A7")
 
    ## Test 3: Airborne Position Odd (TC=11) — CPR pair resolves lat/lon
    ## Expected: Lat ~52.266, Lon ~3.939 (over Netherlands)
    #print("\n[Test 3] Position Odd — expected: Lat ~52.27, Lon ~3.94")
    #decode_adsb("8D40621D58C386435CC412692AD6")
 
    ## Test 4: Velocity subtype 1 (ground speed)
    ## ICAO: 485020, expected: GS ~159 kt, Hdg ~182.9°, VR -832 ft/min
    #print("\n[Test 4] Velocity (GS) — expected: ~159 kt, ~183°")
    #decode_adsb("8D485020994409940838175B284F")
 
    ## Test 5: Velocity subtype 3 (airspeed + heading)
    ## ICAO: A05F21, expected: TAS 376 kt, Hdg ~244°
    #print("\n[Test 5] Velocity (TAS) — expected: TAS 376 kt, Hdg ~244°")
    #decode_adsb("8DA05F219B06B6AF189400CBC33F")
 
    ## Test 6: Corrupted packet — CRC must reject (last byte 98→99)
    #print("\n[Test 6] CRC rejection — corrupted (should be silent)")
    #decode_adsb("8D4840D6202CC371C32CE0576099")
 
    ## Test 7: Real callsign from ADSBexchange raw feed
    ## ICAO: ABA1A0, expected: DAL1084 (Delta Air Lines)
    #print("\n[Test 7] Callsign — expected: ABA1A0, DAL1084")
    #decode_adsb("8DABA1A023101331C38D205B04E5")
 
    ## Test 8: Real velocity from ADSBexchange
    ## ICAO: ABA1A0, expected: GS ~173 kt, Hdg ~301°, VR +2432 ft/min
    #print("\n[Test 8] Velocity — expected: ABA1A0, ~173 kt, ~301°")
    #decode_adsb("8DABA1A0990C950B509C042E09F3")
 
    ## Test 9: Real position from ADSBexchange
    ## ICAO: A48E3A, expected: altitude 8925 ft
    #print("\n[Test 9] Position — expected: A48E3A @ 8925 ft")
    #cpr_cache.clear(); msg_counter = 30
    #decode_adsb("8DA48E3A5831D5652AB9D932A61B")
 
    #print("\n" + "=" * 60)
    #print("Tests complete. Starting live capture...\n")