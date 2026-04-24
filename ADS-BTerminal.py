import subprocess
import sys
import math
import os
import time

# ---------- CRC-24 ----------
CRC_POLY = 0x1FFF409

def crc24(hex_str):
    """CRC-24 over raw bytes; returns remainder."""
    data = bytes.fromhex(hex_str)
    crc = 0
    for byte in data[:-3]:
        crc ^= byte << 16
        for _ in range(8):
            crc <<= 1
            if crc & 0x1000000:
                crc ^= CRC_POLY
    crc &= 0xFFFFFF
    pi = int.from_bytes(data[-3:], 'big')
    return crc ^ pi


# ---------- ADS-B Character Set (6-bit index -> char) ----------
ADSB_CHARSET = (
    " ABCDEFGHIJKLMNOPQRSTUVWXYZ                     0123456789      "
)


# ---------- CPR Position Decoding ----------
NZ = 15

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
    """Globally unambiguous CPR decode."""
    d_lat0 = 360.0 / (4 * NZ)
    d_lat1 = 360.0 / (4 * NZ - 1)

    j = int(math.floor(59.0 * lat0 - 60.0 * lat1 + 0.5))

    lat_even = d_lat0 * ((j % 60) + lat0)
    lat_odd  = d_lat1 * ((j % 59) + lat1)

    if lat_even >= 270.0:
        lat_even -= 360.0
    if lat_odd >= 270.0:
        lat_odd -= 360.0

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


# ---------- Aircraft State Tracker ----------

aircraft_db = {}
# Key: ICAO (str)
# Value: {
#   "callsign": str or None,
#   "altitude": int or None,
#   "lat": float or None,
#   "lon": float or None,
#   "speed": float or None,
#   "heading": float or None,
#   "vr": int or None,
#   "cpr_even": (raw_lat, raw_lon, counter) or None,
#   "cpr_odd":  (raw_lat, raw_lon, counter) or None,
#   "last_seen": float (time.time()),
#   "msg_count": int,
# }

msg_counter = 0
sampling_rate = None


def get_aircraft(icao):
    """Get or create aircraft entry."""
    if icao not in aircraft_db:
        aircraft_db[icao] = {
            "callsign": None,
            "altitude": None,
            "lat": None,
            "lon": None,
            "speed": None,
            "heading": None,
            "vr": None,
            "cpr_even": None,
            "cpr_odd": None,
            "last_seen": time.time(),
            "msg_count": 0,
        }
    ac = aircraft_db[icao]
    ac["last_seen"] = time.time()
    ac["msg_count"] += 1
    return ac


def decode_callsign(me_bits):
    """Decode aircraft identification (TC 1-4)."""
    callsign = ""
    for i in range(8):
        idx = int(me_bits[8 + i * 6 : 14 + i * 6], 2)
        if 0 <= idx < len(ADSB_CHARSET):
            callsign += ADSB_CHARSET[idx]
    return callsign.strip()


def decode_position(icao, me_bits, ac):
    """Decode airborne position (TC 9-18). Updates aircraft state."""
    global msg_counter

    # Altitude
    q_bit = int(me_bits[15])
    if q_bit == 1:
        n = int(me_bits[8:15] + me_bits[16:20], 2)
        ac["altitude"] = n * 25 - 1000

    # CPR encoded lat/lon
    cpr_flag = int(me_bits[21])  # 0 = even, 1 = odd
    raw_lat = int(me_bits[22:39], 2) / 131072.0
    raw_lon = int(me_bits[39:56], 2) / 131072.0

    if cpr_flag:
        ac["cpr_odd"] = (raw_lat, raw_lon, msg_counter)
    else:
        ac["cpr_even"] = (raw_lat, raw_lon, msg_counter)

    # Try global decode if we have both frames
    if ac["cpr_even"] and ac["cpr_odd"]:
        lat0, lon0, c0 = ac["cpr_even"]
        lat1, lon1, c1 = ac["cpr_odd"]
        t = 0 if c0 > c1 else 1
        result = decode_cpr_global(lat0, lon0, lat1, lon1, t)
        if result:
            ac["lat"], ac["lon"] = result


def decode_velocity(me_bits, ac):
    """Decode airborne velocity (TC 19). Updates aircraft state."""
    subtype = int(me_bits[5:8], 2)

    if subtype in (1, 2):
        ew_sign = int(me_bits[13])
        ew_vel  = int(me_bits[14:24], 2) - 1
        ns_sign = int(me_bits[24])
        ns_vel  = int(me_bits[25:35], 2) - 1

        if subtype == 2:
            ew_vel *= 4
            ns_vel *= 4

        vx = -ew_vel if ew_sign else ew_vel
        vy = -ns_vel if ns_sign else ns_vel

        ac["speed"] = math.sqrt(vx ** 2 + vy ** 2)
        ac["heading"] = (math.degrees(math.atan2(vx, vy)) + 360) % 360

        # Vertical rate
        vr_sign = int(me_bits[36])
        vr_val  = (int(me_bits[37:46], 2) - 1) * 64
        ac["vr"] = -vr_val if vr_sign else vr_val

    elif subtype in (3, 4):
        hdg_avail = int(me_bits[13])
        if hdg_avail:
            ac["heading"] = int(me_bits[14:24], 2) * 360.0 / 1024.0

        airspeed = int(me_bits[25:35], 2)
        if subtype == 4:
            airspeed *= 4
        ac["speed"] = float(airspeed)


# ---------- Display ----------

STALE_TIMEOUT = 60  # Remove aircraft not seen for 60 seconds

def print_table():
    """Print the aircraft table to terminal."""
    now = time.time()

    # Remove stale entries
    stale = [k for k, v in aircraft_db.items() if now - v["last_seen"] > STALE_TIMEOUT]
    for k in stale:
        del aircraft_db[k]

    # Sort by ICAO
    sorted_ac = sorted(aircraft_db.items())

    # Clear screen
    if os.name == "nt":
        os.system("cls")
    else:
        print("\033[2J\033[H", end="")

    # Header
    print("=" * 105)
    sr_str = f" @ {sampling_rate}" if sampling_rate else ""
    print(f"  ADS-B RECEIVER — ADALM-PLUTO{sr_str}")
    print(f"  Tracking: {len(sorted_ac)} aircraft | {time.strftime('%H:%M:%S')}")
    print("=" * 105)
    print(f"  {'ICAO':<8} {'Callsign':<10} {'Speed':>7} {'Alt (ft)':>9} "
          f"{'Lat':>10} {'Lon':>11} {'VR(ft/m)':>9} {'Hdg':>7}")
    print("-" * 105)

    if not sorted_ac:
        print("  Waiting for aircraft...")
    else:
        for icao, ac in sorted_ac:
            cs  = ac["callsign"] or "------"
            spd = f"{ac['speed']:.0f} kt" if ac["speed"] is not None else "---"
            alt = f"{ac['altitude']}" if ac["altitude"] is not None else "---"
            lat = f"{ac['lat']:.5f}" if ac["lat"] is not None else "---"
            lon = f"{ac['lon']:.5f}" if ac["lon"] is not None else "---"
            vr  = f"{ac['vr']:+d}" if ac["vr"] is not None else "---"
            hdg = f"{ac['heading']:.1f}" + chr(176) if ac["heading"] is not None else "---"

            print(f"  {icao:<8} {cs:<10} {spd:>7} {alt:>9} "
                  f"{lat:>10} {lon:>11} {vr:>9} {hdg:>7}")

    print("-" * 105)
    print("  Ctrl+C to quit")


# ---------- Main Decoder ----------

last_display_time = 0
DISPLAY_INTERVAL = 0.5  # Refresh table every 0.5 seconds

def decode_adsb(hex_str):
    """Parses a validated hex frame and updates aircraft state."""
    global msg_counter, last_display_time
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
        if crc24(hex_str) != 0:
            return

        icao = hex_str[2:8].upper()
        me_bits = bin_str[32:88]
        tc = int(me_bits[0:5], 2)

        ac = get_aircraft(icao)

        if 1 <= tc <= 4:
            ac["callsign"] = decode_callsign(me_bits)

        elif 9 <= tc <= 18:
            decode_position(icao, me_bits, ac)

        elif tc == 19:
            decode_velocity(me_bits, ac)

    # DF 11 = All-Call Reply (short squitter, 56 bits)
    elif df == 11 and n_bits == 14:
        icao = hex_str[2:8].upper()
        get_aircraft(icao)  # Register ICAO even if no data yet

    # Refresh display periodically
    now = time.time()
    if now - last_display_time >= DISPLAY_INTERVAL:
        print_table()
        last_display_time = now


def main():
    if os.name == "nt":
        exe_name = "AirInterface.exe"
    else:
        exe_name = "./AirInterface"

    try:
        process = subprocess.Popen(
            [exe_name],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
            bufsize=1,
        )
    except FileNotFoundError:
        print(f"Error: {exe_name} not found. Compile the C code first.")
        return

    try:
        for line in iter(process.stdout.readline, ''):
            raw = line.strip()

            # Capture sampling rate from C program
            if raw.startswith('SAMPLING_RATE:'):
                global sampling_rate
                parts = raw.split('(')
                if len(parts) > 1:
                    sampling_rate = parts[1].split(')')[0].strip()  # Extract "2.5 MSPS"
                continue

            if raw.startswith('*') and raw.endswith(';'):
                raw = raw[1:-1]

            if not all(c in '0123456789ABCDEFabcdef' for c in raw):
                continue

            decode_adsb(raw)

    except KeyboardInterrupt:
        print("\nShutting down...")
        process.terminate()
        process.wait(timeout=3)


def run_tests():
    """Run test suite with tabular output."""
    global last_display_time, sampling_rate

    print("Feeding test messages...\n")
    sampling_rate = "Test Mode"

    test_msgs = [
        "8D4840D6202CC371C32CE0576098",   # KLM1023 callsign
        "8D40621D58C382D690C8AC2863A7",   # 40621D position even
        "8D40621D58C386435CC412692AD6",   # 40621D position odd -> lat/lon
        "8D485020994409940838175B284F",   # 485020 velocity
        "8DA05F219B06B6AF189400CBC33F",   # A05F21 velocity (TAS)
        "8DABA1A023101331C38D205B04E5",   # DAL1084 callsign
        "8DABA1A0990C950B509C042E09F3",   # ABA1A0 velocity
        "8DA48E3A5831D5652AB9D932A61B",   # A48E3A position
    ]

    for msg in test_msgs:
        last_display_time = 0  
        decode_adsb(msg)

    print_table()


def main_stdin():
    """Read hex frames from stdin (piped mode)."""
    global sampling_rate
    try:
        for line in sys.stdin:
            raw = line.strip()

            # Capture sampling rate from C program
            if raw.startswith('SAMPLING_RATE:'):
                parts = raw.split('(')
                if len(parts) > 1:
                    sampling_rate = parts[1].split(')')[0].strip()  # Extract "2.5 MSPS"
                continue

            if raw.startswith('*') and raw.endswith(';'):
                raw = raw[1:-1]

            if not raw:
                continue
            if not all(c in '0123456789ABCDEFabcdef' for c in raw):
                continue

            decode_adsb(raw)

        print_table()

    except KeyboardInterrupt:
        print("\nShutting down...")


if __name__ == "__main__":
    if len(sys.argv) > 1 and sys.argv[1] == "--test":
        run_tests()
    elif not sys.stdin.isatty():
        main_stdin()
    else:
        main()