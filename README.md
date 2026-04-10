# ADS-B 

* This project aims to monitor aircraft transponder data through a terminal interface.
* It receives raw I/Q data samples via the **ADALM-PLUTO** SDR operating at 1090 MHz.
* The received signal is processed to detect and decode ADS-B (Automatic Dependent Surveillance–Broadcast) messages.
* Extracted information includes aircraft identification (ICAO), callsign, speed, altitude, position, vertical rate, and heading.
* The decoded data is displayed in a real-time terminal-based interface for easy monitoring.

## System Block Diagram 

```text 
+------------+     +-----------------+     +-----------------+     +------------------+     +-----------------------+
| Rx Antenna | --> | Adalm-Pluto SDR | --> | AirInterface.c  | --> | ADS-BTerminal.py | --> | Decoded ADS-B Messages |
+------------+     +-----------------+     +-----------------+     +------------------+     +-----------------------+
```
 
## Run the System

1. To compile `AirInterface.c`, run this code: `gcc AirInterface.c -o AirInterface.exe -L. -liio -lm`, 
2. Run this command:
   1. For linux: `python3 ADS-BTerminal.py`
   2. For Windows: `python ADS-BTerminal.py`
3. To test the python script run `python ADS-BTerminal.py --test`

## The terminal output should look like 

```text
========================================================================================
  ADS-B RECEIVER — ADALM-PLUTO @ 1090 MHz
  Tracking: 6 aircraft | 21:45:03
========================================================================================
  ICAO     Callsign     Speed   Alt (ft)        Lat         Lon  VR(ft/m)     Hdg       
----------------------------------------------------------------------------------------
  40621D   ------         ---     38000     52.26578     3.93891       ---     ---      
  4840D6   KLM1023        ---       ---          ---         ---       ---     ---      
  485020   ------      159 kt       ---          ---         ---      -832   182.9°     
  A05F21   ------      376 kt       ---          ---         ---       ---   244.0°     
  A48E3A   ------         ---      8925          ---         ---       ---     ---      
  ABA1A0   DAL1084     173 kt       ---          ---         ---     +2432   301.0°     
----------------------------------------------------------------------------------------
```

## Test the System

1. You can generate custom *.bin* files using `GenerateIQTest.py`.  
2. After that, run this command:
   1. For linux: `python3 GenerateIQTest.py`
   2. For Windows: `python GenerateIQTest.py`
3. To verify if it's a valid ADS-B file: `.\TestAirInterface.exe TestADS-BIQ.bin`
4. To test the pipelined system: `.\TestAirInterface.exe TestADS-BIQ.bin 2>$null | python ADS-BTerminal.py`