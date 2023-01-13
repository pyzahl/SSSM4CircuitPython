# SSSM4CircuitPython
Solar Scintillation Seeing Monitor for CircuitPython

Project based on hardware described here
https://www.blackwaterskies.co.uk/2017/06/diy-solar-scintillation-seeing-monitor-sssm/
or originally here:
https://vdocuments.mx/an-inexpensive-solar-scintillation-seeing-monitor-circuit-an-inexpensive.html

This project is based on CircuitPython Version 8 (all required Libs included for reference) running on a Adafruit ESP32-S2-TFT.




--- STARTUP ---

Wi-Fi: off | code.py | 8.0.0-beta.6
Wi-Fi: No IP | code.py | 8.0.0-beta.6
Note: DIP ON (=GND) := False
Dip SW 1 True No Logfile
Dip SW 2 False MQTT IoT Transmit
Dip SW 3 False FC Print
Dip SW 4 False WiFi
Battery Percent: 57.40 %
Battery Voltage: 3.87 V
Connecting WiFi...
Connecting to 31A138_5G
try: NTP clock sync...
try: Sync system time to NTP...
Start Timestamp: 2023 01 13  06:51:27
MQTT Client Configuration:
MQTT: Attempting to connect to 192.168.0.20::1883
Connected to MQTT Broker!
Flags: 0
 RC: 0
Checking for LUX sensor TSL2591
No I2C device at address: 0x29
... none
A0: 1.01
A1: 0.27
C2: 0.03
MQTT publishing...
A0: 1.01
A1: 0.25
C2: 0.05
MQTT publishing...
A0: 1.01
A1: 0.26
C2: 0.08
MQTT publishing...
A0: 1.01
A1: 0.27
C2: 0.10
MQTT publishing...
A0: 1.01
A1: 0.27
C2: 0.13
MQTT publishing...
A0: 1.01
A1: 0.27
C2: 0.16
