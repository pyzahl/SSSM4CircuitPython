# SSSM4CircuitPython
Solar Scintillation Seeing Monitor (SSSM) for CircuitPython

A revised verion of a SSSM for CircuitPython on a ESP32-S2 or -S3-TFT.

Project based on hardware described here
https://www.blackwaterskies.co.uk/2017/06/diy-solar-scintillation-seeing-monitor-sssm/
or originally here:
https://vdocuments.mx/an-inexpensive-solar-scintillation-seeing-monitor-circuit-an-inexpensive.html

Let me copy the intro to SSSM:

A guide to building an inexpensive Solar Scintillation Seeing Monitor. The SSSM is a handy device which monitors the atmospheric seeing conditions during solar imaging. This SSSM can be built for a fraction of the cost of commercially available devices. You can use the SSSM to monitor seeing conditions through the day, or even to control your camera to capture images during the best seeing conditions.

Background:
The original design for the SSSM was published by E. J. Seykora at the Department of Physics, East Carolina University.

Read also how to integrate with Firecapture:
http://www.firecapture.de/
http://www.joachim-stehle.de/downloads/SSSMon/SSSMonPluginManual.pdf

Adapted schematics, connected to A0 and A1 for the ESP32-S2/S3-TFT replacing the originally used arduino.

<img title="Analog circuit schematics and parts list for SSSM analog electronics" alt="Schematics" src="pcb-schematics-analog.png">

This project is based on CircuitPython Version 8 (all required Libs included for reference) running on a Adafruit ESP32-S2/S3-TFT. The S3 is faster and better suited.

Anyhwo, the new S3 seams to have issues with WiFi. WiFi is not required here in USB-Serial mode compatible with the original Firecapture SSSM plugin and interface. Anyways planns are to use for MQTT for web based publishing, this works with the ESP32-S2-TFT, even not long term stable. Causes "Wifi-OFF" Progra mhold on the S3. TDB. May be next gen/version of Circuit Python fixes it. 

Currently: CircuitPython 8.0.0-beta.6

https://circuitpython.org/board/adafruit_feather_esp32s3_tft/

Adafruit ESP32-S3 TFT Feather - 4MB Flash, 2MB PSRAM, STEMMA QT:

https://www.adafruit.com/product/5483

Optional support for digital LUX light sensor TSL2591: Lux and Mag reading.

```
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
```
