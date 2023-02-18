# SSSM for Feather Adafruit ESP32 S2/S3 TFT
# Welcome to CircuitPython 8 :)
# (C) PyZahl 2023

import board
import gc
import time
import math
from digitalio import DigitalInOut, Direction, Pull
from analogio import AnalogIn
#from audioio import WaveFile, AudioOut
#import touchio
#import pulseio
import displayio
#from adafruit_st7789 import ST7789
from adafruit_display_shapes.rect import Rect

import busio
import ipaddress
from adafruit_lc709203f import LC709203F, PackSize

import ssl
import socketpool
import wifi
import adafruit_ntp
import rtc
import adafruit_minimqtt.adafruit_minimqtt as MQTT
import adafruit_requests

import random
#from rainbowio import colorwheel
from adafruit_esp32s2tft import ESP32S2TFT

## Setup the machine

gc.collect()   # make some rooooom

##                          S2 new S2 old
#             Center Value  CZ     Py      Ideal
R2_N_Measurements = 43 ## sqrt (N Samples for Ref Int and RMS)
##
#### new build V1
Analog0InCenter   = 33028
Analog1InCenter   = 33029
## 55  Zero Cal Info: [I, R, U0, U1]  21.58  20.30 1662.75 1663.12 mV avI,R:  436.3  405.0 avU0,1: 33027.8 33028.8 U0,1: 33021.2 33028.6 ## new build CZ - ESP32-S3

## Operation Modes
DipSW = {
	'1': DigitalInOut(board.A2),  ## Read/Write (DIP on := GND [==Logic False]) => recording to logfile / no recording
	'2': DigitalInOut(board.A3),  ## WiFi+MQTT Logging On/Off  *** disabled Use I2C Sensor
	'3': DigitalInOut(board.A4),  ## FireCapture prints/Full report prints to console
	'4': DigitalInOut(board.A5),  ## Zero Calib Info ON / OFF
	}

DipSWinfoClosed = { ## Logic "False" as DIP=ON is pin on GND
        '1': 'Record to Logfile',
        '2': 'WiFi+MQTT IoT Transmit',
        '3': 'FC Print',
        '4': 'Zero Offset Calibration',
        }

DipSWinfoOpen = { ## Logic "True" = OPEN/OFF
        '1': 'No Logging',
        '2': 'No Wifi+MQTT',
        '3': 'Long Status Print',
        '4': 'Normal Operation',
        }

## Print DIP SWITCH INFO
print ('Note: DIP ON (=GND) := False')
for pin in sorted(DipSW):
    DipSW[pin].switch_to_input(pull=Pull.UP)
    if DipSW[pin].value == False:
            info = DipSWinfoClosed[pin]
            dip='On: '
    else:
            info = DipSWinfoOpen[pin]
            dip='Off:'
    print ('Dip SW', pin, dip, info)

## Set a GPIO pin for digital Photo OK
## Photo OK threashold:
PhotoOKArcs = 1.0
PinPhotoOK = DigitalInOut(board.D5)
PinPhotoOK.direction = Direction.OUTPUT

## S3 dual core required wait patch/hack -- I2C issue with current batter monitor driver
def wait_i2c_ready():
	i2c = board.I2C()
	while not i2c.try_lock():
	    pass
	running = True
	try:
	    while running:
		print(
		    "I2C addresses found:",
		    [hex(device_address) for device_address in i2c.scan()],
		)
		# time.sleep(2)
		running = False

	finally:  # unlock the i2c bus when ctrl-c'ing out of the loop
	    i2c.unlock()
## S3 dual core required patch

## Configure Battery Monitor
# Create sensor object, using the board's default I2C bus.
wait_i2c_ready()
battery_monitor = LC709203F(board.I2C())

# Update to match the mAh of your battery for more accurate readings.
# Can be MAH100, MAH200, MAH400, MAH500, MAH1000, MAH2000, MAH3000.
# Choose the closest match. Include "PackSize." before it, as shown.
battery_monitor.pack_size = PackSize.MAH1000 #MAH400   # MAH1200 ???

bm_cell_percent=battery_monitor.cell_percent
print("Battery Percent: {:.2f} %".format(bm_cell_percent))
print("Battery Voltage: {:.2f} V".format(battery_monitor.cell_voltage))
    
 
# Get wifi credentials and details and more credentials (MQTT) from a secrets.py file
try:
    WiFi = True
    from secrets import secrets
except ImportError:
    print("WiFi secrets are kept in secrets.py, please add them there!")
    print("WiFi off now.")
    WiFi = False # disable to continue
    raise

if WiFi and DipSW['2'].value == False:
        print('Connecting WiFi...')
        print("Connecting to %s" % secrets["ssid"])
        wifi.radio.connect(secrets["ssid"], secrets["password"])
        print("Connected to %s!" % secrets["ssid"])

        pool = socketpool.SocketPool(wifi.radio)
        ntp = adafruit_ntp.NTP(pool, tz_offset=5)

        print('try: NTP clock sync...')

        #print(ntp.datetime)
        #print(time.localtime())

        # NOTE: This changes the system time so make sure you aren't assuming that time
        print('try: Sync system time to NTP...')
        rtc.RTC().datetime = ntp.datetime
        tmnow = time.localtime()
        #struct_time(tm_year=2022, tm_mon=7, tm_mday=22, tm_hour=4, tm_min=56, tm_sec=28, tm_wday=4, tm_yday=203, tm_isdst=-1)
        print('Start Timestamp: {:04d} {:02d} {:02d}  {:02d}:{:02d}:{:02d}'.format(tmnow.tm_year, tmnow.tm_mon, tmnow.tm_mday, tmnow.tm_hour, tmnow.tm_min, tmnow.tm_sec))
        WiFi = True
else:
        print ('WiFi OFF, no NTP auto time set.')
        tmnow = time.localtime()
        print ('System Start Timestamp: {:04d} {:02d} {:02d}  {:02d}:{:02d}:{:02d}'.format(tmnow.tm_year, tmnow.tm_mon, tmnow.tm_mday, tmnow.tm_hour, tmnow.tm_min, tmnow.tm_sec))
        WiFi = False

## MQTT ##  
# Setup a feed named 'photocell' for publishing to a feed
sssm_time_feed = "feeds/sssm/time"
sssm_arcs_feed = "feeds/sssm/arcs"
sssm_ref_feed  = "feeds/sssm/refint"
sssm_bat_feed  = "feeds/sssm/battery"

## test adafruit.io -- very limited rate and points (30/min total)
#sssm_time_feed = "pyzahl2/feeds/sssm.time"
#sssm_arcs_feed = "pyzahl2/feeds/sssm.arcs"
#sssm_ref_feed  = "pyzahl2/feeds/sssm.refint"
#sssm_bat_feed  = "pyzahl2/feeds/sssm.battery"

# Setup a feed named 'onoff' for subscribing to changes
#onoff_feed = "/feeds/info"

## MQTT support callbacks
def connect(mqtt_client, userdata, flags, rc):
    print("Connected to MQTT Broker!")
    print("Flags: {0}\n RC: {1}".format(flags, rc))


def disconnect(mqtt_client, userdata, rc):
    print("Disconnected from MQTT Broker!")


def subscribe(mqtt_client, userdata, topic, granted_qos):
    print("Subscribed to {0} with QOS level {1}".format(topic, granted_qos))


def unsubscribe(mqtt_client, userdata, topic, pid):
    print("Unsubscribed from {0} with PID {1}".format(topic, pid))


def publish(mqtt_client, userdata, topic, pid):
    print("Published to {0} with PID {1}".format(topic, pid))


def message(client, topic, message):
    print("New message on topic {0}: {1}".format(topic, message))

## Initiate MQTT
if WiFi:
        print ('MQTT Client Configuration:')
        #pool = socketpool.SocketPool(wifi.radio) ## got that above already

        mqtt_client = MQTT.MQTT(
                broker=secrets["broker"],
                port=secrets["port"],
                username=secrets["aio_username"],
                password=secrets["aio_key"],
                socket_pool=pool,
                #is_ssl=True,
                #ssl_context=ssl.create_default_context(),
        )

        mqtt_client.on_connect = connect
        mqtt_client.on_disconnect = disconnect
        mqtt_client.on_subscribe = subscribe
        mqtt_client.on_unsubscribe = unsubscribe
        #mqtt_client.on_publish = publish
        mqtt_client.on_message = message

	print("MQTT: Attempting to connect to {}::{}".format(mqtt_client.broker, mqtt_client.port))
	try:
		mqtt_client.connect()
		mqtt_client_connected = True
	except RuntimeError as e:
		print ('Connect error: ',e)
		mqtt_client_connected = False
	        print ('MQTT disabled now.')
else:
        print ('MQTT disabled.')

## Create/append to logfile on drive?
## NOTE: Make sure to purge/delete manually when using too much space!!
logging = False
if DipSW['1'].value == False:
	try:
	    with open("/lightcurve.log", "a") as light_log:
		light_log.write('# Log Start: {:04d} {:02d} {:02d}  {:02d}:{:02d}:{:02d}'.format(tmnow.tm_year, tmnow.tm_mon, tmnow.tm_mday, tmnow.tm_hour, tmnow.tm_min, tmnow.tm_sec))
		light_log.flush()
		logging = True
	except OSError as e:  # When the filesystem is NOT writable by CircuitPython...
		print('Can not access drive:', e)

### Prepare data strcutures for graphics
### ESP32-S2 240MHz Color 1.14" IPS TFT with 240x135 pixels ST7789
esp32s2tft = ESP32S2TFT(
    default_bg=0x000000,
    scale=1,
)


# Create the labels
status = esp32s2tft.add_text(
	text='SSSM V1.0',
	text_position=(0, 10),
	text_scale=3,
	text_color=0xFF00FF
)

battery = esp32s2tft.add_text(
	text='{:.1f} %'.format(bm_cell_percent),
	text_position=(240, 10),
	text_anchor_point=(1.0, 0.5),
	text_scale=1,
	text_color=0x00FF00
)

info_last = " "
info_label = esp32s2tft.add_text(
	text=info_last,
	line_spacing=1.0,
	text_position=(240, 20),
	text_anchor_point=(1.0, 0.5),
	text_scale=1,
	text_color=0x606060,
)

reading_last = '(C) PyZahl'

reading = esp32s2tft.add_text(
	text=reading_last,
	text_position=(0, 40),
	text_scale=2,
	text_anchor_point=(0.0, 0.5),
	text_color=0xFFFF00,
)

if 1:
	reading_aux_last = 'Happy Seeing'
#	reading_aux_last = 'Happy Birthday'

	reading_aux = esp32s2tft.add_text(
		text=reading_aux_last,
		text_position=(0, 65),
		text_scale=2,
		text_anchor_point=(0.0, 0.5),
		text_color=0xFFFF00,
	)

	reading_arc_last = 'K150'
#	reading_arc_last = 'Carsten!!!'

	reading_arc = esp32s2tft.add_text(
		text=reading_arc_last,
		text_position=(0, 90),
		text_scale=2,
		text_anchor_point=(0.0, 0.5),
		text_color=0xFFFF00,
	)

# Prepare plotting area bitmap        
if 1:
	GYMAX = 70
	GYMAX1=GYMAX-1
	graph_bitmap = displayio.Bitmap(240, GYMAX, 6)
	color_palette = displayio.Palette(6)
	color_palette[0] = 0x000000 # clear, black
	color_palette[1] = 0xFF4000 # orange for arcs
	color_palette[2] = 0xFFFF00 # yellow for ref sun intensity 
	color_palette[3] = 0x0000FF # blue
	color_palette[4] = 0x330000 # dark red, grid
	color_palette[5] = 0x003300 # 1 arcs line / dark green

	gr_sprite = displayio.TileGrid(graph_bitmap,
		                       pixel_shader=color_palette,
		                       x=0, y=55)
		                       
	esp32s2tft.splash.append(gr_sprite)


#NN=32
#for i in range(NN):
#	esp32s2tft.splash.append(Rect(i*8, 100, 4, 20, fill=0x00FF00))

esp32s2tft.display.show(esp32s2tft.splash)

## Configure Analog Inputs used
# Analog input on A1
analog0in = AnalogIn(board.A0)   # Reference Intensity
analog1in = AnalogIn(board.A1)   # Analog integrated AC signal

######################### HELPERS ##############################

# Helper to convert analog input to voltage
def getVoltage(pin):
    return (pin.value * 3.3) / 65536

def Voltage(value):
    return (value * 3.3) / 65536

def mVoltage(value):
    return (value * 3.3) / 65.536

## Center/Zero Measurement
def SSSM_analog_zero_calib(n22=32):
	N=n22*n22
	fn22=float(n22)
        I=0
        R=0
        U0=0
        U1=0
        ## some integer math radix 2 tricks
	for i in range(N):
		u0 = analog0in.value - Analog0InCenter
		u1 = analog1in.value - Analog1InCenter
                I = I + u0
                R = R + u1
		U0 = U0 + analog0in.value
                U1 = U1 + analog1in.value
                ##**
        I=I/n22
        I=I/fn22
        R=R/n22
        R=R/fn22
        U0=U0/n22
        U0=U0/fn22
        U1=U1/n22
        U1=U1/fn22
        return I, R, U0, U1

        
## SSSM analog read -- NOTE: we do have only FLOAT here (30-bit wide floating point) and 32bit integer
def SSSM_analog(n22=32):
	I=0
	RMS=0
	N=n22*n22
	fn22=float(n22)
        ## some integer math radix 2 tricks
	for i in range(N):
		u0 = analog0in.value - Analog0InCenter
		u1 = analog1in.value - Analog1InCenter
		I = I + u0
		RMS= RMS + u1*u1

        ref = I/fn22   # I>>10 = I/32/32
	if ref < 1:
		ref = 1 # prevent div zero and negative nonsense

	rms = math.sqrt(RMS)  ## sqrt(RMS>>10) = sqrt(RMS)/32
	# arcs=1886.79/425.5*rms/ref ## 425.5 (=20M/47k) is the AC "400Hz" BW signal gain vs ref singal. 1886.79 is the conversion factor to arcs
	# 1886.79/425.5 = 4.434289
	# (rms / 32) / (ref / 32)
	arcs = 4.434289*rms/ref
	if arcs > 20: # clip
		arcs=20.0
                
	return ref/fn22, rms/fn22, arcs

## SSM test -- Only for testing, no actual use
def SSSM_test():
	nowns = time.monotonic_ns()
	startns = nowns
	while True:
		nowns = time.monotonic_ns()
		u0, u1, ref, rms, arcs = SSSM_analog()
		reading_last = 'A0 {:4.2f}V A1 {:4.2f}V'.format(Voltage(u0), Voltage(u1))
		reading_aux_last = 'I {:5.3f} RMS {:5.3f}'.format(Voltage(ref), Voltage(rms))
		reading_arc_last = 'ARCs {:5.2f}'.format(arcs)
		
		print (reading_last+', '+reading_aux_last)
		esp32s2tft.set_text(
			reading_last, reading
		)
		esp32s2tft.set_text(
			reading_aux_last, reading_aux
		)
		esp32s2tft.set_text(
			reading_arc_last, reading_arc
		)

		esp32s2tft.set_text(
			'{:.1f}s Bat {:.1f} %'.format((nowns-startns)/1e9, bm_cell_percent), battery
		)

#SSSM_test()


######################### MAIN LOOP ##############################

esp32s2tft.display.auto_refresh = False

#print('Test Log10(10): ', math.log(10)/2.302585092994046)

I0 = 0
i=0

klen = 16
kshr = 4
rmslen = 16
rmsshr = 4

ISum = -1
dI = 0
IrmsS = 0
arcs = 0.0

arcs_filter = 0.0
ref_filter = 0.0
rms_filter = 0.0

arcs_hist = []
refint_hist = []
nowns = time.monotonic_ns()
startns = nowns

##############
ZN=0
CAL_I, CAL_R, CAL_U0, CAL_U1 = SSSM_analog_zero_calib (R2_N_Measurements)
reading_last='{:4.0f} {:4.0f} {:4.0f} {:4.0f}'.format(mVoltage(CAL_I), mVoltage(CAL_R), mVoltage(CAL_U0), mVoltage(CAL_U1))
print ('***')
print ('Zero Cal Init: [I, R, U0, U1] ', reading_last, 'mV {:6.1f} {:6.1f} {:6.1f} {:6.1f}'.format(CAL_I, CAL_R, CAL_U0, CAL_U1))

if abs(mVoltage(CAL_I)) > 20 or abs(mVoltage(CAL_R)) > 20:
        print ('Zero Offset Calibration Warning! Please check dark.')
        print ('***')
        print ('Actual Center: {:6.0f} {:6.0f}'.format(CAL_U0, CAL_U1))
        CAL_I, CAL_R, CAL_U0, CAL_U1 = SSSM_analog_zero_calib (R2_N_Measurements)
        print ('Actual Center: {:6.0f} {:6.0f}'.format(CAL_U0, CAL_U1))
        CAL_I, CAL_R, CAL_U0, CAL_U1 = SSSM_analog_zero_calib (R2_N_Measurements)
        print ('Actual Center: {:6.0f} {:6.0f}'.format(CAL_U0, CAL_U1))
        CAL_I, CAL_R, CAL_U0, CAL_U1 = SSSM_analog_zero_calib (R2_N_Measurements)
        print ('Actual Center: {:6.0f} {:6.0f}'.format(CAL_U0, CAL_U1))
        print ('***')
else:
        print ('Zero Calibration: passed')
        print ('***')
#############

ref_filter, rms_filter, arcs_filter = SSSM_analog (R2_N_Measurements)

long_reading = '#SEC------\tRef--\tRMS--\tarcs--\tBAT%'
while True:
	# get time in seconds since start
	nowns = time.monotonic_ns()
	s=int((nowns-startns)/1e9)

	# take measurement (SSSM analog)
	ref, rms, arcs = SSSM_analog (R2_N_Measurements)

        # Calculate filtered values
        w  = 0.10 ## 10% IIR
        wf = 1.00-w
        ref_filter  = wf *  ref_filter + w * ref
        rms_filter  = wf *  rms_filter + w * rms
        arcs_filter = wf * arcs_filter + w * arcs
        #arcs_average10  = sum(arcs_hist[-10:])/10 # Average Seeing last 10 readings

        # intensity reference scaled:
        refint = ref/10 # just a value, linear here

        # record last values for plotting and statistics
	if len(arcs_hist) >= 240:
		arcs_hist.pop(0)
		refint_hist.pop(0)
		
	arcs_hist.append(arcs)
	refint_hist.append(refint)

        # Median once enough values
	if len(arcs_hist) >= 18:
            tmp = arcs_hist[-17:] # last 17
            tmp.sort()
            arcs_median = tmp[8]  # take middle value
        else:
            arcs_median = arcs_filter
        
	# log to file?	
	try:
	    with open("/lightcurve.log", "a") as light_log:
		light_log.write(long_reading+'\n')
		logging=True #print('Can not access drive:', e)
	except OSError as e:  # When the filesystem is NOT writable by CircuitPython...
		logging=False #print('Can not access drive:', e)

	# every 60s update battery info
        if s%60 == 0:
        	print('** Battery Check **')
	        wait_i2c_ready()
        	print('** reading...    **')
	        bm_cell_percent = battery_monitor.cell_percent
        	print('** Battery: {:.2f} %  {:.2f} V **'.format(bm_cell_percent, battery_monitor.cell_voltage))

	long_reading = '{:d}\t{:5.3f}\t{:6.3f}\t{:6.3f}\t{:5.1f}'.format(time.time(), Voltage(ref), Voltage(rms), arcs, bm_cell_percent)
        
        # Print Zero Cal Info
        if DipSW['4'].value == False:
                I, R, U0, U1 = SSSM_analog_zero_calib (R2_N_Measurements)
                CAL_I = CAL_I*0.99 + I*0.01
                CAL_R = CAL_R*0.99 + R*0.01
                CAL_U0 = CAL_U0*0.99 + U0*0.01
                CAL_U1 = CAL_U1*0.99 + U1*0.01
                ZN=ZN+1
	        reading_last='{:4.0f} {:4.0f} {:4.0f} {:4.0f}'.format(mVoltage(I), mVoltage(R), mVoltage(U0), mVoltage(U1))
                print (ZN,' Zero Cal Info: [I, R, U0, U1] {:6.2f} {:6.2f} {:6.2f} {:6.2f}'.format(mVoltage(I), mVoltage(R), mVoltage(U0), mVoltage(U1)), 'mV avI,R: {:6.1f} {:6.1f} avU0,1: {:6.1f} {:6.1f} U0,1: {:6.1f} {:6.1f}'.format(CAL_I, CAL_R, CAL_U0, CAL_U1, U0, U1))
        else:
        # DISPLAY Voltages Io, RMS or arcs_filter if "good"
                if arcs > 10 and ref > 2000:
	                reading_last='Io{:6.1f}mV S {:4.1f}"'.format(mVoltage(ref_filter), arcs_filter)
                else:
	                reading_last='Io{:6.1f}mV R {:4.1f}mV'.format(mVoltage(ref_filter), mVoltage(rms_filter))

	# Draw graph into bitmap
	x=0
	yp=0
	for yval in (arcs_hist):
		# clear column
		for yc in range(0,GYMAX):
			if yc % 10 == 0:	
				graph_bitmap[x, yc] = 4	# grid line
			else:
				graph_bitmap[x, yc] = 0	# clear
		graph_bitmap[x, GYMAX-10] = 5 # green (1 arcs grid line) 
		yi = GYMAX1 - int(round(yval*10))%GYMAX
                # draw arcs history graph
		while yp != yi:
			if yp > yi:
				yp = yp - 1
			else:
				yp = yp + 1
			graph_bitmap[x, yp] = 1
		if yp < GYMAX1:
			graph_bitmap[x, yp+1] = 1

                # draw ref intensity history graph -- auto scale (decades)
		yval = refint_hist[x]
		while yval > 10.:
			yval = yval/10.
		while yval < 1.:
			yval = yval*10.
		yi = GYMAX1 - int(round(yval/10.*GYMAX))
		graph_bitmap[x, yi%GYMAX] = 2
		if yi < GYMAX1:
			graph_bitmap[x, yi+1] = 2 #
		x=x+1

        # DISPLAY ARCS MAIN STATUS
	if arcs < 10.0 and ref > 2000: # ref voltage > 200mV
		status_reading = 'S {:4.1f}"'.format(arcs_median)
	else:
		arcs = 9.99
		status_reading = 'SSSM *"'

	esp32s2tft.set_text(
		status_reading, status
	)

	esp32s2tft.set_text(
		reading_last, reading
	)

	esp32s2tft.set_text(
		info_last, info_label
	)

	esp32s2tft.set_text(
		'{:.1f}s Bat {:.1f} %'.format(s, bm_cell_percent), battery
	)

	esp32s2tft.set_text_color(
		0xFF0000 if bm_cell_percent < 20 else 0x00FF00, battery
	)



	esp32s2tft.set_text_color(
		0x00FF00 if arcs < 1.0 else 0xFF0000 if logging else 0xFF00FF, status
	)

	esp32s2tft.set_text_color(
		0xFF0000 if esp32s2tft.peripherals.button else 0x606060, info_label
	)
	#esp32s2tft.peripherals.led = esp32s2tft.peripherals.button
	#if esp32s2tft.peripherals.button:
	#	esp32s2tft.peripherals.neopixel[0] = colorwheel(random.randint(0, 255))

	esp32s2tft.display.refresh ()

	if DipSW['3'].value == False: ## ref signal > 100mV, arcs < 20
        	# Firecapture PlugIn Compatible
                print ('A0: {:4.2f}'.format(Voltage(ref))) # Intensity Value (Volts)
#                print ('A0: {:4.2f}'.format(arcs_median-2)) # Median last 17 Seeing readings [testing]
                print ('A1: {:4.2f}'.format(arcs)) # Normalized RMS / Variation Value / Seeing
#                print ('C2: {:4.2f}'.format(sum(arcs_hist[-10:])/10)) # Average Seeing last 10 readings
#                print ('M1: {:4.2f}'.format(arcs_median)) # Median last 17 Seeing readings
        else:
                print (status_reading, long_reading)
                        
	

        # WiFi + MQTT?
        if WiFi and mqtt_client_connected: ## ref signal > 100mV, arcs < 20
		try:
			esp32s2tft.peripherals.led = True
			print("MQTT publishing...")
			# Poll the message queue
			#mqtt_client.loop()
			# Send a new message
                        #if s%3: # throttle rate
                        if 1: # no throttle
			        #mqtt_client.publish(sssm_time_feed, nowns/1e9)
			        mqtt_client.publish(sssm_ref_feed, Voltage(ref))
			        mqtt_client.publish(sssm_arcs_feed, arcs)
			if s%60 == 0:
				mqtt_client.publish(sssm_bat_feed, bm_cell_percent)
			esp32s2tft.peripherals.led = False
			info_last = "MQTT transmitting"
		except (ValueError, RuntimeError) as e:
			esp32s2tft.peripherals.led = False
			print("MQTT failed, retrying. WiFi reset/reconnect attempt. E:", e)
			info_last = "WiFi reset attempt"
			try:
			        print('*** WiFi Reset/Reconnect attempt')
			        wifi.reset()
			        wifi.connect()
			        mqtt_client.reconnect()
			        print('*** WiFi Reset/Reconnect attempt completed')
			except (ValueError, RuntimeError) as e:
				print("WiFi reset failed, retrying next loop. E:", e)
        else:
		info_last = "No MQTT."
		esp32s2tft.peripherals.led = False
                
	if arcs < PhotoOKArcs:
                info_last = 'Photo OK * '+info_last
		esp32s2tft.peripherals.led = True
		PinPhotoOK.value = True
	else:
		PinPhotoOK.value = False


