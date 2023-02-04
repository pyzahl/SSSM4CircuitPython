# boot.py -- configuration of RW for writing data to drive 

import board
import digitalio
import storage

print ('Boot Setup: drive RO/RW, RO is default, GND A2 for RW')

switch = digitalio.DigitalInOut(board.A2) # DIP 1
switch.direction = digitalio.Direction.INPUT
switch.pull = digitalio.Pull.UP

# If the A2 is connected to ground with a wire
# CircuitPython can write to the drive
if switch.value == False:
    print ('Log File Switch A2 = GND: Remounting RW')
    storage.remount("/", True)
