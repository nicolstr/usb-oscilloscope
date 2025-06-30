# file: app_series_STOP_v1.py
# 
# This program traverses through the FSM of the µC.
# The FSM is controlled with string-commands send over the emulated Serial Interface (hardware: USB FS).
# 
# used driver on µC: USB CDC Device (STM32F7);
# 
# works with STM32CubeIDE-project "Kommunikationskonzept_V1_CDC" (no sendData()!)
#
# Beginning: FSM is in Initial State (Init)
# "APP\n" starts the application (Init->Idle)
# after execution of the program, the FSM is in Idle State
# 
# single commands can be easily sent via hterm-software
# https://www.der-hammer.info/pages/terminal.html
# Sent commands need to include a EOL-char (\n or \r) --> setting "send on enter XX" XX = LF or CR

import time
import serial

#--configure port--
ser = serial.Serial()

# baud rate is emulated!
# real transmission speed is determined by USB speed (USB FS: 12MBit/s)
ser.baudrate = 115200   

# timeout is needed for read()-/readline()-functions (e.g ACQUISITION:DONE)
ser.timeout = 5

# COM-port on Windows PC (VCP)
ser.port = 'COM13'

#print information (see default values of parameters, that were not explicitly set)
print(ser)

# open serial port
ser.open()
print('--Port opened: {0}--'.format(ser.is_open))
print('')

# wait for 2 seconds
time.sleep(2)

# start application of µC
print('--Start app--')
print("out: \"APP\"")
ser.write(b'APP\n')
# get acknoledgement (STATE:IDLE)
line = ser.readline()
print("in: {0}".format(line))

print('')
time.sleep(2)

print('--Start acquisiton (waits for trigger)--')
print("out: \"RUN_START\"")
ser.write(b'RUN_START\n')
# get acknoledgement (STATE:ACQUISITION)
line = ser.readline()
print("in: {0}".format(line))

print('--wait for end of acquisition--')
# wait for ACQUISITON:DONE (if > timeout --> empty)
line = ser.readline()
print("in: {0}".format(line))

print('')
time.sleep(2)

print('--Signal readiness for data reception--')
print("out: \"READY_FOR_DATA\"")
ser.write(b'READY_FOR_DATA\n')
# get acknoledgement (STATE:TRANSMISSION)
line = ser.readline()
print("in: {0}".format(line))

print('')
time.sleep(2)

print('--Acknowledge reception--')
print("out: \"RECEIVED\"")
ser.write(b'RECEIVED\n')
# get acknoledgement (STATE:WAIT_FOR_ACK)
line = ser.readline()
print("in: {0}".format(line))

print('')
time.sleep(2)

print('--send ACK to µC: STOP--')
print("out: \"STOP\"")
ser.write(b'STOP\n') # alternative: CONTINUE -> back in STATE:ACQUISITION
# get acknoledgement (STATE:IDLE)
line = ser.readline()
print("in: {0}".format(line))

# close serial port
ser.close()

