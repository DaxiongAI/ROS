#!/usr/bin/env python

import thread
from math import pi as PI, degrees, radians
import os
import time
import sys, traceback
from serial.serialutil import SerialException
from serial import Serial

SERVO_MAX = 180
SERVO_MIN = 0

class Arduino:
    ''' Configuration Parameters
    '''
    N_ANALOG_PORTS = 6
    N_DIGITAL_PORTS = 12

    def __init__(self, port="/dev/ttyUSB0", baudrate=57600, timeout=0.5, motors_reversed=False):

        self.PID_RATE = 30 # Do not change this!  It is a fixed property of the Arduino PID controller.
        self.PID_INTERVAL = 1000 / 30

        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.encoder_count = 0
        self.writeTimeout = timeout
        self.interCharTimeout = timeout / 40.
        self.motors_reversed = motors_reversed
        # Keep things thread safe
        self.mutex = thread.allocate_lock()

        # An array to cache analog sensor readings
        #self.analog_sensor_cache = [None] * self.N_ANALOG_PORTS

        # An array to cache digital sensor readings
        #self.digital_sensor_cache = [None] * self.N_DIGITAL_PORTS

    def connect(self):
        try:
            print "Connecting to Arduino on port", self.port, "..."
            self.port = Serial(port=self.port, baudrate=self.baudrate, timeout=self.timeout, writeTimeout=self.writeTimeout)
            # The next line is necessary to give the firmware time to wake up.
            time.sleep(1)
            test = self.get_baud()
            if test != self.baudrate:
                time.sleep(1)
                test = self.get_baud()
                if test != self.baudrate:
                    raise SerialException
            print "Connected at", self.baudrate
            print "Arduino is ready."

        except SerialException:
            print "Serial Exception:"
            print sys.exc_info()
            print "Traceback follows:"
            traceback.print_exc(file=sys.stdout)
            print "Cannot connect to Arduino!"
            os._exit(1)

    def open(self):
        ''' Open the serial port.
        '''
        self.port.open()

    def close(self):
        ''' Close the serial port.
        '''
        self.port.close()

    def send(self, cmd):
        ''' This command should not be used on its own: it is called by the execute commands
            below in a thread safe manner.
        '''
        self.port.write(cmd + '\r')

    def recv(self, timeout=0.5):
        timeout = min(timeout, self.timeout)
        ''' This command should not be used on its own: it is called by the execute commands
            below in a thread safe manner.  Note: we use read() instead of readline() since
            readline() tends to return garbage characters from the Arduino
        '''
        c = ''
        value = ''
        attempts = 0
        while c != '\r':
            c = self.port.read(1)   ###one character
            value += c
            attempts += 1
            if attempts * self.interCharTimeout > timeout:
                return None

        value = value.strip('\r')

        return value

    def recv_ack(self):
        ''' This command should not be used on its own: it is called by the execute commands
            below in a thread safe manner.
        '''
        ack = self.recv(self.timeout)
        return ack == 'OK'

    def recv_int(self):
        ''' This command should not be used on its own: it is called by the execute commands
            below in a thread safe manner.
        '''
        value = self.recv(self.timeout)
        try:
            return int(value)
        except:
            return None

    def recv_array(self):
        ''' This command should not be used on its own: it is called by the execute commands
            below in a thread safe manner.
        '''
        try:
            values = self.recv(self.timeout * self.N_ANALOG_PORTS).split()   
            return map(int, values)
        except:
            return []

    def execute(self, cmd):
        ''' Thread safe execution of "cmd" on the Arduino returning a single integer value.
        '''
        self.mutex.acquire()

        try:
            self.port.flushInput()
        except:
            pass

        ntries = 1
        attempts = 0

        try:
            self.port.write(cmd + '\r')
            value = self.recv(self.timeout)
            while attempts < ntries and (value == '' or value == 'Invalid Command' or value == None):
                try:
                    self.port.flushInput()
                    self.port.write(cmd + '\r')
                    value = self.recv(self.timeout)
                except:
                    print "Exception executing command: " + cmd
                attempts += 1
        except:
            self.mutex.release()
            print "Exception executing command: " + cmd
            value = None

        self.mutex.release()
        return int(value)

    def execute_array(self, cmd):
        ''' Thread safe execution of "cmd" on the Arduino returning an array.
        '''
        self.mutex.acquire()

        try:
            self.port.flushInput()
        except:
            pass

        ntries = 1
        attempts = 0

        try:
            self.port.write(cmd + '\r')
            values = self.recv_array()
            while attempts < ntries and (values == '' or values == 'Invalid Command' or values == [] or values == None):
                try:
                    self.port.flushInput()
                    self.port.write(cmd + '\r')
                    values = self.recv_array()
                except:
                    print("Exception executing command: " + cmd)
                attempts += 1
        except:
            self.mutex.release()
            print "Exception executing command: " + cmd
            raise SerialException
            return []

        try:
            values = map(int, values)
        except:
            values = []

        self.mutex.release()
        return values

    def execute_ack(self, cmd):
        ''' Thread safe execution of "cmd" on the Arduino returning True if response is ACK.
        '''
        self.mutex.acquire()

        try:
            self.port.flushInput()
        except:
            pass

        ntries = 1
        attempts = 0

        try:
            self.port.write(cmd + '\r')
            ack = self.recv(self.timeout)
            while attempts < ntries and (ack == '' or ack == 'Invalid Command' or ack == None):
                try:
                    self.port.flushInput()
                    self.port.write(cmd + '\r')
                    ack = self.recv(self.timeout)
                except:
                    print "Exception executing command: " + cmd
            attempts += 1
        except:
            self.mutex.release()
            print "execute_ack exception when executing", cmd
            print sys.exc_info()
            return 0

        self.mutex.release()
        return ack == 'OK'

    def update_pid(self, AWheel_Kp, AWheel_Kd, AWheel_Ki, AWheel_Ko,
                         BWheel_Kp, BWheel_Kd, BWheel_Ki, BWheel_Ko,
                         CWheel_Kp, CWheel_Kd, CWheel_Ki, CWheel_Ko,DWheel_Kp, DWheel_Kd, DWheel_Ki, DWheel_Ko):
        ''' Set the PID parameters on the Arduino
        '''
        print "Updating PID parameters"
        cmd = 'u' + ' ' + str(AWheel_Kp) + ':' + str(AWheel_Kd) + ':' + str(AWheel_Ki) + ':' + str(AWheel_Ko) + ':' + str(BWheel_Kp) + ':' + str(BWheel_Kd) + ':' + str(BWheel_Ki) + ':' + str(BWheel_Ko) + ':' + str(CWheel_Kp) + ':' + str(CWheel_Kd) + ':' + str(CWheel_Ki) + ':' + str(CWheel_Ko) + ':' + str(DWheel_Kp) + ':' + str(DWheel_Kd) + ':' + str(DWheel_Ki) + ':' + str(DWheel_Ko)
        self.execute_ack(cmd)

    def get_baud(self):
        ''' Get the current baud rate on the serial port.
        '''
        try:
            return int(self.execute('b'));
        except:
            return None

    def get_encoder_counts(self):
        values = self.execute_array('e')
        if len(values) != 4:
            print "Encoder count was not 4"
            raise SerialException
            return None
        else:
           ## if self.motors_reversed:
            #    values[0], values[1] = -values[0], -values[1]
            return values

    def reset_encoders(self):
        ''' Reset the encoder counts to 0
        '''
        return self.execute_ack('r')

    def drive(self,AWheel,BWheel,CWheel,DWheel):
        ''' Speeds are given in encoder ticks per PID interval
        '''
       # if self.motors_reversed:
       #     left, right = -left, -right
        return self.execute_ack('m %d %d %d %d' % (AWheel, BWheel, CWheel, DWheel))

    def stop(self):
        ''' Stop both motors.
        '''
        self.drive(0, 0, 0, 0)

    def analog_read(self, pin):
        return self.execute('a %d' %pin)

    def analog_write(self, pin, value):
        return self.execute_ack('x %d %d' %(pin, value))

    def digital_read(self, pin):
        return self.execute('d %d' %pin)

    def digital_write(self, pin, value):
        return self.execute_ack('w %d %d' %(pin, value))

    def pin_mode(self, pin, mode):
        return self.execute_ack('c %d %d' %(pin, mode))

    def get_pidin(self):
        values = self.execute_array('i')
        if len(values) != 4:
            print "pidin was not 4"
            raise SerialException
            return None
        else:
            return values

    def get_pidout(self):
        values = self.execute_array('o')
        if len(values) != 4:
            print "pidout was not 4"
            raise SerialException
            return None
        else:
            return values

    def ping(self, pin):
        ''' The srf05/Ping command queries an SRF05/Ping sonar sensor
            connected to the General Purpose I/O line pinId for a distance,
            and returns the range in cm.  Sonar distance resolution is integer based.
        '''
        return self.execute('p %d' %pin);


""" Basic test for connectivity """
if __name__ == "__main__":
    if os.name == "posix":
        portName = "/dev/ttyACM0"
    else:
        portName = "COM43" # Windows style COM port.

    baudRate = 57600

    myArduino = Arduino(port=portName, baudrate=baudRate, timeout=0.5)
    myArduino.connect()

    print "Sleeping for 1 second..."
    time.sleep(1)

    print "Reading on analog port 0", myArduino.analog_read(0)
    print "Reading on digital port 0", myArduino.digital_read(0)
    print "Blinking the LED 3 times"
    for i in range(3):
        myArduino.digital_write(13, 1)
        time.sleep(1.0)
    #print "Current encoder counts", myArduino.encoders()

    print "Connection test successful.",

    myArduino.stop()
    myArduino.close()

    print "Shutting down Arduino."
