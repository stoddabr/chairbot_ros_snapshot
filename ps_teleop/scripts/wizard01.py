#!/usr/bin/env python
import roslib
import rospy
import serial
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Vector3
from tf.broadcaster import TransformBroadcaster
from math import sin,cos

from geometry_msgs.msg import Twist

import sys, select, termios, tty

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Vector3

import time
from std_msgs.msg import UInt16

msg = """
Reading from the Dualshock4 controller ~!! And Publishing to neato!
---------------------------
Move around:
"""

class Botvac():

    def __init__(self, port):
        self.port = serial.Serial(port,115200)
        # Storage for motor and sensor information
        self.state = {"LeftWheel_PositionInMM": 0, "RightWheel_PositionInMM": 0,"LSIDEBIT": 0,"RSIDEBIT": 0,"LFRONTBIT": 0,
                      "RFRONTBIT": 0,"BTN_SOFT_KEY": 0,"BTN_SCROLL_UP": 0,"BTN_START": 0,"BTN_BACK": 0,"BTN_SCROLL_DOWN": 0}
        self.stop_state = True
        
        self.base_width = 248    # millimeters
        self.max_speed = 300     # millimeters/second
        self.crtl_z = chr(26)
        # turn things on
        self.port.flushInput()
        self.port.write("\n")
        self.setTestMode("on")
        #self.setLDS("on")
        
    def flushing(self):
      self.port.flushInput()
        
    def exit(self):
        self.port.flushInput()
        self.port.write("\n")
        self.setLDS("off")
        self.setTestMode("off")

    def setTestMode(self, value):
        """ Turn test mode on/off. """
        self.port.write("testmode " + value + "\n")
        self.readResponseString()

    def setLDS(self, value):
        self.port.write("setldsrotation " + value + "\n")
        self.readResponseString()

    def requestScan(self):
        """ Ask neato for an array of scan reads. """
        self.port.flushInput()
        self.port.flushOutput()
        self.port.write("getldsscan\n")
        response = self.readResponseString()
        return response

    def readResponseString(self):
        """ Returns the entire response from neato in one string. """
        response = str()
        self.port.timeout = 0.001
        while True:
            try:
                buf = self.port.read(1024)
            except:
                return ""
            if len(buf) == 0:
                self.port.timeout *= 2
            else:
                response += buf
                if buf[len(buf)-1] == self.crtl_z:
                    break;
        self.port.timeout = None
        return response

    def getScanRanges(self):
        """ Read values of a scan -- call requestScan first! """
        ranges = list()
        response = self.requestScan()
        #response = self.readResponseString()
        #print("scan " , response)
        for line in response.splitlines():
            vals = line.split(",")
            # vals[[0] angle, vals[1] range, vals[2] intensity, vals[3] error code
            if len(vals) >= 2 and vals[0].isdigit() and vals[1].isdigit():
                ranges.append(int(vals[1])/1000.0)
        # sanity check
        if len(ranges) != 360:
            return []
        return ranges
        
    def setMotors(self, l, r, s):
        """ Set motors, distance left & right + speed """
        #This is a work-around for a bug in the Neato API. The bug is that the
        #robot won't stop instantly if a 0-velocity command is sent - the robot
        #could continue moving for up to a second. To work around this bug, the
        #first time a 0-velocity is sent in, a velocity of 1,1,1 is sent. Then, 
        #the zero is sent. This effectively causes the robot to stop instantly.
        if (int(l) == 0 and int(r) == 0 and int(s) == 0):
            if (not self.stop_state):
                self.stop_state = True
                l = 1
                r = 1
                s = 1
        else:
            self.stop_state = False
        self.port.write("setmotor "+str(int(l))+" "+str(int(r))+" "+str(int(s))+"\n")
        print "sending commands to neato robot"
        
    def readResponseAndUpdateState(self):
        """ Read neato's response and update self.state dictionary.
            Call this function only after sending a command. """
        response = self.readResponseString()
        for line in response.splitlines():
            #print(line)
            vals = line.split(",")
            if len(vals) >= 2 and vals[0].replace('_', '').isalpha() and vals[1].isdigit():
                self.state[vals[0]] = int(vals[1])
                #print(vals[0] , vals[1])

    def getMotors(self):
        """ Update values for motors in the self.state dictionary.
            Returns current left, right encoder values. """
        self.port.flushInput()
        self.port.write("getmotors\n")
        self.readResponseAndUpdateState()
        return [self.state["LeftWheel_PositionInMM"],self.state["RightWheel_PositionInMM"]]

    def getAnalogSensors(self):
        """ Update values for analog sensors in the self.state dictionary. """
        self.port.write("getanalogsensors\n")
        self.readResponseAndUpdateState()

    def getDigitalSensors(self):
        """ Update values for digital sensors in the self.state dictionary. """
        self.port.write("getdigitalsensors\n")
        self.readResponseAndUpdateState()
        return [self.state["LSIDEBIT"],self.state["RSIDEBIT"],self.state["LFRONTBIT"],self.state["RFRONTBIT"]]

    def getButtons(self):
        """ Update values for digital buttons in the self.state dictionary. """
        self.port.write("getbuttons\n")
        self.readResponseAndUpdateState()
        return [self.state["BTN_SOFT_KEY"],self.state["BTN_SCROLL_UP"],self.state["BTN_START"],self.state["BTN_BACK"],self.state["BTN_SCROLL_DOWN"]]
    
    def getCharger(self):
        """ Update values for charger/battery related info in self.state dictionary. """
        self.port.write("getcharger\n")
        self.readResponseAndUpdateState()

    def setBacklight(self, value):
        if value > 0:
            self.port.write("setled backlighton")
        else:
            self.port.write("setled backlightoff")
        self.readResponseString()
        
def drive():

  if ((x==0 and y==0) and (x_last==0 and y_last==0)):
    print "NOPE"
  else:
    robot.setMotors(x,y,z)

def joy_handler(ps):
  x=ps.x
  y=ps.y
  z=ps.z
  print x,y,z
  drive()
  x_last=x
  y_last=y

def listener():
  rate = rospy.Rate(5)
  while not rospy.is_shutdown():
    rospy.Subscriber("/joy01", Vector3, joy_handler)
    #drive()
    rate.sleep()

if __name__ == '__main__':
  settings = termios.tcgetattr(sys.stdin)
  rospy.init_node('wizard01', anonymous=True)
  port = rospy.get_param('~port', "/dev/ttyACM0")
  robot = Botvac(port)
  global x
  global y
  global z
  x=0
  y=0
  z=0
  global x_last
  global y_last
  global z_last
  x_last=0
  y_last=0
  z_last=0
  try:
    print msg
    listener()
  except rospy.ROSInterruptException:
    pass
