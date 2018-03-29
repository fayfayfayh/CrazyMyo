import sys
import logging
import time

from threading import Thread
from threading import Timer

import cflib.crtp
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.positioning.motion_commander import MotionCommander

from gestures import Gesture
gesture = Gesture() #holds the gestures Queue

import os
import signal

# Myo imports

import myo as libmyo; libmyo.init()
import numpy as np
import math
from math import sin, cos

"""
TIP:
    WHEN PERFORMING SYNC USING MYO CONNECT
    MAKE SURE TO KEEP ARM STRAIGHT AND JUST FLICK WRIST OUT
    IE: WAVE OUT USING JUST YOUR WRIST!


Gestures SUPPORTED
1) If not flying, FIST + PITCH UP TO TAKEOFF
2) Double tap to recalibrate
3) If Flying, FIST + PITCH DOWN TO LAND
4) If flying, a rest pose should trigger hover
5) Yaw = translate left/RIGHT
6) yaw + fingers_spread = YAW
7) pitch = translate forward or backward depending on the quads orientation
8) pitch + fingers spread = adjust Altitude


VIBRATION MEANINGS:
1) SHORT, short and short fingers_spread
2) long and long - FIST
3) LONG AND SHORT - double tap!

Gestures are sent in the following format:
    (myo_pose, rotation_type, rotation_angle)
myo_pose is one of:
    1) DOUBLE_TAP
    2) FINGERS_SPREAD
    3) REST
    4) FIST

rotation_type is one of
    1) roll
    2) pitch
    3) yaw
rotation_angle is an angle in radians
For double taps and rests, the last two variables can safely be ignored.

"""
#rotation_type constants
roll_id = 1
pitch_id = 2
yaw_id = 3

#myo_pose constants
NO_POSE = 5
DOUBLE_TAP = 1
FIST = 2
FINGERS_SPREAD = 3
REST = 4
LAND = 95 #triggered by two consecutive double tap poses

#myo globals
restingRoll = 0  #resting orientation
restingPitch = 0  #resting orientation
restingYaw = 0  #resting orientation
minRot = 0.785 # must rotate arm at least this many RADIANS for gesture detection
inPose = False #flag to see if a pose is currently active
inMotion = False #flag to see if quad is in motion
consecDoubleTaps = 0 #for landing


WARN_VBAT = 3.3
MIN_VBAT = 3.25

#Myo event listener- specialized for calibration type behaviour
class Calibration_Listener(libmyo.DeviceListener):
    """
    Listener implementation for calibration. Return False from any function to
    stop the Hub.
    """

    interval = 0.05  # Output only 0.05 seconds

    def __init__(self):
        super(Calibration_Listener, self).__init__()
        self.orientation = None
        self.acceleration = None
        self.gyroscope = None
        self.pose = libmyo.Pose.rest
        self.emg_enabled = False
        self.locked = False
        self.rssi = None
        self.emg = None
        self.last_time = 0

    def output(self):
        ctime = time.time()
        if (ctime - self.last_time) < self.interval:
            return
        self.last_time = ctime

        parts = []

        parts.append(str(self.pose).ljust(10))

    def on_connect(self, myo, timestamp, firmware_version):
        myo.vibrate('short')
        myo.vibrate('short')
        myo.request_rssi()
        myo.request_battery_level()

    def on_rssi(self, myo, timestamp, rssi):
        self.rssi = rssi
        self.output()

    def on_pose(self, myo, timestamp, pose):
        global restingYaw
        global restingRoll
        global restingPitch

        if pose == libmyo.Pose.double_tap:
            myo.set_stream_emg(libmyo.StreamEmg.enabled)
            self.emg_enabled = True
            #we start the calibration now
            lastQuat = self.orientation #last orientation in quarternion
            restingRoll = math.atan2(2.0*(lastQuat[3]*lastQuat[0] + lastQuat[1]*lastQuat[2]), 1.0 - 2.0*(lastQuat[0]*lastQuat[0] + lastQuat[1] * lastQuat[1]))
            restingPitch = math.asin(max(-1.0, min(1.0, 2.0*(lastQuat[3]*lastQuat[1] - lastQuat[2]*lastQuat[0]))))
            restingYaw = math.atan2(2.0*(lastQuat[3]*lastQuat[2] + lastQuat[0]*lastQuat[1]), 1.0 - 2.0*( lastQuat[1]*lastQuat[1] + lastQuat[2]*lastQuat[2]))
            if restingYaw < 0: #if angle is negative - in interval (0,-180], change to 360 degree equivalent
                restingYaw = math.pi*2 + restingYaw
            print("Detected default orientation [" + str(restingRoll) + "," + str(restingPitch) + "," + str(restingYaw) +"]\n")
            self.output()
            myo.vibrate ('long')
            myo.vibrate ('short')
            return False

        elif pose == libmyo.Pose.fingers_spread:
            myo.set_stream_emg(libmyo.StreamEmg.disabled)
            self.emg_enabled = False
            self.emg = None
        self.pose = pose

        self.output()


    def on_orientation_data(self, myo, timestamp, orientation):
        self.orientation = orientation

    def on_accelerometor_data(self, myo, timestamp, acceleration):
        self.acceleration = acceleration

    def on_gyroscope_data(self, myo, timestamp, gyroscope):
        self.gyroscope = gyroscope

    def on_emg_data(self, myo, timestamp, emg):
        self.emg = emg

    def on_unlock(self, myo, timestamp):
        self.locked = False

    def on_lock(self, myo, timestamp):
        self.locked = True



#MAIN FLIGHT CONTROL MYO EVENT LISTENER
class Listener(libmyo.DeviceListener):
    """
    Listener implementation. Return False from any function to
    stop the Hub.
    """

    interval = 0.05  # Output only 0.05 seconds

    def __init__(self):
        super(Listener, self).__init__()
        self.orientation = None
        self.acceleration = None
        self.gyroscope = None
        self.pose = libmyo.Pose.rest
        self.emg_enabled = False
        self.locked = False
        self.rssi = None
        self.emg = None
        self.last_time = 0


    def output(self):
        ctime = time.time()
        if (ctime - self.last_time) < self.interval:
            return
        self.last_time = ctime

        parts = []
        parts.append(str(self.pose).ljust(10))


    def on_connect(self, myo, timestamp, firmware_version):
        myo.vibrate('short')
        myo.vibrate('short')
        myo.request_rssi()
        myo.request_battery_level()

    def on_rssi(self, myo, timestamp, rssi):
        self.rssi = rssi
        self.output()

    def on_pose(self, myo, timestamp, pose):
        global inPose
        global gesture
        global consecDoubleTaps

        if pose == libmyo.Pose.rest:
            inPose = False
            gesture.add_gesture((REST,0,0))

        if pose == libmyo.Pose.double_tap: # double tap detected
            consecDoubleTaps = consecDoubleTaps + 1
            myo.set_stream_emg(libmyo.StreamEmg.enabled)
            self.emg_enabled = True

            myo.vibrate ('long')
            myo.vibrate ('short')
            print("double tap detected: Drone Yaw Recalibration!\n")
            gesture.add_gesture((DOUBLE_TAP,0,0)) #recal


        elif pose == libmyo.Pose.fingers_spread:
            myo.vibrate ('short')
            myo.vibrate ('short')
            myo.vibrate ('short')
        elif pose == libmyo.Pose.fist: #recali bration
            myo.vibrate ('long')
            myo.vibrate ('long')
            #gesture.add_gesture((FIST,0,0))
            #print("FIST detected: Recalibration!\n")

        self.pose = pose
        if pose != libmyo.Pose.rest:
            self.output()


    def on_orientation_data(self, myo, timestamp, orientation):
        global restingYaw
        global restingRoll
        global restingPitch
        global inPose
        global gesture
        global consecDoubleTaps
        global inMotion
        self.orientation = orientation
        curPose = self.pose
        lastQuat = orientation #last orientation in quarternion
        roll = math.atan2(2.0*(lastQuat[3]*lastQuat[0] + lastQuat[1]*lastQuat[2]), 1.0 - 2.0*(lastQuat[0]*lastQuat[0] + lastQuat[1] * lastQuat[1]))
        pitch = math.asin(max(-1.0, min(1.0, 2.0*(lastQuat[3]*lastQuat[1] - lastQuat[2]*lastQuat[0]))))
        yaw = math.atan2(2.0*(lastQuat[3]*lastQuat[2] + lastQuat[0]*lastQuat[1]), 1.0 - 2.0*( lastQuat[1]*lastQuat[1] + lastQuat[2]*lastQuat[2]))
        if yaw < 0 and not( restingYaw <= math.pi/2 and restingYaw >= 0): #if angle is negative - in interval (0,-180], change to 360 degree equivalent IF resting NOT in [0-90] interval
            yaw = math.pi*2 + yaw
        elif restingYaw >= math.pi*1.5 and yaw >= 0: # else if yaw is positive AND restingPosition is greater than 270 degrees
            yaw = math.pi*2 + yaw

        deltaRoll = roll - restingRoll
        deltaPitch = pitch - restingPitch
        deltaYaw = yaw - restingYaw
        if abs(deltaYaw) > math.pi/2: #temporary failsafe - if we get more than 90 degrees don't make a move
            deltaYaw = 0

        if abs(deltaPitch) >= minRot and abs(deltaPitch) > abs(deltaYaw):
            if curPose == libmyo.Pose.fingers_spread: #UP DOWN
                consecDoubleTaps = 0

                if inMotion == False: #if quad is moving - don't add gesture to queue
                    gesture.add_gesture((FINGERS_SPREAD,pitch_id,deltaPitch))
                    print("Altitude change - Pitch angle: " + str(math.degrees(deltaPitch))+"\n")

            elif curPose == libmyo.Pose.fist: #TAKEOFF AND LAND
                if inMotion == False: #if quad is moving - don't add gesture to queue
                    gesture.add_gesture((FIST,pitch_id,deltaPitch))
                    #print("Altitude c - Pitch angle: " + str(math.degrees(deltaPitch))+"\n")

            else: #forward/backward
                consecDoubleTaps = 0
                if inMotion == False: #if quad is moving - don't add gesture to queue
                    gesture.add_gesture((NO_POSE,pitch_id, deltaPitch))
                    print("Move forward/backward - Pitch angle: " + str(math.degrees(deltaPitch))+"\n")


        elif abs(deltaYaw) >= minRot and abs(deltaYaw) > abs(deltaPitch):
            if curPose != libmyo.Pose.fingers_spread:#left right motion
                consecDoubleTaps = 0
                inPose = True
                if inMotion == False: #if quad is moving - don't add gesture to queue
                    gesture.add_gesture((NO_POSE,yaw_id,deltaYaw))
                    print("left/right - Yaw angle: " + str(math.degrees(deltaYaw))+"\n")


            elif curPose == libmyo.Pose.fingers_spread: #yaw drone
                consecDoubleTaps = 0
                inPose = True
                if inMotion == False: #if quad is moving - don't add gesture to queue
                    gesture.add_gesture((FINGERS_SPREAD,yaw_id, deltaYaw))
                    print("YAW DRONE left/right - Yaw angle: " + str(math.degrees(deltaYaw))+"\n")


        else: # just Hover
            gesture.add_gesture((REST,0,0))

    def on_unlock(self, myo, timestamp):
        self.locked = False
        #self.output()

    def on_lock(self, myo, timestamp):
        self.locked = True
        #self.output()



#Get resting position orientation
def calibrate (hub):
    print("Hold out the arm with the Myo on it straight with your palms facing parallel to the ground\n")
    print ("Hold this position and double tap\n")

    hub.run(1000,Calibration_Listener())

    try:
        while hub.running:
            time.sleep(0.25)
    except KeyboardInterrupt:
        print("\nQuitting ...")
    finally:
        print("Thank you for calibrating")



URI = 'radio://0/80/2M'

# Only output errors from the logging framework
logging.basicConfig(level=logging.ERROR)


class FlightCtrl:

    #initialize variables for attitude tracking
    yawInit = 0
    yawCurr = 0
    theta = 0
    rtCoef = [0,-1]
    lfCoef = [0,1]
    fwCoef = [1,0]
    bkCoef = [-1,0]
    lvSpeed = 0.3


    def __init__(self, _scf):

        self.mc = MotionCommander(_scf, default_height=0.7)
        self.scf = _scf
        self.scf.open_link()

    def perform_gesture(self, g_id):
        global consecDoubleTaps
        global inMotion

        d = 0.3

        if g_id[0] == DOUBLE_TAP: #RECAL
            self.resetYawInit()


        elif g_id[0] == NO_POSE and g_id[1] == yaw_id:
            print("Roll...")

            if (g_id[2] > 0):
                print("turning left")
                inMotion = True
                self.mc.move_distance(self.lvSpeed * self.lfCoef[0], self.lvSpeed * self.lfCoef[1], 0)
                inMotion = False
            else:
                print("turning right")
                inMotion = True
                self.mc.move_distance(self.lvSpeed * self.rtCoef[0], self.lvSpeed * self.rtCoef[1], 0)
                inMotion = False


        elif g_id[0] == NO_POSE and g_id[1] == pitch_id:
            print("Pitch...")

            if (g_id[2] < 0):
                print("moving forward")
                inMotion = True
                self.mc.move_distance(self.lvSpeed * self.fwCoef[0], self.lvSpeed * self.fwCoef[1], 0)
                inMotion = False
            else:
                print("moving backward")
                inMotion = True
                self.mc.move_distance(self.lvSpeed * self.bkCoef[0], self.lvSpeed * self.bkCoef[1], 0)
                inMotion = False


        elif g_id[0] == FINGERS_SPREAD and g_id[1] == pitch_id:

            if g_id[2] > 0:
                if self.mc._thread.get_height() + d < self.mc.max_height:
                    print ("Up...")
                    inMotion = True
                    self.mc.up(d)
                    inMotion = False
                else:
                    print("Max. height %.2fm reached: requested height: %.2f") % (self.mc.max_height, self.mc._thread.get_height() + d)

            else:
                if self.mc._thread.get_height() - d < self.mc.min_height:
                    print("Down...")
                    inMotion = True
                    self.mc.down(d)
                    inMotion = False
                else:
                    print("Max. height %.2fm reached: requested height: %.2f") % (self.mc.max_height, self.mc._thread.get_height() + d)

        elif g_id[0] == FINGERS_SPREAD and g_id[1] == yaw_id:
            print ('Yaw...')
            if g_id[2] > 0:
                inMotion = True
                self.mc.turn_left(30)
                inMotion = False
            else:
                inMotion = True
                self.mc.turn_right(30)
                inMotion = False

        elif g_id[0] == FIST and g_id[1] == pitch_id:

                if self.mc._is_flying and g_id[2] < 0: #land in this case
                    print("Landing...")
                    inMotion = True
                    self.mc.land()
                    inMotion = False

                elif self.mc._is_flying == False and g_id[2] > 0: #takeoff
                    consecDoubleTaps = 0
                    print("Taking off...")
                    inMotion = True
                    self.mc.take_off()
                    inMotion = False
                    self.resetYawInit()
                    threadUpdate = Thread(target = self._updateYaw, args = (self.scf,))
                    threadUpdate.start()

        elif g_id[0] == LAND: #deprecated TODO TAKE THIS OUT
            print("Landing...")
            inMotion = True
            self.mc.land()
            inMotion = False

        else: #rest behaviour
            if self.mc._is_flying: # If we're not flying it won't do anything, so can ignore that case
                inMotion = True
                self.mc.stop()
                inMotion = False

    """Functions to update attitude by reading storage text file"""
    def updateCoef(self):
        try:
            self.theta = (self.yawCurr - self.yawInit)*(3.1415926/180)
            self.rtCoef = [-sin(self.theta), -cos(self.theta)]
            self.lfCoef = [sin(self.theta), cos(self.theta)]
            self.fwCoef = [cos(self.theta), -sin(self.theta)]
            self.bkCoef = [-cos(self.theta), sin(self.theta)]
        except Exception, e:
            print str(e)
            print("Update failed")

    def updateYawCurr(self):
        try:
            with open('SensorMaster.txt','r') as stbFile:
                stbLines = stbFile.readlines()

            currAttitude = stbLines[len(stbLines)-1]
            currentYaw = currAttitude.split(',')[2]
            self.yawCurr = float(currentYaw)
            #update all coefficients after updating the yaw angle
            coef = self.updateCoef()
        except Exception, e:
            print str(e)
            print("Update current yaw failed")

    def resetYawInit(self):
        try:
             #set recalibrate initial Yaw value
            with open('SensorMaster.txt','r') as stbFile:
                stbLines = stbFile.readlines()

            while len(stbLines) == 0:
                with open('SensorMaster.txt','r') as stbFile:
                    stbLines = stbFile.readlines()

            newInitAttitude = stbLines[len(stbLines)-1]
            initYaw = newInitAttitude.split(',')[2]
            print("Yaw angle recalibrated: %2f") % (float(initYaw),)

            self.yawInit = float(initYaw)
        except Exception, e:
            print str(e)
            print("Reset failed!")

    def _updateYaw(self, scf):
        try:
            while True:
                keepUpdating = self.updateYawCurr();
                time.sleep(0.1)

        except Exception, e:
            print str(e)
            print("Update failed, Landing")
            scf.close_link()

    def gesture_ctrl(self, fc, g):
        global gesture

        self.resetYawInit()

        try:

            while True:

                g_id = gesture.get_gesture()

                if g_id is not None:
                    fc.perform_gesture(g_id)

        except Exception, e:
            print (str(e))
            self.scf.close_link()

#Myo events class
class Myo:

    def __init__(self):
        pass

    def gesture_detection(self, g):

        print("Connecting to Myo ... Use CTRL^C to exit.")
        try:
            hub = libmyo.Hub()
        except MemoryError:
            print("Myo Hub could not be created. Make sure Myo Connect is running.")
            return

        hub.set_locking_policy(libmyo.LockingPolicy.none)

        calibrate(hub) #perform calibration to get resting position orientation
        hub.run(1000, Listener())

        # Listen to keyboard interrupts and stop the hub in that case.
        try:
            while hub.running:
                time.sleep(0.5)
        except KeyboardInterrupt:
            print("\nQuitting ...")

        finally:
            print("Shutting down hub...")
            hub.shutdown()


def main():

    try:

        #logfile reset
        myfile = open('SensorMaster.txt', 'w')
        myfile.write
        myfile.close()

        cflib.crtp.init_drivers(enable_debug_driver=False)

        scf = SyncCrazyflie(URI)

        fc = FlightCtrl(scf)

        m = Myo()

        Thread(target = fc.gesture_ctrl, args = (fc, gesture)).start()
        Thread(target = m.gesture_detection, args = (gesture,)).start()

        prev_t = 0
        while True:
            t = int(time.time())
            if scf.vbat is not None:
                if t % 20 == 0 and t != prev_t: # print every 20 seconds
                    print "Battery voltage: %.2fV" % (scf.vbat,)
                    prev_t = t

                if t % 5 == 0 and t != prev_t and scf.vbat > MIN_VBAT and scf.vbat < WARN_VBAT:
                    print "WARN: Battery voltage: %.2fV. Landing soon..." % (scf.vbat,)
                    prev_t = t

                if t % 5 == 0 and scf.vbat < MIN_VBAT:
                    print "Battery voltage %.2f too low; landing..." % (scf.vbat)
                    fc.mc.land()

                    os.kill(os.getpid(), signal.SIGINT)


            time.sleep(0.1)

    except KeyboardInterrupt:
        print('\nClosing link...')
        scf.close_link()

        print('Shutting down...')
        os.kill(os.getpid(), signal.SIGINT)

    except Exception, e:
        print(str(e))
        print('\nShutting down...')

        scf.close_link()
        sys.exit(0)

if __name__ == '__main__':
    main()
