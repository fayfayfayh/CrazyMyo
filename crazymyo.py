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
1) If not flying, double tap once to start flying
2) double tap once to recalibrate
3) If Flying, double tap twice to land
4) If flying, a rest pose should trigger hover
5) Yaw = translate left/RIGHT
6) yaw + fist = YAW
7) pitch = translate forward or backward depending on the quads orientation
8) pitch + fingers spread = adjust Altitude


VIBRATION MEANINGS:
1) SHORT and short fingers_spread
2) long and long - FIST
3) LONG AND SHORT - double tap!

Gestures are sent in the following format:
    (myo_pose, rotation_type, rotation_angle)
myo_pose is one of:
    1) DOUBLE_TAP
    2) FINGERS_SPREAD
    3) REST
    4) FIST
    5) LAND (to accommodate landing)
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
MIN_VBAT = 3.15

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
        # TODO: Should clean up these comments
        ctime = time.time()
        if (ctime - self.last_time) < self.interval:
            return
        self.last_time = ctime

        parts = []
        # if self.orientation:
        #     for comp in self.orientation:
        #         parts.append(str(comp).ljust(15))



        # if self.acceleration:
        #     for comp in self.acceleration:
        #         parts.append(str(comp).ljust(15))

        parts.append(str(self.pose).ljust(10))
        # parts.append('E' if self.emg_enabled else ' ')
        # parts.append('L' if self.locked else ' ')
        # parts.append(self.rssi or 'NORSSI')
        # if self.emg:
        #     for comp in self.emg:
        #         parts.append(str(comp).ljust(5))
        #print('\r\n' + ''.join('[{0}]'.format(p) for p in parts), end='')
        #sys.stdout.flush()

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
            print("Detected default orientation [" + str(restingRoll) + "," + str(restingPitch) + "," + str(restingYaw) +"]\n")
            self.output()
            myo.vibrate ('long')
            myo.vibrate ('short')
            return False

        elif pose == libmyo.Pose.fingers_spread:
            myo.set_stream_emg(libmyo.StreamEmg.disabled)
            self.emg_enabled = False
            self.emg = None
        #if pose == libmyo.Pose.double_tap
        self.pose = pose
        #self.pose =
        #if the pose is fingers spread, then turn on flag for fingers fingers_spread
        #if pose is fist turn on flag for
        #cancel a pose and hover with double_tap
        #if pose is
        #If pose is rest then hover

        self.output()


    def on_orientation_data(self, myo, timestamp, orientation):
        self.orientation = orientation
        #self.output()

    def on_accelerometor_data(self, myo, timestamp, acceleration):
        self.acceleration = acceleration
        #accelData.append([acceleration[0],acceleration[1], acceleration[2]])



    def on_gyroscope_data(self, myo, timestamp, gyroscope):
        self.gyroscope = gyroscope

    def on_emg_data(self, myo, timestamp, emg):
        self.emg = emg
        #self.output()

    def on_unlock(self, myo, timestamp):
        self.locked = False
        #self.output()

    def on_lock(self, myo, timestamp):
        self.locked = True
        #self.output()

    def on_event(self, kind, event):
        """
        Called before any of the event callbacks.
        """

    def on_event_finished(self, kind, event):
        """
        Called after the respective event callbacks have been
        invoked. This method is *always* triggered, even if one of
        the callbacks requested the stop of the Hub.
        """

    def on_pair(self, myo, timestamp, firmware_version):
        """
        Called when a Myo armband is paired.
        """

    def on_unpair(self, myo, timestamp):
        """
        Called when a Myo armband is unpaired.
        """
        #land gracefully

    def on_disconnect(self, myo, timestamp):
        """
        Called when a Myo is disconnected.
        """
        #land gracefully

    def on_arm_sync(self, myo, timestamp, arm, x_direction, rotation,
                    warmup_state):
        """
        Called when a Myo armband and an arm is synced.
        """

    def on_arm_unsync(self, myo, timestamp):
        """
        Called when a Myo armband and an arm is unsynced.
        """

    def on_battery_level_received(self, myo, timestamp, level):
        """
        Called when the requested battery level received.
        """

    def on_warmup_completed(self, myo, timestamp, warmup_result):
        """
        Called when the warmup completed.
        """


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
        # TODO: should clean up these comments
        ctime = time.time()
        if (ctime - self.last_time) < self.interval:
            return
        self.last_time = ctime

        parts = []
        parts.append(str(self.pose).ljust(10))

        #print('\r\n' + ''.join('[{0}]'.format(p) for p in parts), end='')
        #sys.stdout.flush()

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

        if pose == libmyo.Pose.double_tap: #double tap detected
            consecDoubleTaps = consecDoubleTaps + 1
            myo.set_stream_emg(libmyo.StreamEmg.enabled)
            self.emg_enabled = True
            if consecDoubleTaps < 2:
                print("Double tap detected: Recalibration!\n")
                inPose = False

                lastQuat = self.orientation #last orientation in quarternion
                restingRoll = math.atan2(2.0*(lastQuat[3]*lastQuat[0] + lastQuat[1]*lastQuat[2]), 1.0 - 2.0*(lastQuat[0]*lastQuat[0] + lastQuat[1] * lastQuat[1]))
                restingPitch = math.asin(max(-1.0, min(1.0, 2.0*(lastQuat[3]*lastQuat[1] - lastQuat[2]*lastQuat[0]))))
                restingYaw = math.atan2(2.0*(lastQuat[3]*lastQuat[2] + lastQuat[0]*lastQuat[1]), 1.0 - 2.0*( lastQuat[1]*lastQuat[1] + lastQuat[2]*lastQuat[2]))
                print("Detected default orientation [" + str(restingRoll) + "," + str(restingPitch) + "," + str(restingYaw) +"]\n")
                #self.output()
                myo.vibrate ('long')
                myo.vibrate ('short')
                gesture.add_gesture((DOUBLE_TAP,0,0))



            else: #this means we want to land if we get two double taps in a row
                inPose = False
                myo.vibrate ('long')
                myo.vibrate ('short')
                gesture.add_gesture((LAND,0,0))
                consecDoubleTaps = 0


        elif pose == libmyo.Pose.fingers_spread:
            myo.vibrate ('short')
            myo.vibrate ('short')
        elif pose == libmyo.Pose.fist:
            myo.vibrate ('long')
            myo.vibrate ('long')

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

        deltaRoll = roll - restingRoll
        deltaPitch = pitch - restingPitch
        deltaYaw = yaw - restingYaw

        if abs(deltaPitch) >= minRot and abs(deltaPitch) > abs(deltaYaw):
            if curPose == libmyo.Pose.fingers_spread: #UP DOWN
                consecDoubleTaps = 0

                if inMotion == False: #if quad is moving - don't add gesture to queue
                    gesture.add_gesture((FINGERS_SPREAD,pitch_id,deltaPitch))
                    print("Altitude change - Pitch angle: " + str(math.degrees(deltaPitch))+"\n")


            else: #forward/backward
                consecDoubleTaps = 0
                if inMotion == False: #if quad is moving - don't add gesture to queue
                    gesture.add_gesture((NO_POSE,pitch_id, deltaPitch))
                    print("Move forward/backward - Pitch angle: " + str(math.degrees(deltaPitch))+"\n")


        elif abs(deltaYaw) >= minRot and abs(deltaYaw) > abs(deltaPitch):
            if curPose != libmyo.Pose.fist:#left right motion
                consecDoubleTaps = 0
                inPose = True
                if inMotion == False: #if quad is moving - don't add gesture to queue
                    gesture.add_gesture((NO_POSE,yaw_id,deltaYaw))
                    print("left/right - Yaw angle: " + str(math.degrees(deltaYaw))+"\n")


            elif curPose == libmyo.Pose.fist: #yaw drone
                consecDoubleTaps = 0
                inPose = True
                if inMotion == False: #if quad is moving - don't add gesture to queue
                    gesture.add_gesture((FIST,yaw_id, deltaYaw))
                    print("YAW DRONE left/right - Yaw angle: " + str(math.degrees(deltaYaw))+"\n")


        else: # just Hover
            gesture.add_gesture((REST,0,0))

    def on_unlock(self, myo, timestamp):
        self.locked = False
        #self.output()

    def on_lock(self, myo, timestamp):
        self.locked = True
        #self.output()

    def on_event(self, kind, event):
        """
        Called before any of the event callbacks.
        """

    def on_event_finished(self, kind, event):
        """
        Called after the respective event callbacks have been
        invoked. This method is *always* triggered, even if one of
        the callbacks requested the stop of the Hub.
        """

    def on_pair(self, myo, timestamp, firmware_version):
        """
        Called when a Myo armband is paired.
        """

    def on_unpair(self, myo, timestamp):
        """
        Called when a Myo armband is unpaired.
        """

    def on_disconnect(self, myo, timestamp):
        """
        Called when a Myo is disconnected.
        """

    def on_arm_sync(self, myo, timestamp, arm, x_direction, rotation,
                    warmup_state):
        """
        Called when a Myo armband and an arm is synced.
        """

    def on_arm_unsync(self, myo, timestamp):
        """
        Called when a Myo armband and an arm is unsynced.
        """

    def on_battery_level_received(self, myo, timestamp, level):
        """
        Called when the requested battery level received.
        """

    def on_warmup_completed(self, myo, timestamp, warmup_result):
        """
        Called when the warmup completed.
        """

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

        if g_id[0] == DOUBLE_TAP:
            if self.mc._is_flying:
                print("Hovering...")
                #mc.stop()
                self.resetYawInit()
            else:
                consecDoubleTaps = 0
                print("Taking off...")
                inMotion = True
                self.mc.take_off()
                inMotion = False
                self.resetYawInit()
                threadUpdate = Thread(target = self._updateYaw, args = (self.scf,))
                threadUpdate.start()


        elif g_id[0] == NO_POSE and g_id[1] == yaw_id:
            print("Roll...")
            #mc.move_distance(0,math.copysign(d, g_id[2]),0)
            if (g_id[2] > 0):
                #turn left
                print("turning left")
                inMotion = True
                self.mc.move_distance(self.lvSpeed * self.lfCoef[0], self.lvSpeed * self.lfCoef[1], 0)
                inMotion = False
            else:
                #turn right
                print("turning right")
                inMotion = True
                self.mc.move_distance(self.lvSpeed * self.rtCoef[0], self.lvSpeed * self.rtCoef[1], 0)
                inMotion = False


        elif g_id[0] == NO_POSE and g_id[1] == pitch_id:
            print("Pitch...")
            #mc.move_distance(math.copysign(d, g_id[2]), 0, 0)
            if (g_id[2] < 0):
                #move forward
                print("moving forward")
                inMotion = True
                self.mc.move_distance(self.lvSpeed * self.fwCoef[0], self.lvSpeed * self.fwCoef[1], 0)
                inMotion = False
            else:
                #move backward
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

        elif g_id[0] == FIST and g_id[1] == yaw_id:
            print ('Yaw...')
            if g_id[2] > 0:
                inMotion = True
                self.mc.turn_left(30)
                inMotion = False
            else:
                inMotion = True
                self.mc.turn_right(30)
                inMotion = False

        elif g_id[0] == LAND:
            print("Landing...")
            inMotion = True
            self.mc.land()
            inMotion = False

        else: #rest behaviour
            if self.mc._is_flying:
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

        while True:
            t = int(time.time())
            if scf.vbat is not None:
                if t % 20 == 0: # print every 20 seconds
                    print "Battery voltage: %.2fV" % (scf.vbat,)

                if t % 5 == 0 and scf.vbat > MIN_VBAT and scf.vbat < WARN_VBAT:
                    print "WARN: Battery voltage: %.2fV. Landing soon..." % (scf.vbat,)
                    # would be nice to have some sort of vibration indicator here and below

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
        print('\nShutting down... (main)')

        scf.close_link()
        sys.exit(0)

if __name__ == '__main__':
    main()
