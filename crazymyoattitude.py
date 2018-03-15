from __future__ import print_function

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

"""
Gestures SUPPORTED
1) If not flying, double tap once to start flying
2) If flying, double tap once to hover
3) If Flying, double tap twice to land
4) If flying, a rest pose should trigger hover
5) Yaw + fist = translate left/RIGHT
6) yaw + fingers spread = YAW
7) roll + fist = roll? Do a flip if the angle is over some to be determined later amount
8) pitch + fist = translate forward or backward depending on the quads orientation
9) pitch + fingers spread = adjust Altitude

Gestures are sent in the following format:
    (myo_pose, rotation_type, rotation_angle)
myo_pose is one of:
    1) libmyo.Pose.double_tap
    2) libmyo.Pose.fingers_spread
    3) libmyo.Pose.rest
    4) libmyo.Pose.fist
    5) LAND (to accommodate landing)
rotation_type is one of
    1) roll
    2) pitch
    3) yaw
rotation_angle is an angle in radians
For double taps and rests, the last two variables can safely be ignored.

"""

roll_id = 1
pitch_id = 2
yaw_id = 3

DOUBLE_TAP = libmyo.Pose.double_tap
FIST = libmyo.Pose.fist
FINGERS_SPREAD = libmyo.Pose.fingers_spread

#myo globals
restingRoll = 0  #resting orientation
restingPitch = 0  #resting orientation
restingYaw = 0  #resting orientation
minRot = 0.35 # must rotate arm at least this many RADIANS for gesture detection
inPose = False #flag to see if a pose is currently active

# These need to be reworked

LAND = 95 #triggered by two consecutive double tap poses
UP = 3 #negative pitch angle
DOWN = 4 #positive pitch angle
LEFT = 5 #positive angle
RIGHT = 6 #negative angle
FORWARD = 7 #downward motion = positive pitch angle
BACKWARD = 8 #upward motion = negative pitch angle
TILT_RIGHT = 9 #CW ROTATION POS ANGLES
TILT_LEFT = 10 #CCW ROTATION  NEG ANGLES
YAW_RIGHT = 11 #negative angle
YAW_LEFT = 12 #positive angle
REST = 13 #hover like with double tap

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
        global inPose
        global gesture

        if pose == libmyo.Pose.rest:
            inPose = False
            gesture.add_gesture((pose,0,0))

        if pose == libmyo.Pose.double_tap: #double tap detected

            myo.set_stream_emg(libmyo.StreamEmg.enabled)
            self.emg_enabled = True
            if self.pose != libmyo.Pose.double_tap:
                print("Double tap detected")
                inPose = False

                gesture.add_gesture((pose,0,0))

            else: #this means we want to land if we get two double taps in a row
                inPose = False
                gesture.add_gesture((LAND,0,0))


        elif pose == libmyo.Pose.fingers_spread:
            myo.set_stream_emg(libmyo.StreamEmg.disabled)
            self.emg_enabled = False
            self.emg = None
        self.pose = pose
        if pose != libmyo.Pose.rest:
            self.output()


    def on_orientation_data(self, myo, timestamp, orientation):
        global restingYaw
        global restingRoll
        global restingPitch
        global inPose
        global gesture
        self.orientation = orientation
        curPose = self.pose
        lastQuat = orientation #last orientation in quarternion
        roll = math.atan2(2.0*(lastQuat[3]*lastQuat[0] + lastQuat[1]*lastQuat[2]), 1.0 - 2.0*(lastQuat[0]*lastQuat[0] + lastQuat[1] * lastQuat[1]))
        pitch = math.asin(max(-1.0, min(1.0, 2.0*(lastQuat[3]*lastQuat[1] - lastQuat[2]*lastQuat[0]))))
        yaw = math.atan2(2.0*(lastQuat[3]*lastQuat[2] + lastQuat[0]*lastQuat[1]), 1.0 - 2.0*( lastQuat[1]*lastQuat[1] + lastQuat[2]*lastQuat[2]))

        deltaRoll = roll - restingRoll
        deltaPitch = pitch - restingPitch
        deltaYaw = yaw - restingYaw

        if abs(deltaPitch) >= minRot and abs(deltaPitch) > abs(deltaYaw) and abs(deltaPitch) > abs(deltaRoll) and inPose == False:
            if curPose == libmyo.Pose.fingers_spread: #UP DOWN
                inPose = True
                gesture.add_gesture((curPose,pitch_id,deltaPitch))

                print("Altitude change - Pitch angle: " + str(math.degrees(deltaPitch))+"\n")
            elif curPose == libmyo.Pose.fist: #forward/backward
                inPose = True
                gesture.add_gesture((curPose,pitch_id, deltaPitch))

                print("Move forward/backward - Pitch angle: " + str(math.degrees(deltaPitch))+"\n")

        if abs(deltaRoll) >= minRot and abs(deltaRoll) > abs(deltaYaw) and abs(deltaRoll) > abs(deltaPitch) and inPose == False:
            if curPose == libmyo.Pose.fist:
                inPose = True

                #gesture.add_gesture((curPose,roll_id, deltaRoll))


                #print("ROLL! - Roll angle: " + str(math.degrees(deltaRoll))+"\n")

        if abs(deltaYaw) >= minRot and abs(deltaYaw) > abs(deltaPitch) and abs(deltaYaw) > abs(deltaRoll) and inPose == False:
            if curPose == libmyo.Pose.fist:#left right motion
                inPose = True

                gesture.add_gesture((curPose,yaw_id,deltaYaw))


                print("left/right - Yaw angle: " + str(math.degrees(deltaYaw))+"\n")

            elif curPose == libmyo.Pose.fingers_spread: #yaw drone
                inPose = True

                gesture.add_gesture((curPose,yaw_id, deltaYaw))

                print("YAW DRONE left/right - Yaw angle: " + str(math.degrees(deltaYaw))+"\n")


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
        myo.vibrate ('long')
        myo.vibrate ('short')
        print("Thank you for calibrating")



URI = 'radio://0/80/2M'

# Only output errors from the logging framework
logging.basicConfig(level=logging.ERROR)


class FlightCtrl:

    def __init__(self, scf):

        scf.open_link()


    def perform_gesture(self, g_id, mc):
        #initialize variables for attitude tracking
        yawInit = 0
        yawCurr = 0
        theta = 0
        rtCoef = [0,-1]
        lfCoef = [0,1]
        fwCoef = [1,0]
        bkCoef = [-1,0]
        levelSpeed = 0.3

        #initialize fixed movement distance
        d = 0.3

        if g_id[1] == roll_id:
            print("Corresponds to roll")
            print(g_id[0])
        elif g_id[1] == pitch_id:
            print("Corresponds to pitch")
            print(g_id[0])
        elif g_id[1] == yaw_id:
            print("Corresponds to yaw")
            print(g_id[0])

        if g_id[0] == DOUBLE_TAP:
            if mc._is_flying:
                print("Hovering...")
                mc.stop()
            else:
                print("Taking off...")
                mc.take_off()
            self.resetYawInit()

        elif g_id[0] == FIST and g_id[1] == yaw_id:
            print("Roll...")
            #mc.move_distance(0,math.copysign(d, g_id[2]),0)
            if (g_id[2] > 0)
                #turn right
                mc.move_distance(self.d * self.rtCoef[0], self.d * self.rtCoef[1], 0)
            else
                #turn left
                mc.move_distance(self.d * self.lfCoef[0], self.d * self.lfCoef[1], 0)

        elif g_id[0] == FIST and g_id[1] == pitch_id:
            print("Pitch...")
            #mc.move_distance(math.copysign(d, g_id[2]), 0, 0)
            if (g_id[2] < 0)
                #move forward
                mc.move_distance(self.d * self.fwCoef[0], self.d * self.fwCoef[1], 0)
            else
                #move backward
                mc.move_distance(self.d * self.bkCoef[0], self.d * self.bkCoef[1], 0)

        elif g_id[0] == FINGERS_SPREAD and g_id[1] == pitch_id:

            if g_id[2] < 0:
                #if mc._thread.get_height() + d < mc.max_height:
                print ("Up...")
                mc.up(d)
                #else:
                    #print("Max. height" + mc.max_height + "m reached: requested height: " + (mc._thread.get_height() + d))
            else:
                #if mc._thread.get_height() - d < mc.min_height:
                print("Down...")
                mc.down(d)
                #else:
                #    print("Min. height" + mc.min_height + "m reached: requested height: " + (mc._thread.get_height() - d))

        elif g_id[0] == FINGERS_SPREAD and g_id[1] == yaw_id:
            print ('Yaw...')
            if g_id[2] < 0:
                mc.turn_left(30)
            else:
                mc.turn_right(30)

        elif g_id[1] == LAND:
            print("Landing...")
            mc.land()
        else:
            if mc._is_flying:
                mc.stop()

    """Functions to update attitude by reading storage text file"""
    def updateCoef(self):

        self.theta = (self.yawCurr - self.yawInit)*(3.1415926/180)
        #print('current angle:')
        #print(self.theta)
        #print('\n')
        self.rtCoef = [-sin(self.theta), -cos(self.theta)]
        self.lfCoef = [sin(self.theta), cos(self.theta)]
        self.fwCoef = [cos(self.theta), -sin(self.theta)]
        self.bkCoef = [-cos(self.theta), sin(self.theta)]

    def updateYawCurr(self):
        with open('SensorMaster.txt','r') as stbFile:
            stbLines = stbFile.readlines()

        currAttitude = stbLines[len(stbLines)-1]
        dumb1, dumb2, currentYaw, dumb3 = currAttitude.split(',')
        self.yawCurr = float(currentYaw)
        #update all coefficients after updating the yaw angle
        coef = self.updateCoef()

    def resetYawInit(self):
         #set recalibrate initial Yaw value
        with open('SensorMaster.txt','r') as stbFile:
            stbLines = stbFile.readlines()

        while len(stbLines) == 0:
            with open('SensorMaster.txt','r') as stbFile:
                stbLines = stbFile.readlines()

        print("yaw angle recalibrated :")
        newInitAttitude = stbLines[len(stbLines)-1]

        dumb1, dumb2, initYaw, dumb3 = newInitAttitude.split(',')
        print(initYaw)
        self.yawInit = float(initYaw)

    def _updateYaw(self, scf):
        try:
            while True:
                keepUpdating = self.updateYawCurr();
                time.sleep(0.1)

        except Exception, e:
            print str(e)
            scf.close_link()

    def gesture_ctrl(self, scf, fc, g):
        global gesture
        mc = MotionCommander(scf)
        mc._reset_position_estimator()

        self.resetYawInit()
        threadUpdate = Thread(tartget = self._updateYaw, argu = (scf,))
        threadUpdate.start()

        try:

            while True:

                g_id = gesture.get_gesture()

                if g_id is not None:
                    #print("Gesture detected: ")
                    #print(g_id)
                    fc.perform_gesture(g_id, mc)

        except Exception, e:
            print (str(e))
            scf.close_link()

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
                time.sleep(0.25)
        except KeyboardInterrupt:
            print("\nQuitting ...")
            #perhaps try landing here too
        #except :

        finally:
            print("Shutting down hub...")
            hub.shutdown()


def main():

    try:

        cflib.crtp.init_drivers(enable_debug_driver=False)

        scf = SyncCrazyflie(URI)
        fc = FlightCtrl(scf)
        m = Myo()

        Thread(target = fc.gesture_ctrl, args = (scf,fc, gesture)).start()
        Thread(target = m.gesture_detection, args = (gesture,)).start()

        while True:
            time.sleep(0.5)



    except KeyboardInterrupt:
        print('\nClosing link...')
        #perhaps land gracefully here
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
