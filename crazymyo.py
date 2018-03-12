from __future__ import print_function

import sys
import logging
import time

from threading import Thread
from threading import Timer

import cflib.crtp
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.positioning.motion_commander import MotionCommander
#from cflib.crazyflie.commander import Commander

from gestures import Gesture
gesture = Gesture() #holds the gestures Queue

import os
import signal

#myo imports

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

roll = 1
pitch = 2
yaw = 3

#poses from pilot perspective #remove this code...
DOUBLE_TAP = libmyo.Pose.double_tap
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

#myo globals
restingRoll = 0  #resting orientation
restingPitch = 0  #resting orientation
restingYaw = 0  #resting orientation
minRot = 0.35 # must rotate arm at least this many RADIANS for gesture detection
inPose = False #flag to see if a pose is currently active

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
        print('\r\n' + ''.join('[{0}]'.format(p) for p in parts), end='')
        sys.stdout.flush()

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
        # print("Pose is: ")
        # print(pose)

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
        print('\r\n' + ''.join('[{0}]'.format(p) for p in parts), end='')
        sys.stdout.flush()

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

        # print("on_pose: Pose is: ")
        # print (pose)
        # print("\n\n")
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
                gesture.add_gesture((curPose,pitch,deltaPitch))

                print("Altitude change - Pitch angle: " + str(math.degrees(deltaPitch))+"\n")
            elif curPose == libmyo.Pose.fist: #forward/backward
                inPose = True
                gesture.add_gesture((curPose,pitch, deltaPitch))

                print("Move forward/backward - Pitch angle: " + str(math.degrees(deltaPitch))+"\n")

        if abs(deltaRoll) >= minRot and abs(deltaRoll) > abs(deltaYaw) and abs(deltaRoll) > abs(deltaPitch) and inPose == False:
            if curPose == libmyo.Pose.fist:
                inPose = True

                gesture.add_gesture((curPose,roll, deltaRoll))


                print("ROLL! - Roll angle: " + str(math.degrees(deltaRoll))+"\n")

        if abs(deltaYaw) >= minRot and abs(deltaYaw) > abs(deltaPitch) and abs(deltaYaw) > abs(deltaRoll) and inPose == False:
            if curPose == libmyo.Pose.fist:#left right motion
                inPose = True

                gesture.add_gesture((curPose,yaw,deltaYaw))


                print("left/right - Yaw angle: " + str(math.degrees(deltaYaw))+"\n")

            elif curPose == libmyo.Pose.fingers_spread: #yaw drone
                inPose = True

                gesture.add_gesture((curPose,yaw, deltaYaw))

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
        print("Thank you for calibrating")



URI = 'radio://0/80/2M'

# Only output errors from the logging framework
logging.basicConfig(level=logging.ERROR)


class FlightCtrl:

    def __init__(self, scf):

        #global gesture
        #self.cf = scf.cf
        #self.cmdr = Commander(self.cf)
        scf.open_link()
    

    def perform_gesture(self, g_id, mc):

        d = 0.3

        print("\nID")
        print(g_id[0])
        print("\n")

        if g_id[0] == DOUBLE_TAP:
            print("Double tap detected")
            if mc._is_flying: 
                mc.stop()
                pass
            else:
                mc.take_off()
                #print("We'd take off here")
        # Add new gestures here

        elif g_id[1] == roll:
            print("roll")
            mc.move_distance(0,math.copysign(d, g_id[2]),0)
        
        elif g_id[1] == pitch:
            print ('pitch')
            mc.move_distance(math.copysign(d, g_id[2]),0,0)

        elif g_id[1] == yaw:   
            print ('yaw') 
            if g_id[2] < 0:
                mc.turn_left(g_id[2])
            else:
                mc.turn_right(g_id[2])

        elif g_id[1] == 'LAND':
            mc.land()
        else:
            if mc._is_flying:
                mc.stop()
            print("Unknown gesture")

    def gesture_ctrl(self, scf, fc, g):
        global gesture
        mc = MotionCommander(scf)
        mc._reset_position_estimator()

        try:

            while True:

                g_id = gesture.get_gesture()

                if g_id is not None:
                    print("Gesture detected: ")
                    print(g_id)
                    fc.perform_gesture(g_id, mc)
                
        except Exception, e:
            print (str(e))
            scf.close_link()

#Myo events class
class Myo:

    def __init__(self):
        #global gesture
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
