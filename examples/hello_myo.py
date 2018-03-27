# Copyright (c) 2015  Niklas Rosenstein
# Hacked by Kelvin Chukwu and Fay Huang
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
# THE SOFTWARE.

from __future__ import print_function
#from scipy import integrate

import myo as libmyo; libmyo.init()
#libmyo.init('/Users/fayhuang/Desktop/Capstone/sdk/myo.framework')
import time
import sys
import numpy as np
#import matplotlib
#import matplotlib.pyplot as plt
import math

accelData = []
inCalibration = False
restingRoll = 0  #resting orientation
restingPitch = 0  #resting orientation
restingYaw = 0  #resting orientation
minRot = 0.785 # must rotate arm at least this much for gesture detection try 35 degs first
inPose = False #flag to see if a pose is currently active
consecDoubleTaps = 0 #for landing
isFlying = False




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
            print("[" + str(restingRoll) + "," + str(restingPitch) + "," + str(restingYaw) +"]\n")
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
        accelData.append([acceleration[0],acceleration[1], acceleration[2]])

    def on_unlock(self, myo, timestamp):
        self.locked = False
        #self.output()

    def on_lock(self, myo, timestamp):
        self.locked = True
        #self.output()


    def on_battery_level_received(self, myo, timestamp, level):
        """
        Called when the requested battery level received.
        """

    def on_warmup_completed(self, myo, timestamp, warmup_result):
        """
        Called when the warmup completed.
        """



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
        global consecDoubleTaps
        global isFlying
        if pose == libmyo.Pose.rest:
            inPose = False


        if pose == libmyo.Pose.double_tap: #double tap detected

            myo.set_stream_emg(libmyo.StreamEmg.enabled)
            self.emg_enabled = True


            if consecDoubleTaps < 2:
                if isFlying == True:
                    print ("Double tap detected: Recalibration!\n")
                    inPose = False
                    consecDoubleTaps = consecDoubleTaps + 1
                else:
                    print("Takeoff")
                    isFlying = True
                    consecDoubleTaps = consecDoubleTaps + 1

            else: #this means we want to land if we get two double taps in a row
                consecDoubleTaps = 0
                inPose = False
                isFlying = False
                print("LAND requested - two double taps in a row")





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
        global consecDoubleTaps
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
                inPose = True
                print("Altitude change - Pitch angle: " + str(math.degrees(deltaPitch))+"\n")



            else: #forward/backward
                consecDoubleTaps = 0
                inPose = True
                print("Move forward/backward - Pitch angle: " + str(math.degrees(deltaPitch))+"\n")





        elif abs(deltaYaw) >= minRot and abs(deltaYaw) > abs(deltaPitch):
            if curPose != libmyo.Pose.fist:#left right motion
                consecDoubleTaps = 0
                inPose = True
                print("left/right - Yaw angle: " + str(math.degrees(deltaYaw))+"\n")



            elif curPose == libmyo.Pose.fist: #yaw drone
                consecDoubleTaps = 0
                inPose = True
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


def main():
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
    finally:
        print("Shutting down hub...")
        hub.shutdown()
        #plotting stuff of old
        # numAccelData = np.array(accelData)
        # dataTimes = np.arange(0,(numAccelData.shape[0])*0.02, 0.02) #) timestamps array for accel
        #
        # # numSpeedData = numpy.empty_like(numAccelData)
        # #numSpeedData = numAccelData.copy()*(-9.8)
        #
        # xnumSpeedData = integrate.cumtrapz((numAccelData[:,0]*(-9.8)).squeeze(),dataTimes)#x
        # ynumSpeedData = integrate.cumtrapz((numAccelData[:,1]*(-9.8)).squeeze(),dataTimes)#y
        # znumSpeedData = integrate.cumtrapz((numAccelData[:,2]*(-9.8)).squeeze(),dataTimes)#z
        #
        # #numSpeedData
        #
        # speedDataTimes = np.arange(0,(xnumSpeedData.shape[0])*0.02, 0.02) # speed timestamps
        #
        # fig, ax = plt.subplots()
        #
        # ax.plot(np.arange(0,(numAccelData.shape[0])*0.02, 0.02),numAccelData[:,0]*(-9.8), 'r-', label='x acceleration')
        # ax.plot(np.arange(0,(numAccelData.shape[0])*0.02, 0.02),numAccelData[:,1]*(-9.8), 'b-', label='y acceleration')
        # ax.plot(np.arange(0,(numAccelData.shape[0])*0.02, 0.02),numAccelData[:,2]*(-9.8), 'g-', label='z acceleration')
        #
        # legend = ax.legend()
        #
        # plt.xlabel('Time [s]')
        # plt.ylabel('Acceleration in [m/s^2])')
        # plt.title('Myo Acceleration: logo on top of arm')
        #
        # # plt.title('Accel Data')
        # plt.draw()
        # plt.savefig('acceleration_const_speed_unnormed.png')
        #
        # #Next plot velocity less initial velocity
        # plt.clf()
        #
        # fig, ax = plt.subplots()
        #
        # ax.plot(speedDataTimes,xnumSpeedData, 'r-', label='x speed')
        # ax.plot(speedDataTimes,ynumSpeedData, 'b-', label='y speed')
        # ax.plot(speedDataTimes,znumSpeedData, 'g-', label='z speed')
        #
        # legend = ax.legend()
        #
        # plt.xlabel('Time [s]')
        # plt.ylabel('Speed [m/s]')
        # plt.title('Difference from initial speed')
        #
        # # plt.title('Accel Data')
        # plt.draw()
        # plt.savefig('speed_const_speed_test.png')





if __name__ == '__main__':
    main()
