# -*- coding: utf-8 -*-
#
#     ||          ____  _ __
#  +------+      / __ )(_) /_______________ _____  ___
#  | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
#  +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
#   ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
#
#  Copyright (C) 2014 Bitcraze AB
#
#  Crazyflie Nano Quadcopter Client
#
#  This program is free software; you can redistribute it and/or
#  modify it under the terms of the GNU General Public License
#  as published by the Free Software Foundation; either version 2
#  of the License, or (at your option) any later version.
#
#  This program is distributed in the hope that it will be useful,
#  but WITHOUT ANY WARRANTY; without even the implied warranty of
#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#  GNU General Public License for more details.
#  You should have received a copy of the GNU General Public License
#  along with this program; if not, write to the Free Software
#  Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston,
#  MA  02110-1301, USA.


import logging
import time
import math
from threading import Thread
from threading import Timer

import cflib
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig

from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.positioning.motion_commander import MotionCommander

import matplotlib.pyplot as plt
import matplotlib.animation as animation

logging.basicConfig(level=logging.ERROR)


class displayStb (object):
    
    tsInit = 0

    def __init__(self,ax1,ax2,ax3):
        self.ax1 = ax1
        self.ax2 = ax2
        self.ax3 = ax3
        self.ax1.plot(0,0)
        self.ax2.plot(0,0)
        self.ax3.plot(0,0)
        self.tsInit = 0

    def update(self,arb):
        graph_data = open('SensorMaster.txt','r').read()
        lines = graph_data.split('\n')
        timescale = []
        stbRoll = []
        stbYaw = []
        stbPitch = []
        j=1

        for line in lines:
            if len(lines)-100 > j:
                j=j+1
                continue
            if len(line) > 2:
                if self.tsInit == 0:
                    ts, roll, yaw, pitch = line.split(',')
                    timescale.append(ts)
                    stbRoll.append(roll)
                    stbYaw.append(yaw)
                    stbPitch.append(pitch)
                    self.tsInit = ts
                else:
                    ts, roll, yaw, pitch = line.split(',')
                    timescale.append(str(int(ts) - int(self.tsInit)))
                    stbRoll.append(roll)
                    stbYaw.append(yaw)
                    stbPitch.append(pitch)
            j=j+1

        self.ax1.clear()
        self.ax2.clear()
        self.ax3.clear()
        self.ax1.plot(timescale,stbRoll)
        self.ax3.plot(timescale,stbYaw)
        self.ax2.plot(timescale,stbPitch)


class MotorRun:

    yawInit = 0
    yawCurr = 0
    _theta = 0
    _rtCoef = [1,0]
    _lfCoef = [-1,0]
    _fwCoef = [0,1]
    _bkCoef = [0,-1]

    def __init__(self,cfobject):
        #get initial orientation

        # We take off when the commander is created
        with MotionCommander(cfobject) as mc:
            #update current Yaw angle at 100 Hz
            threading.Timer(10.0, self._updateYawCurr).start()
            #use seperate thread for motor operation
            Thread(target = self._quadMotion(mc)).start()

    def _updateYawCurr(self):

        #get the most updated yaw angle from storage text file
        with open('SensorMaster.txt','r') as stbFile:
            stbLines = stbFile.readlines()
        
        n=1
        while len(stbLines[len(stbLines) - n]) < 2:
            n+=1
        currAttitude = stbLines[len(stbLines - n)]
        yawCurr = currAttitude['stabilizer.yaw']
        #update all coefficients after updating the yaw angle
        coef = self._updateCoef

    def _updateCoef(self):
        self._theta = self.yawCurr - self.yawInit
        self._rtCoef = [cos(self._theta), -sin(self._theta)]
        self._lfCoef = [-cos(self._theta), sin(self._theta)]
        self._fwCoef = [sin(self._theta), cos(self._theta)]
        self._bkCoef = [-sin(self._theta), -cos(self._theta)]


    def _quadMotion(self,mc):
        #do stuff here
        
        #from example (hovering)
        time.sleep(1)

        # There is a set of functions that move a specific distance
        # We can move in all directions
        
        time.sleep(1)

        mc.up(0.1)
        mc.down(0.1)
        time.sleep(1)

        
        # And we can stop
        mc.stop()
    


if __name__ == '__main__':

    #Definition
    URI="radio://0/80/2M"
    ifstart = 0

    #file clean up
    myfile = open('StabilizerData.txt', 'w')
    myfile.write('')
    myfile.close()
    myfile = open('AccelerometerData.txt', 'w')
    myfile.write('')
    myfile.close()
    myfile = open('GyroscopeData.txt', 'w')
    myfile.write('')
    myfile.close()
    myfile = open('SensorMaster.txt', 'w')
    myfile.write


    #Wait for instructiosn to fly	
    while (ifstart == 0):
        ifstart = input('Enter "1" to search for Crazyflie\n')

    # Initialize the low-level drivers (don't list the debug drivers)
    cflib.crtp.init_drivers(enable_debug_driver=False)
    
    # Scan to ensure our quadcopter can be found through radio
    print('Scanning interfaces for Crazyflies...')
    available = cflib.crtp.scan_interfaces()
    print('Crazyflies found')

    """
    for i in available:
        print(i[0])
    """

    if len(available) > 0:       
        #le = MotorRampExample(available[0][0])        
       
        #make connection and start saving sensor log
        with SyncCrazyflie(URI) as scf:

            #create class to initialize quadcopter movement
            mr = MotorRun(scf)

            #run GUI for live plots in main thread
            fig = plt.figure()
            axRoll = fig.add_subplot(3,1,1)
            axYaw = fig.add_subplot(3,1,2)
            axPitch = fig.add_subplot(3,1,3)
            animate = displayStb(axRoll,axYaw,axPitch)
            ani = animation.FuncAnimation(fig, animate.update, interval=20)
            plt.show()
    else:
        print('No Crazyflies found, cannot run example')


