# -*- coding: utf-8 -*-
#
#     ||          ____  _ __
#  +------+      / __ )(_) /_______________ _____  ___
#  | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
#  +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
#   ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
#
#  Copyright (C) 2017 Bitcraze AB
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
"""
This script shows the basic use of the MotionCommander class.

Simple example that connects to the crazyflie at `URI` and runs a
sequence. This script requires some kind of location system, it has been
tested with (and designed for) the flow deck.

Change the URI variable to your Crazyflie configuration.
"""
import sys
import logging
import time
import pygame
from threading import Thread

import cflib.crtp
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.positioning.motion_commander import MotionCommander
from cflib.positioning.commander import Commander

import matplotlib.pyplot as plt
import matplotlib.animation as animation

URI = 'radio://0/80/2M'

# Only output errors from the logging framework
logging.basicConfig(level=logging.ERROR)

def yaw_test(scf):
    cmdr = Commander(scf.cf)
    cmdr.send_stop_setpoint()
    try:
        while True:
            time.sleep(1)

    except KeyboardInterrupt:
        print('\nShutting down...')
        scf.close_link()

    except Exception, e:
        print str(e)
        scf.close_link()


def key_ctrl(mc, scf):

    print 'Spacebar to start'
    raw_input()
    pygame.display.set_mode((400, 300))

    print 'WASD for throttle & yaw; arrow keys for left/right/forward/backward' 
    print 'Spacebar to land'
    
    vel_x = 0
    vel_y = 0
    vel_z = 0
    yaw_rate = 0

    mc.take_off()

    while True:
        
        for event in pygame.event.get():

            if event.type == pygame.KEYDOWN:

                if event.key == pygame.K_w:
                    vel_z = 0.3

                if event.key == pygame.K_SPACE:  
                    print 'Space pressed, landing'
                    
                    mc.land()
                    time.sleep(2)
                    print 'Closing link...'
                    scf.close_link()
                    print 'Link closed'
                    pygame.quit()
                    sys.exit(0)

                if event.key == pygame.K_a:
                    yaw_rate = -60

                if event.key == pygame.K_s:
                    vel_z = -0.3

                if event.key == pygame.K_d:
                    yaw_rate = 60

                if event.key == pygame.K_UP:
                    vel_x = 0.3   

                if event.key == pygame.K_DOWN:
                    vel_x = -0.3

                if event.key == pygame.K_LEFT:
                    vel_y = 0.3

                if event.key == pygame.K_RIGHT:
                    vel_y = -0.3

                if event.key == pygame.K_RCTRL:
                    mc.stop()
                

            if event.type == pygame.KEYUP:
                
                if event.key == pygame.K_w or event.key == pygame.K_s :
                    vel_z = 0

                if event.key == pygame.K_a or event.key == pygame.K_d:
                    yaw_rate = 0

                if event.key == pygame.K_UP or event.key == pygame.K_DOWN:
                    vel_x = 0  

                if event.key == pygame.K_LEFT or event.key == pygame.K_RIGHT:
                    vel_y = 0


            mc._set_vel_setpoint(vel_x, vel_y, vel_z, yaw_rate)


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




if __name__ == '__main__':

    try:

   	    # Clean all files if not done so previously
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

        cflib.crtp.init_drivers(enable_debug_driver=False)

        
        scf = SyncCrazyflie(URI)
        scf.open_link()


        mc = MotionCommander(scf)

        #mc._reset_position_estimator()
        #mc.take_off()

        #key = Thread(target = key_ctrl, args = (mc,scf))
        #key.start()
        letstest = yaw_test(scf)

        """fig = plt.figure()
        axRoll = fig.add_subplot(3,1,1)
        axYaw = fig.add_subplot(3,1,2)
        axPitch = fig.add_subplot(3,1,3)
        animate = displayStb(axRoll,axYaw,axPitch)
        ani = animation.FuncAnimation(fig, animate.update, interval=20)
        plt.show()"""


       
    except Exception, e:
        print(str(e))
        print('\nShutting down...')
                
        pygame.quit()
        sys.exit(0) 
    
                