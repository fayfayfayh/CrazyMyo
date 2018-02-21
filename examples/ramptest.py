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
"""
Simple example that connects to the first Crazyflie found, ramps up/down
the motors and disconnects.
"""
import logging
import time
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

class MotorRun:

    def __init__(self, link_uri):
        """ Initialize and run the example with the specified link_uri """
        self._cf = Crazyflie()

        self._cf.connected.add_callback(self._connected)
        self._cf.disconnected.add_callback(self._disconnected)
        self._cf.connection_failed.add_callback(self._connection_failed)
        self._cf.connection_lost.add_callback(self._connection_lost)

        self._cf.open_link(link_uri)

        print('Connecting to %s' % link_uri)

    def _connected(self, link_uri): 
        """ This callback is called form the Crazyflie API when a Crazyflie
        has been connected and the TOCs have been downloaded."""

        """This is from exmaple basicLog"""
        print('Connected to %s' % link_uri)

        # Start a separate thread to do the motor test.
        # Do not hijack the calling thread!
        Thread(target = self._ramp_motors).start()
        Thread(target = self._getStabilizer).start()
        Thread(target = self._getAccelerometer).start()
        Thread(target = self._getGyroscope).start()
        

    def _stab_log_error(self, logconf, msg):
        """Callback from the log API when an error occurs""" 
        print('Error when logging %s: %s' % (logconf.name, msg))

    def _stab_log_data(self, timestamp, data, logconf):
        """Callback froma the log API when data arrives"""
        """print('[%d][%s]: %s' % (timestamp, logconf.name, data))"""
        with open('StabilizerData.txt', 'a') as stabilizerData:
            stabilizerData.write('[%d][%s]: %s' % (timestamp, logconf.name, data))
            stabilizerData.write('\n')
        with open('SensorMaster.txt', 'a') as sensorMaster:
            sensorMaster.write('%d,%.2f,%.2f,%.2f' %(timestamp, data['stabilizer.roll'], data['stabilizer.yaw'], data['stabilizer.pitch']))
            sensorMaster.write('\n')

    def _log_gyro_data(self, timestamp, data, logconf):
        with open('GyroscopeData.txt', 'a') as GyroscopeData:
            GyroscopeData.write('[%d][%s]: %s \n' % (timestamp, logconf.name, data))

    def _log_accel_data(self, timestamp, data, logconf):
        with open('AccelerometerData.txt', 'a') as AccelerometerData:
            AccelerometerData.write('[%d][%s]: %s \n' % (timestamp, logconf.name, data))

    def _connection_failed(self, link_uri, msg):
        """Callback when connection initial connection fails (i.e no Crazyflie
        at the specified address)"""
        print('Connection to %s failed: %s' % (link_uri, msg))

    def _connection_lost(self, link_uri, msg):
        """Callback when disconnected after a connection has been made (i.e
        Crazyflie moves out of range)"""
        print('Connection to %s lost: %s' % (link_uri, msg))

    def _disconnected(self, link_uri):
        """Callback when the Crazyflie is disconnected (called in all cases)"""
        print('Disconnected from %s' % link_uri)

    def _ramp_motors(self):
        thrust_mult = 1
        thrust_step = 500
        thrust = 0
        pitch = 0
        roll = 0
        yawrate = 0
        ifFly = 0

        while (ifFly == 0):
            thrust = input('Enter desired thrust (between 20000 to 30000) to fly Cazyflie\n')
            if(thrust < 20000 or thrust > 30000):
                print('invalid thrust value, please try again. ')
            else:
            	ifFly = 1

        # Unlock startup thrust protection
        self._cf.commander.send_setpoint(0, 0, 0, 0)

        while (thrust >= 20000):
            self._cf.commander.send_setpoint(roll, pitch, yawrate, thrust)
            time.sleep(0.1)
            if thrust >= 36000:
                thrust_mult = -1
            thrust += thrust_step * thrust_mult
        self._cf.commander.send_setpoint(0, 0, 0, 0)
        # Make sure that the last packet leaves before the link is closed
        # since the message queue is not flushed before closing
        time.sleep(0.1)
        self._cf.close_link()

    def _getStabilizer(self):
    	 # The definition of the logconfig can be made before connecting
        self._lg_stab = LogConfig(name='Stabilizer', period_in_ms=10)
        self._lg_stab.add_variable('stabilizer.roll', 'float')
        self._lg_stab.add_variable('stabilizer.pitch', 'float')
        self._lg_stab.add_variable('stabilizer.yaw', 'float')
        # Adding the configuration cannot be done until a Crazyflie is
        # connected, since we need to check that the variables we
        # would like to log are in the TOC.
        try:
            self._cf.log.add_config(self._lg_stab)
            # This callback will receive the data
            self._lg_stab.data_received_cb.add_callback(self._stab_log_data)
            # This callback will be called on errors
            self._lg_stab.error_cb.add_callback(self._stab_log_error)
            # Start the logging
            self._lg_stab.start()
        except KeyError as e:
            print('Could not start log configuration,'
                  '{} not found in TOC'.format(str(e)))
        except AttributeError:
            print('Could not add Stabilizer log config, bad configuration.')

    def _getAccelerometer(self):
        self._log_conf = LogConfig(name="Accel", period_in_ms=10)
        self._log_conf.add_variable('acc.x', 'float')
        self._log_conf.add_variable('acc.y', 'float')
        self._log_conf.add_variable('acc.z', 'float')

        try:
            self._log = self._cf.log.add_config(self._log_conf)
            if self._log_conf is not None:
                self._log_conf.data_received_cb.add_callback(self._log_accel_data)
                self._log_conf.start()
            else:
                print("acc.x/y/z not found in log TOC") 
        except KeyError as e:
            print('Could not start log configuration,'
                  '{} not found in TOC'.format(str(e)))
        except AttributeError:
            print('Could not add Accelerometer log config, bad configuration.')

    def _getGyroscope(self):
        self._gyro_conf = LogConfig(name="Gyro", period_in_ms=10)
        self._gyro_conf.add_variable('gyro.x', 'float')
        self._gyro_conf.add_variable('gyro.y', 'float')
        self._gyro_conf.add_variable('gyro.z', 'float')
    
        try:
            self._gyro = self._cf.log.add_config(self._gyro_conf)
            if self._gyro_conf is not None:
                self._gyro_conf.data_received_cb.add_callback(self._log_gyro_data)
                self._gyro_conf.start()
            else:
                print("gyro.x/y/z not found in log TOC") 
        except KeyError as e:
            print('Could not start log configuration,'
                  '{} not found in TOC'.format(str(e)))
        except AttributeError:
            print('Could not add Gyroscope log config, bad configuration.')

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
   
    ifstart = 0
    	
    while (ifstart == 0):
        ifstart = input('entre "1" to search for Crazyflie\n')

    #clearn all files if not done so previously
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

    # Initialize the low-level drivers (don't list the debug drivers)
    cflib.crtp.init_drivers(enable_debug_driver=False)
    # Scan for Crazyflies and use the first one found
    print('Scanning interfaces for Crazyflies...')
    available = cflib.crtp.scan_interfaces()
    print('Crazyflies found:')
    for i in available:
        print(i[0])

    if len(available) > 0:
        #le = MotorRampExample(available[0][0])
        le = MotorRun("radio://0/80/2M")
        #ADD THE GRAPHING HERE INSTEAD!!!!!!
    else:
        print('No Crazyflies found, cannot run example')


    fig = plt.figure()
    axRoll = fig.add_subplot(3,1,1)
    axYaw = fig.add_subplot(3,1,2)
    axPitch = fig.add_subplot(3,1,3)
    animate = displayStb(axRoll,axYaw,axPitch)
    ani = animation.FuncAnimation(fig, animate.update, interval=20)
    plt.show()
