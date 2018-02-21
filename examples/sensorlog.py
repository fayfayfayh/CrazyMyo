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
Simple example that connects to the first Crazyflie found, logs the Stabilizer
and prints it to the console. After 10s the application disconnects and exits.
"""
import logging
import time
from threading import Thread
from threading import Timer

#from cflib import crtp
import cflib.crtp  # noqa
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig

# Only output errors from the logging framework
logging.basicConfig(level=logging.ERROR)


class LoggingExample:
    """
    Simple logging example class that logs the Stabilizer from a supplied
    link uri and disconnects after 5s.
    """

    def __init__(self, link_uri):
        """ Initialize and run the example with the specified link_uri """

        # Create a Crazyflie object without specifying any cache dirs
        self._cf = Crazyflie()

        # Connect some callbacks from the Crazyflie API
        self._cf.connected.add_callback(self._connected)
        self._cf.disconnected.add_callback(self._disconnected)
        self._cf.connection_failed.add_callback(self._connection_failed)
        self._cf.connection_lost.add_callback(self._connection_lost)

        print('Connecting to %s' % link_uri)

        # Try to connect to the Crazyflie
        self._cf.open_link(link_uri)

        # Variable used to keep main loop occupied until disconnect
        self.is_connected = True

    def _connected(self, link_uri):
        """ This callback is called form the Crazyflie API when a Crazyflie
        has been connected and the TOCs have been downloaded."""
        print('Connected to %s' % link_uri)

        Thread(target = self._getAccelerometer).start()
        Thread(target = self._getGyroscope).start()
        
        # Start a timer to disconnect in 10s
        t = Timer(5, self._cf.close_link)
        t.start()

    def _getAccelerometer(self):
        self._log_conf = LogConfig(name="Accel", period_in_ms=10)
        self._log_conf.add_variable('acc.x', 'float')
        self._log_conf.add_variable('acc.y', 'float')
        self._log_conf.add_variable('acc.z', 'float')

        try:
            self._log= self._cf.log.add_config(self._log_conf)
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
            self._gyro= self._cf.log.add_config(self._gyro_conf)
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

    def _log_accel_data(self, timestamp, data, logconf):
        print('[%d] Accelerometer: x=%.2f, y=%.2f, z=%.2f' %
                     (timestamp, data['acc.x'], data['acc.y'], data['acc.z']))

    def _log_gyro_data(self, timestamp, data, logconf):
        print('[%d] Gyroscope: x=%.2f, y=%.2f, z=%.2f' %
                     (timestamp, data['gyro.x'], data['gyro.y'], data['gyro.z']))


    def _connection_failed(self, link_uri, msg):
        """Callback when connection initial connection fails (i.e no Crazyflie
        at the speficied address)"""
        print('Connection to %s failed: %s' % (link_uri, msg))
        self.is_connected = False

    def _connection_lost(self, link_uri, msg):
        """Callback when disconnected after a connection has been made (i.e
        Crazyflie moves out of range)"""
        print('Connection to %s lost: %s' % (link_uri, msg))

    def _disconnected(self, link_uri):
        """Callback when the Crazyflie is disconnected (called in all cases)"""
        print('Disconnected from %s' % link_uri)
        self.is_connected = False


if __name__ == '__main__':
    # Initialize the low-level drivers (don't list the debug drivers)
    cflib.crtp.init_drivers(enable_debug_driver=False)
    # Scan for Crazyflies and use the first one found
    print('Scanning interfaces for Crazyflies...')
    available = cflib.crtp.scan_interfaces()
    print('Crazyflies found:')
    for i in available:
        print(i[0])

    if len(available) > 0:
        le = LoggingExample(available[0][0])
    else:
        print('No Crazyflies found, cannot run example')

    # The Crazyflie lib doesn't contain anything to keep the application alive,
    # so this is where your application should do something. In our case we
    # are just waiting until we are disconnected.
    while le.is_connected:
        time.sleep(1)
