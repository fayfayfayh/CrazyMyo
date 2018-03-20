# -*- coding: utf-8 -*-
#
#     ||          ____  _ __
#  +------+      / __ )(_) /_______________ _____  ___
#  | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
#  +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
#   ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
#
#  Copyright (C) 2016 Bitcraze AB
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
The syncronous Crazyflie class is a wrapper around the "normal" Crazyflie
class. It handles the asynchronous nature of the Crazyflie API and turns it
into blocking function. It is useful for simple scripts that performs tasks
as a sequence of events.
"""
from threading import Event
from threading import Thread
from threading import Timer

import cflib
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig

from cflib.crazyflie import Crazyflie


class SyncCrazyflie:

    def __init__(self, link_uri, cf=None):
        """ Create a synchronous Crazyflie instance with the specified
        link_uri """

        if cf:
            self.cf = cf
        else:
            self.cf = Crazyflie()

        self._link_uri = link_uri
        self._connect_event = Event()
        self._is_link_open = False
        self._error_message = None
        self.vbat = None

        self.cf.connected.add_callback(self._connected)
        self.cf.connection_failed.add_callback(self._connection_failed)
        self.cf.disconnected.add_callback(self._disconnected)

    def open_link(self):
        if (self.is_link_open()):
            raise Exception('Link already open')

        print('Connecting to %s' % self._link_uri)
        self.cf.open_link(self._link_uri)
        self._connect_event.wait()
        if not self._is_link_open:
            raise Exception(self._error_message)

    def __enter__(self):
        self.open_link()
        return self

    def close_link(self):
        self.cf.close_link()
        self._is_link_open = False

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.close_link()

    def is_link_open(self):
        return self._is_link_open

    def _connected(self, link_uri):
        """ This callback is called form the Crazyflie API when a Crazyflie
        has been connected and the TOCs have been downloaded."""
        print('Connected to %s' % link_uri)
        self._is_link_open = True
        self._connect_event.set()


        #Open threads for recording sensor data
        Thread(target = self._getStabilizer).start()
        Thread(target = self._getVbat).start()
        #Thread(target = self._getAccelerometer).start()
        #Thread(target = self._getGyroscope).start()

    def _connection_failed(self, link_uri, msg):
        """Callback when connection initial connection fails (i.e no Crazyflie
        at the specified address)"""
        print('Connection to %s failed: %s' % (link_uri, msg))
        self._is_link_open = False
        self._error_message = msg
        self._connect_event.set()

    def _disconnected(self, link_uri):
        self._is_link_open = False


    """all definition after this point is for status monitor"""

    def _vbat_log_data(self, timestamp, _vbat, logconf):
        self.vbat = _vbat['pm.vbat']
        

    def _vbat_log_error(self, logconf, msg):
        """Callback from the log API when an error occurs"""
        print('Error when logging %s: %s' % (logconf.name, msg))

    def _stab_log_error(self, logconf, msg):
        """Callback from the log API when an error occurs"""
        print('Error when logging %s: %s' % (logconf.name, msg))

    def _stab_log_data(self, timestamp, data, logconf):
        """Callback froma the log API when data arrives"""
        """print('[%d][%s]: %s' % (timestamp, logconf.name, data))"""
        #with open('StabilizerData.txt', 'a') as stabilizerData:
            #stabilizerData.write('[%d][%s]: %s' % (timestamp, logconf.name, data))
            #stabilizerData.write('\n')
        with open('SensorMaster.txt', 'a') as sensorMaster:
            sensorMaster.write('%d,%.2f,%.2f,%.2f' %(timestamp, data['stabilizer.roll'], data['stabilizer.yaw'], data['stabilizer.pitch']))
            sensorMaster.write('\n')

    def _log_gyro_data(self, timestamp, data, logconf):
        with open('GyroscopeData.txt', 'a') as GyroscopeData:
            GyroscopeData.write('[%d][%s]: %s \n' % (timestamp, logconf.name, data))

    def _log_accel_data(self, timestamp, data, logconf):
        with open('AccelerometerData.txt', 'a') as AccelerometerData:
            AccelerometerData.write('[%d][%s]: %s \n' % (timestamp, logconf.name, data))

    def _getVbat(self):

        self._lg_vbat = LogConfig(name='bat', period_in_ms=10)
        self._lg_vbat.add_variable('pm.vbat', 'float')

        try:
            self.cf.log.add_config(self._lg_vbat)
            # This callback will receive the data
            self._lg_vbat.data_received_cb.add_callback(self._vbat_log_data)

            # Start the logging
            self._lg_vbat.start()

        except KeyError as e:
            print('Could not start log configuration,'
                  '{} not found in TOC'.format(str(e)))
        except AttributeError:
            print('Could not add battery log config, bad configuration.')

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
            self.cf.log.add_config(self._lg_stab)
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
            self._log = self.cf.log.add_config(self._log_conf)
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
            self._gyro = self.cf.log.add_config(self._gyro_conf)
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
