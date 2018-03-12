import sys
import logging
import time

from threading import Thread
from threading import Timer

import cflib.crtp

import os
import signal

URI = 'radio://0/80/2M'

import cflib
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig

logging.basicConfig(level=logging.ERROR)

class Comm:

    def __init__(self, link_uri):
        """ Initialize and run the test with the specified link_uri """

        self._cf = Crazyflie()

        self.mc = MotionCommander(self._cf, default_height=0.7)

        self._cf.connected.add_callback(self._connected)
        self._cf.disconnected.add_callback(self._disconnected)
        self._cf.connection_failed.add_callback(self._connection_failed)
        self._cf.connection_lost.add_callback(self._connection_lost)

        self._cf.open_link(link_uri)

        print('Connecting to %s' % link_uri)
        self.is_connected = True


    def _connected(self, link_uri):
        """ This callback is called from the Crazyflie API when a Crazyflie
        has been connected and the TOCs have been downloaded."""

        print('Connected to %s' % link_uri)

        # The definition of the logconfig can be made before connecting
        self._lg_stab = LogConfig(name='Stabilizer', period_in_ms=10)
        self._lg_stab.add_variable('stabilizer.roll', 'float')
        self._lg_stab.add_variable('stabilizer.pitch', 'float')
        self._lg_stab.add_variable('stabilizer.yaw', 'float')

        self._log_conf = LogConfig(name="Accel", period_in_ms=10)
        self._log_conf.add_variable('acc.x', 'float')
        self._log_conf.add_variable('acc.y', 'float')
        self._log_conf.add_variable('acc.z', 'float')

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
            print('Could not add log config, bad configuration.')

        # Start a separate thread to do the motor test.
        # Do not hijack the calling thread!
        self.is_connected = True
        worker = Thread(target = self.flip_test)
        worker.start()

    def _log_accel_data(self, timestamp, data, logconf):
        #print('[%d] Accelerometer: x=%.2f, y=%.2f, z=%.2f' %
        #            (timestamp, data['acc.x'], data['acc.y'], data['acc.z']))

        """This is from the basiclog file"""
    def _stab_log_error(self, logconf, msg):
        """Callback from the log API when an error occurs"""
        #print('Error when logging %s: %s' % (logconf.name, msg))

    def _stab_log_data(self, timestamp, data, logconf):
        """Callback froma the log API when data arrives"""
        #print('[%d][%s]: %s' % (timestamp, logconf.name, data))


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
        print('Disconnected from %s' % link_uri) #mc = MotionCommander(self._cf)

    def send_setpoint(self, roll, pitch, yaw, thrust):
        """
        Send a new control setpoint for roll/pitch/yaw/thrust to the copter

        The arguments roll/pitch/yaw/trust is the new setpoints that should
        be sent to the copter
        """
        if thrust > 0xFFFF or thrust < 0:
            raise ValueError('Thrust must be between 0 and 0xFFFF')

        if self._x_mode:
            roll, pitch = 0.707 * (roll - pitch), 0.707 * (roll + pitch)

        pk = CRTPPacket()
        pk.port = CRTPPort.COMMANDER
        pk.data = struct.pack('<fffH', roll, -pitch, yaw, thrust)
        self._cf.send_packet(pk)

    def flip_test(self): 

        # Unlock startup thrust protection
        self._cf.commander.send_setpoint(0, 0, 0, 0)

        
        print 'Beginning test...'
      
        self.mc.take_off()

        
        thrust = 35000
        self._cf.commander.send_setpoint(0, 0, 0, thrust) 
        time.sleep(1)

        self._cf.commander.send_setpoint(0, 0, 0, 0	) 

            

if __name__ == '__main__':

    try:
        # Initialize the low-level drivers (don't list the debug drivers)
        cflib.crtp.init_drivers(enable_debug_driver=False)
        # Scan for Crazyflies and use the first one found

        print('Scanning interfaces for Crazyflies...')
        available = cflib.crtp.scan_interfaces()

        for i in available:
            print('Found: ' + i[0])
            
        if len(available) > 0:
            le = Comm('radio://0/80/2M')
        else:
            print('No Crazyflies found')
            sys.exit(0)
    
        while True:
            time.sleep(0.1)

    except KeyboardInterrupt:
        print('\nUser interrupted operation; shutting down...')
                
        le._cf.commander.send_setpoint(0, 0, 0, 0)
        le._cf.close_link()
        sys.exit(0)    
