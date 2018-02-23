# -*- coding: utf-8 -*-

import sys
import logging
import time

from math import sin, cos
from threading import Thread
from threading import Timer
import os
import signal

import cflib.crtp
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.positioning.motion_commander import MotionCommander
import _globals

class FlightCtrl:

    def __init__(self, scf):
        
        global gesture_queue
        scf.open_link()
        

    def perform_gesture(g_id, mc):

        if g_id == DOUBLE_TAP:
            if mc.is_flying():
                pass
            else:
                mc.take_off()
        else:
            pass


    def gesture_ctrl(self, scf, g):
        mc = MotionCommander(scf)
        mc._reset_position_estimator() 

        print 'Enter to start (c to cancel)'
        char = raw_input()

        if char == 'c':
            scf.close_link()
            sys.exit(0)

        try:

            while True:

                g_id = gesture_queue.get_gesture()

                if gesture is not None:
                    perform_gesture(g_id, mc)

        except Exception, e:
            print str(e)
            scf.close_link()
