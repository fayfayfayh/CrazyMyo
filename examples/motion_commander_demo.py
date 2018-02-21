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
import pygame as p

import cflib.crtp
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.positioning.motion_commander import MotionCommander

URI = 'radio://0/80/2M'

# Only output errors from the logging framework
#logging.basicConfig(level=logging.ERROR)


if __name__ == '__main__':

    try:
        
        p.init()
        
        print '\nWASD for throttle & yaw; arrow keys for left/right/forward/backward' 
        print 'Any key to start; spacebar to land\n'

        raw_input()

        while True:

            keys = pygame.key.get_pressed()

            if keys[K_w]:
                print 'w'
            elif keys[K_a]:
                print 'a'
            elif keys[K_a]:
                print 'a'
            elif keys[K_s]:
                print 's'
            elif keys[K_UP]:
                print 'up'
            elif keys[K_LEFT]:
                print 'left'
            elif keys[K_DOWN]:
                print 'down'
            elif keys[K_RIGHT]:
                print 'right'

        

        # cflib.crtp.init_drivers(enable_debug_driver=False)


        # with SyncCrazyflie(URI) as scf:
        # # We take off when the commander is created
        #     with MotionCommander(scf) as mc:
        #         time.sleep(1)

        #         # We land when the MotionCommander goes out of scope

        #         print 'Enter WASD to move, ctrl+c to exit'   

        #         key = raw_input()

                

        #         while True:

        #             key = raw_input()

        #             if key == 'w':
        #                 print 'going up'
        #                 mc.up(0.1)

        #             elif key == 'a':
        #                 print 'going left'
        #                 mc.left(0.1)

        #             elif key == 's':
        #                 print 'doing down'
        #                 mc.down(0.1)

        #             elif key == 'd':
        #                 print 'going right'
        #                 mc.right(0.1)

        #             else:
        #                 pass
                    
    except KeyboardInterrupt:
        print('\nUser interrupted operation; shutting down...')
                
        
        sys.exit(0) 
    
    


    # Initialize the low-level drivers (don't list the debug drivers)

    
