import sys
import logging
import time

from threading import Thread
from threading import Timer

import cflib.crtp
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.positioning.motion_commander import MotionCommander

from gestures import Gesture
gesture = Gesture()

import os
import signal

DOUBLE_TAP = 1

URI = 'radio://0/80/2M'

# Only output errors from the logging framework
logging.basicConfig(level=logging.ERROR)


class FlightCtrl:

    def __init__(self, scf):
        
        global gesture
        scf.open_link()
        

    def perform_gesture(self, g_id, mc):

        if g_id == DOUBLE_TAP:
            if mc._is_flying:
                pass
            else:
                mc.take_off()
        # Add new gestures here
        else: 
            pass


    def gesture_ctrl(self, scf, fc, g):
        mc = MotionCommander(scf)
        mc._reset_position_estimator() 

        print 'Enter to start (c to cancel)'
        char = raw_input()

        if char == 'c':
            print('\nClosing link...')
            scf.close_link()

            print('Shutting down...')
            os.kill(os.getpid(), signal.SIGINT) 

        try:

            while True:

                g_id = gesture.get_gesture()

                if gesture is not None:
                    fc.perform_gesture(g_id, mc)

        except Exception, e:
            print str(e)
            scf.close_link()


class Myo:

    def __init__(self):
        global gesture

    def gesture_detection(self, g):
        # Add gesture detection stuff here
        # Put it into the queue similar to below, a corresponding update to perform_gesture in 
        # FlightCtrl would be needed for full integration
        g=1
        gesture.queue.put(g, block=False)


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