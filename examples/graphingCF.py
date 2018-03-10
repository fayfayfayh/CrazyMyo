import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation

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
                    print("initial")
                    timescale.append(float(ts)/1000)
                    stbRoll.append(round(float(roll), 2))
                    stbYaw.append(round(float(yaw), 2))
                    stbPitch.append(round(float(pitch), 2))
                    self.tsInit = float(ts)/1000
                else:
                    ts, roll, yaw, pitch = line.split(',')
                    timescale.append(float(ts)/1000  - self.tsInit)
                    stbRoll.append(round(float(roll), 2))
                    stbYaw.append(round(float(yaw), 2))
                    stbPitch.append(round(float(pitch), 2))

            j=j+1

        self.ax1.clear()
        self.ax2.clear()
        self.ax3.clear()
        self.ax1.plot(timescale,stbRoll)
        self.ax2.plot(timescale,stbPitch)
        self.ax3.plot(timescale,stbYaw)
        self.ax1.set_ylabel('Roll (degree)')
        self.ax2.set_ylabel('Pitch (degree)')
        self.ax3.set_ylabel('Yaw (degree)')
        self.ax3.set_xlabel('Operation Time')



if __name__ == '__main__':

    fig1 = plt.figure()
    fig1.suptitle("Stabilizer Data", fontsize=14)

    axRoll = fig1.add_subplot(3,1,1)
    axRoll.set_ylabel('Roll (degree)')
    axRoll.set_title("Roll Motion")

    axPitch = fig1.add_subplot(3,1,2)
    axPitch.set_ylabel('Pitch (degree)')

    axYaw = fig1.add_subplot(3,1,3)
    axYaw.set_xlabel('Operation Time')
    axYaw.set_ylabel('Yaw (degree)')


    animate = displayStb(axRoll,axYaw,axPitch)
    ani = animation.FuncAnimation(fig1, animate.update, interval=20)
    plt.show()
