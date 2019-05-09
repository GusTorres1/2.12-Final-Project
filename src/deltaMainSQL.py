#!/usr/bin/python
"""
Delta Robot Main Interface

Based off of:

Daniel J. Gonzalez - dgonz@mit.edu
2.12 Intro to Robotics Spring 2019

From Sandra:
Included the odrive_interface code
Let's run it to test!
"""
#####################################
realBot = True
#####################################

if realBot:
    import odrive_interface as bot
else:
    import robot212_virtual as bot

import kinematicsSolverEdited as kin
import time
import numpy as np

pi = np.pi #3.1415927
bot.trajMoveRad((0,0,0))
deltaKin = kin.deltaSolver()

if __name__ == "__main__":
    bot.connect_all()
    bot.calibrate()
    
    bot.full_init()
    bot.connect_all()
    
    bot.trajMoveCnt((0,0,0))
    isRunning = True
    mainRunning = True
    inp = input("Press ENTER to begin or q+ENTER to quit...")
    if inp == 'q':

        mainRunning = False
    while mainRunning:
        tStart = time.time()
        while isRunning:
            t = time.time() - tStart
            zD0 = deltaKin.z
            if t > 20:
                isRunning = False
            elif t<10:
                #   Define Trajectory
                r = (t/10)*100 #mm
                freq = 0.5#Hz
                xD = 0
                yD = 0
                zD = zD0 - r*2

                #   Solve for and execute trajectory, print the endpoint and joint angles, update the plot. 
                thtDes = deltaKin.IK((xD, yD, zD))
                bot.trajMoveRad(thtDes, 2*pi/8, 2*pi/8) # (Desired Angles [rad], Max Velocity [rad/s], Acceleration/Deceleration [rad/s^2])
                print((xD, yD, zD), thtDes)
                deltaKin.updatePlot((xD, yD, zD))
            elif t>10 and t<20:
                #   Define Trajectory
                r = 100 - ((t-10)/10)*100 #mm
                freq = 0.5#Hz
                xD = r*np.cos((freq*2*np.pi)*t)
                yD = r*np.sin((freq*2*np.pi)*t)
                zD = zD0 - r*2

                #   Solve for and execute trajectory, print the endpoint and joint angles, update the plot. 
                thtDes = deltaKin.IK((xD, yD, zD))
                bot.trajMoveRad(thtDes, 2*pi/8, 2*pi/8) # (Desired Angles [rad], Max Velocity [rad/s], Acceleration/Deceleration [rad/s^2])
                print((xD, yD, zD), thtDes)
                deltaKin.updatePlot((xD, yD, zD))
        inp = input("Press ENTER to run again or q+ENTER to quit...")
        if inp == 'q':
            mainRunning = False
        else:
            isRunning = True
    bot.trajMoveRad((0,0,0))
