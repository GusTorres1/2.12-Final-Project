#!/usr/bin/env python

import rospy

from std_msgs.msg import Float32MultiArray, Int8, Bool

from odrive_ros import odrive_interface
bot = odrive_interface.ODriveInterfaceAPI()

import kinematicsSolverEdited as kin
from time import sleep
import Queue

deltaKin = kin.deltaSolver()

pi = 3.1415

class ODriveNode(object):
    def __init__(self):
        rospy.init_node('odrive')
        
        self.command_queue = Queue.Queue(maxsize=5)
        rospy.Subscriber('/pos', Float32MultiArray, self.cmd_callback)
        
        self.vac_pub = rospy.Publisher('/toggle_vac', Int8, queue_size = 1)
        self.cam_pub = rospy.Publisher('/camera_move', Bool, queue_size = 1)
        
        self.thtDes = [0, 0, 0]
        bot.connect_all()
        print('ready')

    def cmd_callback(self, msg):
        '''
        TODO: Find the closest one and move to it, dawg
        We should figure out that logic.
        '''
        print('hello')
        self.pos = msg.data
        self.thtDes = deltaKin.solveIt(self.pos)
        bot.trajMoveRad(self.thtDes, 2*pi/8, 2*pi/8)
        
    def ifClose(self, tht):
        if bot.rad2Count(tht[0]) - bot.axis0.controller.pos_setpoint <= 500:
            if bot.rad2Count(tht[1]) - bot.axis1.controller.pos_setpoint <= 500:
                if bot.rad2Count(tht[2]) - bot.axis2.controller.pos_setpoint <= 500:
                    return True
        return False
        
    def main_loop(self):
        main_rate = rospy.Rate(1)
        pizzaTop = True
        punchDough = False
        movePizza = False
        parmShake = False
        
        vacInt = 2 # no vacuum
        moveBool = True # Send one value
        balRad = 1.25 # inches
        pizRad = 4.0 # inches
        tab = 70 # inches
        
        in2mm = 25.4
        mm2in = 1/in2mm
        in2m = in2mm/1000
        
        while not rospy.is_shutdown():
            '''
            Testing the vac ros node comm
            rospy.sleep(0.1)
            self.vac_pub.publish(2)
            rospy.sleep(0.5)
            self.vac_pub.publish(1)
            rospy.sleep(1.5)
            self.vac_pub.publish(0)
            rospy.sleep(0.1)
            self.vac_pub.publish(1)
            '''
            
            #try:
                #main_rate.sleep()
            #except rospy.ROSInterruptException:
                #break
            #if pizzaTop:
                #self.vac_pub.publish(vacInt)
                #sleep(0.25) # Let vacuum activate for a bit
                #self.cam_pub.publish(moveBool)
                #sleep(0.1)
                #moveBool = False # Stop sending more values
                #self.cam_pub.publish(moveBool)
                #while not moveBool:
                    #rospy.sleep(0.1)
                    #bot.trajMoveRad(self.thtDes, 2*pi/8, 2*pi/8)
                    
                    ## If close enough, tell camera good to send next position
                    #if self.ifClose(self.thtDes):
                        #if vacInt == 2:
                            #vacInt = 0 # Begin to vacuum
                        #elif vacInt == 0:
                            #vacInt = 1 # Inflate a tiny bit
                            #self.vac_pub.publish(vacInt)
                            #sleep(0.1)
                            #vacInt = 2
                        #moveBool = True
            
            #elif movePizza:
                #self.vac_pub.publish(0) # stay vacuuming
                #self.cam_pub.publish(moveBool)
                #sleep(0.1)
                #self.pos[1] = self.pos[1] - (balRad + pizRad) * in2m
                #tht = deltaKin.solveIt(self.pos)
                #moveBool = False
                #self.cam_pub.publish(moveBool) # Stop sending values
                #while not moveBool:
                    #bot.trajMoveRad(tht, 2*pi/8, 2*pi/8)
                    
                    ## If close enough, tell camera good to send next position                   
                    #if self.ifClose(tht):
                        #moveBool = True
                    
                #pos = [self.pos[0], -tab/2, self.pos[2]]
                #tht = deltaKin.solveIt(pos)
                #bot.trajMoveRad(tht, 2*pi/8, 2*pi/8)
                
                #if self.ifClose(tht):
                    ## Somehow tell mobile bot that it has the pizza
                    #pass
            
            #elif punchDough:
                #pass
                ## TODO: Eventually do star compass shape :D
            
            #elif parmShake:
                #pass
                ## Mebbe

if __name__ == '__main__':
    try:
        odrive_node = ODriveNode()
        odrive_node.main_loop()
    except rospy.ROSInterruptException:
        pass
