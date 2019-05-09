#!/usr/bin/env python
import rospy

#import communication # For communicating with the mobile robot
#mobile_ready = rospy.get_param('/MOBILE_ARRIVED')

from std_msgs.msg import Float32MultiArray, Int8, Bool

from odrive_ros import odrive_interface
bot = odrive_interface.ODriveInterfaceAPI()

import kinematicsSolverEdited as kin
from time import sleep
import Queue

import numpy as np

deltaKin = kin.deltaSolver()

pi = 3.1415
balRad = 1.25 # inches
pizRad = 4.0 # inches
tab = 70 # inches

in2mm = 25.4
mm2in = 1/in2mm
in2m = in2mm/1000

#Operating heights
zhome = -730 #mm
ztable = -795 #mm
zdrop = -750 #mm
yhome = - 300 #mm
xhome = 0 #mm

homeTht = deltaKin.solveIt([xhome, yhome, zhome])

class ODriveNode(object):
    def __init__(self):
        rospy.init_node('odrive')
        
        self.command_queue = Queue.Queue(maxsize=5)
        rospy.Subscriber('/pos', Float32MultiArray, self.cmd_callback)
        #rospy.Subscriber('/what_task', Int8, self.task_callback)
        self.task_comp = True
        
        self.pizzaTop = True
        self.topDrop = False
        self.movePizza = False
        self.punchDough = False
        self.parmShake = False
        
        self.task_comp = True
        
        self.vac_pub = rospy.Publisher('/toggle_vac', Int8, queue_size = 1)
        self.task_pub = rospy.Publisher('/task_complete', Bool, queue_size = 1)
        
        self.thtDes = [0, 0, 0]
        bot.connect_all()
        print('ready')

    def cmd_callback(self, msg):
        print('hello')
        if self.task_comp: # Only get the position values if task is complete
            print("Let's get started")
            self.pos = msg.data # Get the proper value from the dictionary
            self.pos = list(self.pos)
            self.pos.append(zhome)
            if self.topDrop == True:
                self.pos[2]=(zdrop)
            if self.movePizza:
                theta = np.atan2(pos[0], pos[1]-(tab/2-2)*in2mm)
                self.pos[0] -= np.sin(theta)*(pizRad+balRad)*in2mm
                self.pos[1] += np.cos(theta)*(pizRad+balRad)*in2mm
            self.thtDes = deltaKin.solveIt(self.pos)
    
    '''   
    def task_callback(self, msg):
        self.task = msg.data
        # 0 - pick up topping
        # 1 - drop topping
        # 2 - move pizza
        # 3 - punch dough
        # 4 - parm shake
        if self.task == 0 or self.task == 1:
            self.pizzaTop = True
        elif self.task == 2:
            self.movePizza = True
        elif self.task == 3:
            self.punchDough = True
        elif self.task == 4:
            self.parmShake = True
    '''
        
    def ifClose(self, tht):
        if bot.rad2Count(tht[0]) - bot.axis0.controller.pos_setpoint <= 500:
            if bot.rad2Count(tht[1]) - bot.axis1.controller.pos_setpoint <= 500:
                if bot.rad2Count(tht[2]) - bot.axis2.controller.pos_setpoint <= 500:
                    print("we get there homie")
                    return True
        return False
        
    def main_loop(self):
        main_rate = rospy.Rate(1)
        vacInt = 2 # no vacuum
        
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
            
            try:
                main_rate.sleep()
            except rospy.ROSInterruptException:
                break
            if self.pizzaTop:
                for i in range(9):
                    print("Let's take a pic.")
                    bot.trajMoveRad(homeTht)
                    self.task_pub.publish(self.task_comp)
                    rospy.sleep(0.1)
                    print("Let's grab the topping.")
                    self.task_comp = False
                    self.task_pub.publish(self.task_comp)
                    self.vac_pub.publish(vacInt)
                    bot.trajMoveRad(self.thtDes, 2*pi/8, 2*pi/8)
                    #print(self.pos)
                    rospy.sleep(0.3)
                    if self.ifClose(self.thtDes):
                        posGrip = [self.pos[0], self.pos[1], ztable]
                        thtGrip = deltaKin.solveIt(posGrip)
                        bot.trajMoveRad(thtGrip)
                        #print(posGrip)
                        rospy.sleep(0.5)
                        if self.ifClose(thtGrip):
                            self.topDrop = True
                            self.vac_pub.publish(0)
                            self.task_comp = True
                            self.task_pub.publish(self.task_comp)
                            rospy.sleep(0.1)
                    if self.topDrop:
                        print("Drop it.")
                        self.task_comp = False
                        self.task_pub.publish(self.task_comp)
                        bot.trajMoveRad(self.thtDes, 2*pi/8, 2*pi/8)
                        #print(self.pos)
                        rospy.sleep(0.5)
                        if self.ifClose(self.thtDes):
                            self.vac_pub.publish(1)
                            rospy.sleep(0.1)
                            self.vac_pub.publish(2)
                            self.topDrop = False
                            self.task_comp = True
                            self.task_pub.publish(self.task_comp)
                            rospy.sleep(0.1)
                print("Let's move the pizza now.")
                self.pizzaTop = False
                #Call_Mobile_Pizza_ready()
                #if mobile_ready:
                    #self.movePizza = True
                    
            if self.movePizza:
                print("Another pic dude")
                bot.trajMoveRad(homeTht)
                self.task_pub.publish(self.task_comp)
                rospy.sleep(0.1)
                self.task_comp = False
                bot.trajMoveRad(self.thtDes)
                if self.ifClose(self.thtDes):
                    posMove = [self.pos[0], self.pos[1], ztable]
                    thtMove = deltaKin.solveIt(posMove)
                    bot.trajMoveRad(thtMove)
                    rospy.sleep(0.5)
                    if self.ifClose(self.thtMove):
                        posEnd = [0, (tab/2+2)*in2mm, ztable]
                        thtEnd = deltaKin.solveIt(posEnd)
                        bot.trajMoveRad(thtEnd)
                        rospy.sleep(0.5)
                        if self.ifClose(self.thtEnd):
                            #Call_Mobile_Pizza_pushed()
                            self.movePizza = False
                            self.punchDough = True
                            self.task_comp = True
                            
            if self.punchDough:
                bot.trajMoveRad(homeTht)
                self.task_pub.publish(self.task_comp)
                self.vac_pub.publish(0)
                rospy.sleep(0.1)
                self.task_comp = False
                self.task_pub.publish(self.task_comp)
                pos_list = [self.pos, np.array(self.pos)+np.array([20,0,0]), \
                            np.array(self.pos)+np.array([0,20,0]), \
                            np.array(self.pos)-np.array([20,0,0]), \
                            np.array(self.pos)-np.array([0,20,0]), self.pos]
                tht_list = [self.thtDes, deltaKin.solveIt(pos_list[1]),\
                            deltaKin.solveIt(pos_list[2]), deltaKin.solveIt(pos_list[3]),
                            deltaKin.solveIt(pos_list[4]), self.thtDes]
                for i in range(6):
                    bot.trajMoveRad(tht_list[i])
                    if self.ifClose(self.thtDes):
                        posPunch = [tht_list[i][0], tht_list[i][1], ztable]
                        thtPunch = deltaKin.solveIt(posPunch)
                        bot.trajMoveRad(thtPunch)
                        rospy.sleep(0.5)
                self.task_comp = True
                self.task_pub.publish(self.task_comp)
                self.punchDough = False
                                
            #elif parmShake:
                #pass
                ## Mebbe
            
            else:
                pass

if __name__ == '__main__':
    try:
        odrive_node = ODriveNode()
        odrive_node.main_loop()
    except rospy.ROSInterruptException:
        pass
