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
balRad = 1.25 # inches
pizRad = 4.0 # inches
tab = 70 # inches

in2mm = 25.4
mm2in = 1/in2mm
in2m = in2mm/1000

#TODO: Put these values in, fool
zhome = 0
ztable = 0
zdrop = 0

class ODriveNode(object):
    def __init__(self):
        rospy.init_node('odrive')
        
        self.command_queue = Queue.Queue(maxsize=5)
        rospy.Subscriber('/pos', Float32MultiArray, self.cmd_callback)
        #rospy.Subscriber('/what_task', Int8, self.task_callback)
        
        self.vac_pub = rospy.Publisher('/toggle_vac', Int8, queue_size = 1)
        self.task_pub = rospy.Publisher('/task_complete', Bool, queue_size = 1)
        
        self.thtDes = [0, 0, 0]
        bot.connect_all()
        print('ready')
        
        self.task_comp = False

    def cmd_callback(self, msg):
        #print('hello')
        if self.task_comp: # Only get the position values if task is complete
            self.pos = msg.data # Get the proper value from the dictionary
            if self.pizzaTop == True:
                self.pos.append(ztable)
            elif self.topDrop == True:
                self.pos.append(zdrop)
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
            self.pizzaTop = True
            self.topDrop = False
            self.movePizza = False
            self.punchDough = False
            self.parmShake = False
            
            try:
                main_rate.sleep()
            except rospy.ROSInterruptException:
                break
            if self.pizzaTop:
                for i in range(9):
                    self.task_comp = False
                    self.task_pub.publish(self.task_comp)
                    bot.trajMoveRad(self.thtDes, 2*pi/8, 2*pi/8)                
                    self.vac_pub.publish(vacInt)
                    rospy.sleep(0.1)
                    if self.ifClose(self.thtDes):
                        self.vac_pub.publish(0)
                        self.task_comp = True
                        self.task_pub.publish(self.task_comp)
                        self.topDrop = True
                    if self.topDrop:
                        self.task_comp = False
                        self.task_pub.publish(self.task_comp)
                        bot.trajMoveRad(self.thtDes, 2*pi/8, 2*pi/8) 
                        rospy.sleep(0.1)
                        if self.ifClose(self.thtDes):
                            self.vac_pub.publish(1)
                            rospy.sleep(0.1)
                            self.vac_pub.publish(2)
                            self.task_comp = True
                            self.topDrop = False
                self.movePizza = True
            if self.movePizza:
                pass # TODO: Write code
                
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
