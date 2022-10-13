#!/usr/bin/env python3

import rospy
import numpy as np
import random
from std_msgs.msg import Float64

class Mazinger():
    def __init__(self):
        # Declaring publishers
        self.joint_pub_1 = rospy.Publisher('/arm_robot_1/joint1_position_controller/command', Float64, queue_size=1)
        self.joint_pub_2 = rospy.Publisher('/arm_robot_1/joint2_position_controller/command', Float64, queue_size=1)
        self.joint_pub_3 = rospy.Publisher('/arm_robot_1/joint3_position_controller/command', Float64, queue_size=1)
        self.joint_pub_4 = rospy.Publisher('/arm_robot_1/joint4_position_controller/command', Float64, queue_size=1)

        # Configuration and initial states
        self.joint_1 = 0.0
        self.joint_2 = 0.0
        self.joint_3 = 0.0
        self.joint_4 = 0.0

        self.rate = rospy.Rate(50)  # 50Hz
        self.ctrl_c = False

        rospy.on_shutdown(self.shutdownhook)

    def move_arm_robot(self):
        aux = np.concatenate([np.arange(0,1.57,0.01),np.arange(1.57,-1.57,-0.01),np.arange(-1.57,0,0.01)])
        flag = 0
        cnt = 0

        while not self.ctrl_c:
            # Moving First Joint
            if (flag == 0):
                self.joint_1 = aux[cnt]
                if(cnt > 626): 
                    flag = 1
                    cnt = 0
                else: 
                    cnt += 1
            
            # Moving Second Joint
            if (flag == 1):
                self.joint_2 = aux[cnt]
                if(cnt > 626): 
                    flag = 2
                    cnt = 0
                else: 
                    cnt += 1

            # Moving Third Joint
            if (flag == 2):
                self.joint_3 = aux[cnt]
                if(cnt > 626): 
                    flag = 3
                    cnt = 0
                else: 
                    cnt += 1

            # Moving Fourth Joint
            if (flag == 3):
                self.joint_4 = aux[cnt]
                if(cnt > 626): 
                    flag = 4
                    cnt = 0
                else: 
                    cnt += 1

            # Moving Joints Randomly
            if (flag == 4):
                self.joint_1 = aux[random.randint(0,626)]
                self.joint_2 = aux[random.randint(0,626)]
                self.joint_3 = aux[random.randint(0,626)]
                self.joint_4 = aux[random.randint(0,626)]
                rospy.sleep(5)

            print('Position Joint #1: ', round(self.joint_1*(180/np.pi),2), '[°]')
            print('Position Joint #2: ', round(self.joint_2*(180/np.pi),2), '[°]')
            print('Position Joint #3: ', round(self.joint_3*(180/np.pi),2), '[°]')
            print('Position Joint #4: ', round(self.joint_4*(180/np.pi),2), '[°]')
            print('----------------------------------------------')

            self.joint_pub_1.publish(self.joint_1)
            self.joint_pub_2.publish(self.joint_2)
            self.joint_pub_3.publish(self.joint_3)
            self.joint_pub_4.publish(self.joint_4)
            self.rate.sleep()

    def shutdownhook(self):
        self.joint_1 = 0.0
        self.joint_2 = 0.0
        self.joint_3 = 0.0
        self.joint_4 = 0.0

        self.joint_pub_1.publish(self.joint_1)
        self.joint_pub_2.publish(self.joint_2)
        self.joint_pub_3.publish(self.joint_3)
        self.joint_pub_4.publish(self.joint_4)

        self.ctrl_c = True


if __name__ == '__main__':
    rospy.init_node('project2_test', anonymous=True)
    rosbot_object = Mazinger()
    try:
        rosbot_object.move_arm_robot()
    except rospy.ROSInterruptException:
        pass
