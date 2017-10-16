#!/usr/bin/env python
# Software License Agreement (BSD License)
#
# Copyright (c) 2008, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Willow Garage, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Revision $Id$


import rospy
from std_msgs.msg import String

# from BNO055 import *
from datetime import datetime
import json 
import numpy as np
import time
import Adafruit_PCA9685
import csv
from subprocess import Popen, PIPE

# return the init content of imu data
def imu_init():
    IMU_file = './IMU_data/' + fileName + '.csv'
    tick = datetime.now()

    #with open(IMU_file, 'a') as csvfile:
    #        IMU_writer = csv.writer(csvfile, delimiter = ' ')
    #        IMU_writer.writerow("The flat matrix is " + str(flatMatrix) + " \n")
    
    with open(IMU_file, 'a') as csvfile:
            IMU_writer = csv.writer(csvfile, delimiter = ' ')
            flatMatrixWrite = [flatMatrix[0][0], flatMatrix[0][1], flatMatrix[0][2], flatMatrix[1][0],  flatMatrix[1][1], flatMatrix[1][2]]
            IMU_writer.writerow(flatMatrixWrite)

    # IMU_output.writerow("The flat matrix is " + str(flatMatrix) + " \n")

    #this determines the number of times the IMU reads data
    IMU_cycleThreshold = 5
    IMU_cycleCounter = 1 
    return IMU_cycleThreshold, IMU_cycleCounter

#read data from IMU
def imu_getdata(IMU_cycleCounter, IMU_cycleThreshold):
    if IMU_cycleCounter == IMU_cycleThreshold:
        currentTimeD = tick - datetime.now()
        currentReading, errorCounter = produceVector(bno)
        currentEditedReading = currentReading - flatMatrix

        dataWrite = [currentTimeD.total_seconds(), currentEditedReading[0][0], currentEditedReading[0][1], currentEditedReading[0][2], currentEditedReading[1][0], currentEditedReading[1][1], currentEditedReading[1][2], chassisOutput1, tibiaOutput1, chassisOutput2, tibiaOutput2, chassisOutput3, tibiaOutput3, chassisOutput4, tibiaOutput4] 
        
        #IMU_output.writerow(str(currentTimeD.total_seconds()) + " ")
        #IMU_output.write(str(currentReading[0])+ ' ' + str(currentReading[1]))
        #IMU_output.writerow(str(currentEditedReading[0]) + ' ' + str(currentEditedReading[1]))

        #the below line also outputs tibia and femur positions
        #IMU_output.writerow(str(chassisOutput1) + " " + str(tibiaOutput1) + " ")
        #IMU_output.writerow(str(chassisOutput2) + " " + str(tibiaOutput2) + " ")
        #IMU_output.writerow(str(chassisOutput3) + " " + str(tibiaOutput3) + " ")
        #IMU_output.writerow(str(chassisOutput4) + " " + str(tibiaOutput4) + " ")

        with open(IMU_file, 'a') as csvfile:
                IMU_writer = csv.writer(csvfile, delimiter = ' ')
                IMU_writer.writerow(dataWrite)
        #IMU_output.writerow('\n')

def motor_getsignal(motor_type, chassisOutput1, tibiaOutput1, chassisOutput2, tibiaOutput2,
                chassisOutput3, tibiaOutput3, chassisOutput4, tibiaOutput4):
    motor_signal_data  = {}
    motor_signal_data['motor_type'] = motor_type

    # 4 sequence of the singals (equivalent to output_motor(chassisOutput1, tibiaOutput1, 0, 1)):
    motor_signal_data['motor_output_1'] = {}
    # 4 arguments for the function outputMotor(chassisOutput, tibiaOutput, chassisNum, tibiaNum)
    motor_signal_data['motor_output_1']['chassisOutput'] = chassisOutput1
    motor_signal_data['motor_output_1']['tibiaOutput'] = tibiaOutput1
    motor_signal_data['motor_output_1']['chassisNum'] = 0
    motor_signal_data['motor_output_1']['tibiaNum'] = 1

    motor_signal_data['motor_output_2'] = {}
    # 4 arguments for the function outputMotor(chassisOutput, tibiaOutput, chassisNum, tibiaNum)
    motor_signal_data['motor_output_2']['chassisOutput'] = chassisOutput2
    motor_signal_data['motor_output_2']['tibiaOutput'] = tibiaOutput2
    motor_signal_data['motor_output_2']['chassisNum'] = 2
    motor_signal_data['motor_output_2']['tibiaNum'] = 3

    motor_signal_data['motor_output_3'] = {}
    # 4 arguments for the function outputMotor(chassisOutput, tibiaOutput, chassisNum, tibiaNum)
    motor_signal_data['motor_output_3']['chassisOutput'] = chassisOutput3
    motor_signal_data['motor_output_3']['tibiaOutput'] = tibiaOutput3
    motor_signal_data['motor_output_3']['chassisNum'] = 4
    motor_signal_data['motor_output_3']['tibiaNum'] = 5

    motor_signal_data['motor_output_4'] = {}
    # 4 arguments for the function outputMotor(chassisOutput, tibiaOutput, chassisNum, tibiaNum)
    motor_signal_data['motor_output_4']['chassisOutput'] = chassisOutput4
    motor_signal_data['motor_output_4']['tibiaOutput'] = tibiaOutput4
    motor_signal_data['motor_output_4']['chassisNum'] = 6
    motor_signal_data['motor_output_4']['tibiaNum'] = 7
    return motor_signal_data




def spline_run(chassis, tibia, phase, bno, flatMatrix):
    # publisher init
    motor_pub = rospy.Publisher('motor_signal', String, queue_size=10)
    imu_pub = rospy.Publisher('motor_signal', String, queue_size=10)
    rospy.init_node('spline_runner', anonymous=True)
    rate = rospy.Rate(10) # 10hz

    IMU_cycleThreshold, IMU_cycleCounter = imu_init()
    
    # init leg counter *(may put in a separate function)
    leg1_counter = ((4.0*phase)/360.0)*len(chassis)
    leg2_counter = ((3.0*phase)/360.0)*len(chassis)
    leg3_counter = ((2.0*phase)/360.0)*len(chassis)
    leg4_counter = ((1.0*phase)/360.0)*len(chassis)

    #In the case of a phase greater than 180, leg1 and leg2 must be corrected back to 0 and 180 degrees
    if(phase >= 180):
        leg2_counter = ((1.0*phase)/360.0)*len(chassis)
        leg1_counter = ((2.0*phase)/360.0)*len(chassis)
        

    for i in range(3):
        for i in range(len(chassis)):
            if leg1_counter >= len(chassis):
                leg1_counter -= len(chassis)
            if leg2_counter >= len(chassis):
                leg2_counter -= len(chassis)
            if leg3_counter >= len(chassis):
                leg3_counter -= len(chassis)
            if leg4_counter >= len(chassis):
                leg4_counter -= len(chassis)

            #determines whether the IMU will read in the current cycle
            if dataIMU==1:
                if IMU_cycleCounter == IMU_cycleThreshold:
                        IMU_cycleCounter = 1
                else:
                        IMU_cycleCounter+= 1
            
            #run for percentages
            #if motor_type == 1 or motor_type == 2:
            chassisOutput1 = chassis[leg1_counter]*(motor0_max-motor0_min) + motor0_min
            tibiaOutput1 = tibia[leg1_counter]*(motor1_max-motor1_min)+motor1_min
            
            chassisOutput2 = chassis[leg2_counter]*(motor2_max-motor2_min) + motor2_min
            tibiaOutput2 = tibia[leg2_counter]*(motor3_max-motor3_min)+motor3_min

            chassisOutput3 = chassis[leg3_counter]*(motor4_max-motor4_min) + motor4_min
            tibiaOutput3 = tibia[leg3_counter]*(motor5_max-motor5_min)+motor5_min

            chassisOutput4 = chassis[leg4_counter]*(motor6_max-motor6_min) + motor6_min
            tibiaOutput4 = tibia[leg4_counter]*(motor7_max-motor7_min)+motor7_min   

            # send signal to motor
            # self.motor.output_motor(chassisOutput1, tibiaOutput1, 0, 1)
            # self.motor.output_motor(chassisOutput2, tibiaOutput2, 2, 3)                 
            # self.motor.output_motor(chassisOutput3, tibiaOutput3, 4, 5)
            # self.motor.output_motor(chassisOutput4, tibiaOutput4, 6, 7)

            # get motor signal data in dictionary in the format:

            '''motor_signal_data = {
            'motor_type': 1 or 2
            'motor_output_1': {
                chassisOutput, 
                tibiaOutput, 
                chassisNum, 
                tibiaNum
            }
            'motor_output_2': {
                chassisOutput, 
                tibiaOutput, 
                chassisNum, 
                tibiaNum
            }
            'motor_output_3': {
                chassisOutput, 
                tibiaOutput, 
                chassisNum, 
                tibiaNum
            }
            'motor_output_4': {
                chassisOutput, 
                tibiaOutput, 
                chassisNum, 
                tibiaNum
            }
            }'''


            singal1 = motor_getsignal(1, chassisOutput1, tibiaOutput1, chassisOutput2, \
                tibiaOutput2, chassisOutput3, tibiaOutput3, chassisOutput4, tibiaOutput4)
            rospy.loginfo(str(singal1))
            motor_pub.publish(str(singal1))
            rate.sleep()

            singal2 = motor_getsignal(2, chassisOutput1, tibiaOutput1, chassisOutput2, \
                tibiaOutput2, chassisOutput3, tibiaOutput3, chassisOutput4, tibiaOutput4)
            rospy.loginfo(str(singal2))
            motor_pub.publish(str(singal2))
            rate.sleep()

            # #read data from IMU 
            # depend on motor type?
            # imu = imu_getdata(IMU_cycleCounter, IMU_cycleThreshold)

            #run for motor angles
            #elif self.motor.motor_type == 3:
            chassisOutput1 = chassis[leg1_counter]
            tibiaOutput1 = tibia[leg1_counter]
            
            chassisOutput2 = chassis[leg2_counter]
            tibiaOutput2 = tibia[leg2_counter]

            chassisOutput3 = chassis[leg3_counter]
            tibiaOutput3 = tibia[leg3_counter]

            chassisOutput4 = chassis[leg4_counter]
            tibiaOutput4 = tibia[leg4_counter]  
            
            # self.motor.output_motor(chassisOutput1, tibiaOutput1, 0, 1)
            # self.motor.output_motor(chassisOutput2, tibiaOutput2, 2, 3) 
            # self.motor.output_motor(chassisOutput3, tibiaOutput3, 4, 5)
            # self.motor.output_motor(chassisOutput4, tibiaOutput4, 6, 7)

            singal3 = motor_getsignal(2, chassisOutput1, tibiaOutput1, chassisOutput2, \
                tibiaOutput2, chassisOutput3, tibiaOutput3, chassisOutput4, tibiaOutput4)
            rospy.loginfo(str(singal3))
            motor_pub.publish(str(singal3))
            rate.sleep()

             #read data from IMU
            imu_getdata(IMU_cycleCounter, IMU_cycleThreshold)

            leg1_counter+=1
            leg2_counter+=1
            leg3_counter+=1
            leg4_counter+=1

if __name__ == '__main__':
    try:
        spline_run()
    except rospy.ROSInterruptException:
        pass
