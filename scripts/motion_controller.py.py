#!/usr/bin/env python

import rospy
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from geometry_msgs.msg import Twist
from turtlebot3_msgs.msg import SensorState
from geometry_msgs.msg import Quaternion
from std_msgs.msg import Float64
import math

#global variables

roll=pitch=yaw=0.0
t_yaw=0
t_yaw_deg=0.0
kP=0.4
command=Twist()
flag=1
positioning=0
ready=0
first_time=0 #esta variable me indica cuando si realizar go front y cuando ya se ha realizado
flag2=0


class FoS():
    def __init__(self):
        self.pub_mov=rospy.Publisher('/cmd_vel',Twist,queue_size=1)
        self.sub_bump=rospy.Subscriber('sensor_state', SensorState, self.get_bump, queue_size = 1)
        self.sub_direction=rospy.Subscriber('angle', Float64,self.get_doa)
        self.sub_orientation=rospy.Subscriber('/odom',Odometry,self.get_rotation)                                                                        
        self.fos()
    def get_bump(self,sensor): #UNICAMENTE CUANDO SE HA ESCUCHADO UDABOT
        global flag2
        if (sensor.illumination==1):
            flag2=1
        else:
            flag2=0
    def get_doa(self,data):
        #to define
        global t_yaw_deg
        
        yaw_deg=yaw*180/math.pi
        
        if (t_yaw_deg+5.0>yaw_deg and t_yaw_deg-5.0<yaw_deg and flag==1 and flag2==1):
            t_yaw_deg=float(data.data)

            if(t_yaw_deg>0 and t_yaw_deg<=180):
                t_yaw_deg=-t_yaw_deg
            else:
                t_yaw_deg=360-t_yaw_deg
   
        else:
            t_yaw_deg=t_yaw_deg
        
    def get_rotation(self,msg):
        global roll,pitch,yaw
        orientation_q=msg.pose.pose.orientation
        orientation_list=[orientation_q.x,orientation_q.y, orientation_q.z, orientation_q.w]
        (roll,pitch, yaw)=euler_from_quaternion(orientation_list)
        self.rotate()

    def rotate(self):
        global command, t_yaw, t_yaw_deg,flag, positioning, ready,first_time
        t_yaw=(t_yaw_deg*math.pi)/180.0
        yaw_deg=yaw*180.0/math.pi
        if(t_yaw<=math.pi and t_yaw>=-math.pi): #compruebo que sea un valor valido entre -180 y 180
            if (t_yaw_deg+5>yaw_deg and t_yaw_deg-5<yaw_deg): #si esta en la posicion con tolerancia 5 grados
                command.angular.z=0 #dejo de girar
                self.pub_mov.publish(command)
                print('Im here!')
                print('Target=' ,t_yaw_deg,' Current: ', yaw_deg)
                if(t_yaw_deg<5 and t_yaw_deg>-5): #esta es la posicion inicial, no debo avanzar
                    self.stop()
                    flag=1
                else: #por otro lado, cuando encontro el angulo y es distinto a la posicion inicial, avanza
                    flag=0
                    self.advance()
                    ready=1
            elif (positioning==1):
                flag=0
                self.advance()
            elif (positioning==0):
                flag=0
                command.linear.x = 0.0
                command.angular.z=kP*(t_yaw-yaw)
                self.pub_mov.publish(command)
                print('Target=' ,t_yaw_deg,' Current: ', yaw_deg)
        else:
            self.stop()
            print('It looks like a problem, it is not a valid value')
            print('Target=' ,t_yaw_deg,' Current: ', yaw_deg)
            t_yaw_deg=0.0

    def advance(self):
        self.sub_ir=rospy.Subscriber('sensor_state', SensorState, self.get_ir, queue_size = 1)

    def get_ir(self, sensor):
        global command, t_yaw_deg, flag, positioning,ready, first_time
        yaw_deg=yaw*180/math.pi
        if (ready ==1 and flag==0):
            if first_time==0: #es la primera vez
                if sensor.cliff==0:
                    self.gofront()
                else:
                    self.goBack()
                    self.stop()
                    first_time=1
            else:
                 if sensor.bumper==1: #me muevo solamente cuando vuelva al idle
                    if sensor.cliff==3:
                        print('Oh, a cliff, save meee')
                        self.goBack()
                        rospy.sleep(1)
                        self.stop()
                        t_yaw_deg=0.0
                        flag=1
                        positioning=0
                        ready=0
                        first_time=0
                
                    elif sensor.cliff==2:
                        self.adjust_left()
                        positioning=1
            
                    elif sensor.cliff==1:
                        self.adjust_right()
                        positioning=1
                    else:
                        self.gofront()
                
#funciones para el movimiento
    def gofront(self):
        global command
        linear_vel = -0.05
        command.linear.x = linear_vel
        command.angular.z=0
        self.pub_mov.publish(command)
    def goBack(self):
        global command
        linear_vel = 0.05
        command.linear.x = linear_vel
        command.angular.z=0
        self.pub_mov.publish(command)
    def adjust_left(self):  #adjust lef and adjust right sirven para oientar el robot recto antes de retroceder
        global command
        command.linear.x = 0.0
        command.angular.z=0.2
        self.pub_mov.publish(command)
    def adjust_right(self):
        global command
        command.linear.x = 0.0
        command.angular.z=-0.2
        self.pub_mov.publish(command)
    def stop(self):
        global command
        command.linear.x = 0.0
        command.angular.z=0
        self.pub_mov.publish(command)

#funcion para 
    def fos(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            rate.sleep()


def main():
    rospy.init_node('follower')
    try:
        fos=FoS()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()
