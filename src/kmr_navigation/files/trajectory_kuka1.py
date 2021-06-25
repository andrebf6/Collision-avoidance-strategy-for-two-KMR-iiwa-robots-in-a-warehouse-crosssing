#!/usr/bin/env python3

# Copyright 2021 Andrea Bravo.
# Copyright 2021 Universitat Politècnica de Catalunya.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

# Description: Driving the robot giving a set of desired positions

import rclpy # provides canonical Python API for interacting with ROS 
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist, Pose  # provides messages for common geometric primitives such as points, vectors, and poses.
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String # provides standard messages
import numpy as np
import math
from numpy import linalg as LA

msg = """
Robot KUKA1 following a desired trajectory in a warehouse crossing.

"""
def listener_callback(odom_data):
    xr= odom_data.pose.pose.position.x
    yr= odom_data.pose.pose.position.y
    orienx= odom_data.pose.pose.orientation.x
    orieny= odom_data.pose.pose.orientation.y
    orienz= odom_data.pose.pose.orientation.z
    orienw= odom_data.pose.pose.orientation.w

    siny_cosp = 2 * (orienw * orienz + orienx * orieny)
    cosy_cosp = 1 - 2 * (orieny * orieny + orienz * orienz)
    yawc = np.arctan2(siny_cosp, cosy_cosp)

    pr[0]=xr; pr[1]=yr; yaw[0]= yawc

def listener_callback1(scan_data):
    read=scan_data.ranges
    nr=len(scan_data.ranges)
    readings[:]=read[:]


if __name__=="__main__":# Proteger parte del código. Assegura que el bloque solo se ejecutará si tu módulo es el programa principal.
    rclpy.init() #initialize
    # Create node : KUKA_travels
    node = rclpy.create_node('KUKA1_travels')

     # The node is subscribed to the topic /odom, and recives messages of the type Pose and Twist, with a "queue size" of 10 queued messages
    sub = node.create_subscription(Odometry, '/robot1/odom',listener_callback, 10)
    # The node is subscribed to the topic /scan, and recives messages of the type Laserscan, with the quality of service specified by the publisher (GAZEBO plugin)
    sub2 = node.create_subscription(LaserScan,'/robot1/scan',listener_callback1, rclpy.qos.qos_profile_sensor_data)
    
    # The node publishes messages of the type Twist, to the topic /cmd_vel, with a "queue size" of 10 queued messages
    pub = node.create_publisher(Twist,'/robot1/cmd_vel', 10)

    #Initialization values
    vx = 0.0
    vy = 0.0
    speed = 0.3

    tol=1e-2
    pr=np.ones(2)*2000
    dl=np.ones(2); dl[0]=0.483; dl[1]=-0.258
    yaw=np.ones(1)*2000
    rotz=np.zeros((2,2))*2000

    twist = Twist() # create a twist message
    twist.linear.x = vx*speed # add linear x value
    twist.linear.y = vy*speed 
    twist.linear.z = 0.0
    twist.angular.x = 0.0   # add angular x value
    twist.angular.y = 0.0
    twist.angular.z = 0.0
    pub.publish(twist) # Publish message to the subscriber

    #values for the scanner
    amin=-2.3562
    amax=2.3562
    nsamp=810
    ainc=(amax-amin)/nsamp
    readings=np.ones(nsamp)*2000
    angles=np.linspace(amin,amax,nsamp)
    dd=2
    theta=-0.78539816339

    # Important points
    pa=np.ones(2);pa[0]=2.5; pa[1]=1.2
    pb=np.ones(2);pb[0]=2.1; pb[1]=3

    pl=np.ones(2)*2000
    pma=np.ones(2)*2000; pma[0]=pa[0]
    pmb=np.ones(2)*2000; pmb[0]=pb[0]

    try: # The try block lets you test a block of code for errors.
        print(msg) # missatge inicial
        nw=5
        waypoints=np.ones((nw,2))
        waypoints[0,0]=2.758; waypoints[0,1]=2.1
        waypoints[1,0]=2.5; waypoints[1,1]=2.1
        waypoints[2,0]=1.6; waypoints[2,1]=1.2
        waypoints[3,0]=1.6; waypoints[3,1]=0.942
        waypoints[4,0]=1.6; waypoints[4,1]=-3

        for kk in range(nw):
            pd=waypoints[kk,:]
            print("desired position:", pd)

            p=0.0
            if kk ==0: # Collision detection activated
                while(1):
                    rclpy.spin_once(node,executor=None, timeout_sec=1)

                    if pr[0]!=2000 and pr[1]!=2000:
                        dv=(pd-pr)/LA.norm(pd-pr) # unitary vector that gives the direction in the x-y plain
                        rotz[0,0]=math.cos(yaw[0]); rotz[0,1]=math.sin(yaw[0])
                        rotz[1,0]=-math.sin(yaw[0]); rotz[1,1]=math.cos(yaw[0])
                        dv=rotz.dot(dv) # Change the velocity comands to the base-footprint frame

                        vx=float(dv[0])
                        vy=float(dv[1])

                        twist = Twist() # create a twist message
                        twist.linear.x = vx*speed # add linear x value
                        twist.linear.y = vy*speed 
                        twist.linear.z = 0.0
                        twist.angular.x = 0.0   # add angular x value
                        twist.angular.y = 0.0
                        twist.angular.z = 0.0
                        pub.publish(twist) # Publish message to the subscriber

                        if (p%15) == 0:
                            print("real position:", pr)
                            print('yaw', yaw[0] )
                            print(twist)
                        p=p+1

                        if LA.norm(pd-pr)< tol:
                            print("Goal achieved")
                            
                            speed = 0.3
                            vx = 0.0
                            vy = 0.0
                            
                            twist = Twist() # create a twist message
                            twist.linear.x = vx*speed # add linear x value
                            twist.linear.y = vy*speed 
                            twist.linear.z = 0.0
                            twist.angular.x = 0.0   # add angular x value
                            twist.angular.y = 0.0
                            twist.angular.z = 0.0
                            pub.publish(twist) # Publish message to the subscriber

                            break

                        # Compute cone
                        inv_rotz=np.transpose(rotz)
                        pl=pr+inv_rotz.dot(dl)
                                    
                        la1=LA.norm(pa-pl)
                        lb1=LA.norm(pb-pl)

                        pma[1]=pl[1]
                        pmb[1]=pl[1]

                        salpha1=LA.norm(pa-pma)/la1; alpha1=math.asin(salpha1) 
                        salpha2=LA.norm(pb-pmb)/lb1; alpha2=math.asin(salpha2)

                        blalpha1=3.14159265359-yaw[0]-theta+alpha1
                        tt=blalpha1//6.28318530718; blalpha1=blalpha1-tt*6.28318530718
                        if blalpha1 > 3.14159265359:
                            blalpha1=blalpha1-6.28318530718
                        if blalpha1 < -3.14159265359:
                            blalpha1=blalpha1+6.28318530718

                        blalpha2=3.14159265359-yaw[0]-theta-alpha2
                        uu=blalpha2//6.28318530718; blalpha2=blalpha2-uu*6.28318530718
                        if blalpha2 > 3.14159265359:
                            blalpha2=blalpha2-6.28318530718
                        if blalpha2 < -3.14159265359:
                            blalpha2=blalpha2+6.28318530718

                        i1= int((blalpha1+amax)/ainc)
                        i2= int((blalpha2+amax)/ainc)
                        i_readings=readings[i2:i1]
                                    
                        if np.any(i_readings<dd) == True and i_readings[5]!=2000:
                            col_idx=np.where(i_readings<dd)[0]
                            col_ang=angles[col_idx]
                            col_dis=i_readings[col_idx]
                            print('Collision danger!!')
                            #print(col_ang*57.2957795131,col_dis)
                    
            elif kk ==2: # Collision detection deactivated + turning
                turn=0.1
                speed=0.1
                while(1):
                    rclpy.spin_once(node,executor=None, timeout_sec=1)
                    
                    dv=(pd-pr)/LA.norm(pd-pr) # unitary vector that gives the direction in the x-y plain
                    rotz[0,0]=math.cos(yaw[0]); rotz[0,1]=math.sin(yaw[0])
                    rotz[1,0]=-math.sin(yaw[0]); rotz[1,1]=math.cos(yaw[0])
                    dv=rotz.dot(dv) # Change the velocity comands to the base-footprint frame

                    vx=float(dv[0])
                    vy=float(dv[1])

                    if abs(yaw[0]) > tol:
                        if yaw[0]<0:
                            wz=1
                        if yaw[0]>0:
                            wz=-1
                    elif abs(yaw[0]) < tol:
                        wz=0

                    twist = Twist() # create a twist message
                    twist.linear.x = vx*speed # add linear x value
                    twist.linear.y = vy*speed 
                    twist.linear.z = 0.0
                    twist.angular.x = 0.0   # add angular x value
                    twist.angular.y = 0.0
                    twist.angular.z = wz*turn
                    pub.publish(twist) # Publish message to the subscriber

                    if (p%15) == 0:
                        print("real position:", pr)
                        print('yaw', yaw[0] )
                        print(twist)
                    p=p+1

                    if LA.norm(pd-pr)< tol:
                        print("Goal achieved")
                            
                        speed = 0.3
                        vx = 0.0
                        vy = 0.0
                            
                        twist = Twist() # create a twist message
                        twist.linear.x = vx*speed # add linear x value
                        twist.linear.y = vy*speed 
                        twist.linear.z = 0.0
                        twist.angular.x = 0.0   # add angular x value
                        twist.angular.y = 0.0
                        twist.angular.z = 0.0
                        pub.publish(twist) # Publish message to the subscriber

                        break

            else : # Collision detection deactivated
                while(1):
                    rclpy.spin_once(node,executor=None, timeout_sec=1)

                    dv=(pd-pr)/LA.norm(pd-pr) # unitary vector that gives the direction in the x-y plain
                    rotz[0,0]=math.cos(yaw[0]); rotz[0,1]=math.sin(yaw[0])
                    rotz[1,0]=-math.sin(yaw[0]); rotz[1,1]=math.cos(yaw[0])
                    dv=rotz.dot(dv) # Change the velocity comands to the base-footprint frame

                    vx=float(dv[0])
                    vy=float(dv[1])

                    twist = Twist() # create a twist message
                    twist.linear.x = vx*speed # add linear x value
                    twist.linear.y = vy*speed 
                    twist.linear.z = 0.0
                    twist.angular.x = 0.0   # add angular x value
                    twist.angular.y = 0.0
                    twist.angular.z = 0.0
                    pub.publish(twist) # Publish message to the subscriber

                    if (p%15) == 0:
                        print("real position:", pr)
                        print('yaw', yaw[0] )
                        print(twist)
                    p=p+1

                    if LA.norm(pd-pr)< tol:
                        print("Goal achieved")
                            
                        speed = 0.3
                        vx = 0.0
                        vy = 0.0
                            
                        twist = Twist() # create a twist message
                        twist.linear.x = vx*speed # add linear x value
                        twist.linear.y = vy*speed 
                        twist.linear.z = 0.0
                        twist.angular.x = 0.0   # add angular x value
                        twist.angular.y = 0.0
                        twist.angular.z = 0.0
                        pub.publish(twist) # Publish message to the subscriber

                        break

        print("Path followed")

    except Exception as e: # When an error/exception occures, handle it using the except block of code
        print(e)

    finally: # The finally block will be executed regardless if the try block raises an error or not.
        vx = 0.0
        vy = 0.0
        speed = 0.3

        twist = Twist() # create a twist message
        twist.linear.x = vx*speed # add linear x value
        twist.linear.y = vy*speed 
        twist.linear.z = 0.0
        twist.angular.x = 0.0   # add angular x value
        twist.angular.y = 0.0
        twist.angular.z = 0.0
        pub.publish(twist) # Publish message to the subscriber