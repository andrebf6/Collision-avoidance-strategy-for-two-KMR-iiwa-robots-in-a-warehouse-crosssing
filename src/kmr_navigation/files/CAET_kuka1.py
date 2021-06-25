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
Robot KUKA1 following a desired trajectory in a warehouse crossing while avoiding KUKA2.

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

    pr1[0]=xr; pr1[1]=yr; yaw1[0]= yawc

def listener_callback1(scan_data):
    read=scan_data.ranges
    readings[:]=read[:]
    global scan
    scan= True

    time_sec=scan_data.header.stamp.sec
    time_nan=scan_data.header.stamp.nanosec; time_nan=time_nan*1e-9
    global t
    t=time_sec+time_nan

def listener_callback2(odom_data2):
    xr2= odom_data2.pose.pose.position.x
    yr2= odom_data2.pose.pose.position.y
    orienx2= odom_data2.pose.pose.orientation.x
    orieny2= odom_data2.pose.pose.orientation.y
    orienz2= odom_data2.pose.pose.orientation.z
    orienw2= odom_data2.pose.pose.orientation.w

    siny_cosp = 2 * (orienw2 * orienz2 + orienx2 * orieny2)
    cosy_cosp = 1 - 2 * (orieny2 * orieny2 + orienz2 * orienz2)
    yawc2 = np.arctan2(siny_cosp, cosy_cosp)

    vxr2= odom_data2.twist.twist.linear.x
    vyr2= odom_data2.twist.twist.linear.y

    pr2_real[0]=xr2; pr2_real[1]=yr2; yaw2[0]= yawc2
    v2_real[0]=vxr2; v2_real[1]=vyr2



if __name__=="__main__":# Proteger parte del código. Assegura que el bloque solo se ejecutará si tu módulo es el programa principal.
    rclpy.init() #initialize
    # Create node : KUKA_travels
    node = rclpy.create_node('KUKA1_travels')

     # The node is subscribed to the topic /odom, and recives messages of the type Pose and Twist, with a "queue size" of 10 queued messages
    sub = node.create_subscription(Odometry, '/robot1/odom',listener_callback, 10)
    # The node is subscribed to the topic /scan, and recives messages of the type Laserscan, with the quality of service specified by the publisher (GAZEBO plugin)
    sub2 = node.create_subscription(LaserScan,'/robot1/scan',listener_callback1, rclpy.qos.qos_profile_sensor_data)
    # The node is subscribed to the topic /odom, and recives messages of the type Pose and Twist, with a "queue size" of 10 queued messages
    sub3 = node.create_subscription(Odometry, '/robot2/odom',listener_callback2, 10)
    
    # The node publishes messages of the type Twist, to the topic /cmd_vel, with a "queue size" of 10 queued messages
    pub = node.create_publisher(Twist,'/robot1/cmd_vel', 10)

    #Initialization values for driving robot 1:
    vx1 = 0.0                                       # Unitary vector for the desired linear velocity (x component) 
    vy1 = 0.0                                       # Unitary vector for the desired linear velocity (y component)
    speed1 = 0.15                                    # Modulus of the desired linear velocity (m/s)
    dv1_o=np.zeros(2)*2000                          #Initialization value for the unitary vector for the desired linear velocity in theodom frame (used only in the first printing-p=0)

    pr1=np.ones(2)*2000                             # Position of the robot 1's centre of mass in the odom frame  (m)
    dl=np.ones(2); dl[0]=0.483; dl[1]=-0.258        # Laser B1 position in the base-footprint frame (m)
    yaw1=np.ones(1)*2000                            # Orientation of the robot in the 2D plane (rad) 
    rotz=np.zeros((2,2))*2000                       # Initialize rotation matrix (change from the odom to the base-footprint frame)
    tol=1e-2                                        # Tolerance of reached goal
                                                  
    #Scanner specifications
    amin=-2.3562                                    # Minimum scanning angle (rad)                
    amax=2.3562                                     # Maximum scanning angle (rad) 
    rmin=0.3                                        # Minimum scanning distance (m)
    rmax=10                                         # Maximum scanning distance (m)
    nsamp=810                                       # Number of samples
    ainc=(amax-amin)/nsamp                          # Angular resolution (rad)
    scan_freq=5                                     # Average scanning frequence (Hz)
    readings=np.ones(nsamp)*2000                    # Initialize sensor readings
    angles=np.linspace(amin,amax,nsamp)             # Angles for the sensor readings (rad)
    theta=-0.78539816339                            # Laser's orientation in the base-footprint frame (rad)
    rotzl=np.ones((2,2))                            # Rotation matrix (change from the base-footprint to the laser-link frame)                       
    rotzl[0,0]=math.cos(theta)
    rotzl[0,1]=math.sin(theta)
    rotzl[1,0]=-math.sin(theta)
    rotzl[1,1]=math.cos(theta)
    inv_rotzl=np.transpose(rotzl)                   # Rotation matrix (change from the laser-link to the base-footprint frame)

    global scan                                    
    scan=False                                      # Boorelian variable set to true when a laser readings are recieved
    
    # Trunacte scanner reading
    pa=np.ones(2);pa[0]=2.5; pa[1]=1.1              # Points that limit the vision cone
    pb=np.ones(2);pb[0]=2.1; pb[1]=3.2

    #Compute maximum collision-danger distance
    x_lim=-0.4                                       #Condition for the linear constrain                                   

    #Position estimate for robot 2:
    tol4=5*1e-3                                     # Tolerance for robot 1's x-component of the velocity
    tol3=0.09                                       # Tolerance for robot 2's velocity direction
    R=0.626                                         # Robot's radius
    cr=np.ones(2);cr[0]=0.8; cr[1]=2.1              # Crossing's centre

    #Velocity estimate for robot 2:
    global t
    t=2000                                          # Initialization value of the time at which the last sensor reading has been finished (in the Gazebo time frame)
    inct=1                                          # Aproximate time interval to compute the velocity
    MS=int(inct*scan_freq)                          # Samples distance to compute the velocity
    opr2=np.ones(2)*2000                            # Initialization of the array to store old position values
    SF=2                                            # Simulation time factor           

    #Collision avoidance
    tol2=1e-4                                       # Tolerance for robot 2's modulus of the velocity
    Ad=0.5                                          # Adaptation coeficient for the linear velocity modulus

    #Initialization values for robot 2:
    pr2_real=np.ones(2)*2000                        # Position of the robot 2's centre of mass in the odom frame  (m)
    v2_real=np.ones(2)*2000                         # Velocity of the robot 2's centre of mass in the odom frame  (m)
    yaw2=np.ones(1)*2000                            # Orientation of the robot 1's in the 2D plane (rad)
    rotz2=np.zeros((2,2))*2000                      # Initialize rotation matrix (change from the odom to the base-footprint frame)

    # Stop the robot at the begginging of the simulation
    twist = Twist() # create a twist message
    twist.linear.x = vx1*speed1 # add linear x value
    twist.linear.y = vy1*speed1 
    twist.linear.z = 0.0
    twist.angular.x = 0.0   # add angular x value
    twist.angular.y = 0.0
    twist.angular.z = 0.0
    pub.publish(twist) # Publish message to the subscriber

    try: # The try block lets you test a block of code for errors.
        print(msg) # Initial message

        #Define trajectory for robot1
        nw=3
        waypoints=np.ones((nw,2))
        waypoints[0,0]=2.7; waypoints[0,1]=2.1
        waypoints[1,0]=0.8; waypoints[1,1]=1.2
        waypoints[2,0]=0.8; waypoints[2,1]=-2

        for kk in range(nw):
            pd=waypoints[kk,:]
            print("desired position:", pd)

            p=0.0
            m=0.0
            if kk ==0:          # Collision detection activated
                while(1):
                    rclpy.spin_once(node,executor=None, timeout_sec=1)

                    if pr1[0]!=2000 and pr1[1]!=2000 and scan==True: # Position information has been published at least once and a new sensor reading has entred.
                        
                        #Compute unitary vector that gives the direction and sense of the desired linear velocity to drive robot1 to pd (in the odom frame)
                        dv1_o=(pd-pr1)/LA.norm(pd-pr1)                                  # unitary linear velocity vector (cartesian coordinates)
                        tphi=math.tan(dv1_o[1]/dv1_o[0])                                # unitary linear velocity vector angular component (polar coordinates)     
                        cphi=math.sin(dv1_o[0]/LA.norm(dv1_o))
                        phi=math.atan(dv1_o[1]/dv1_o[0])
                        if cphi<0:
                            phi=phi+3.14159265359
                        if tphi<0 and cphi>0:
                            phi=phi+6.28318530718   

                        rotz[0,0]=math.cos(yaw1[0]); rotz[0,1]=math.sin(yaw1[0])        # Build rotation matrix (to change from the odom to the base-footprint frame)        
                        rotz[1,0]=-math.sin(yaw1[0]); rotz[1,1]=math.cos(yaw1[0])
                        
                        dv1=rotz.dot(dv1_o)                                             # Change the velocity comands to the base-footprint frame
                        vx1=float(dv1[0])
                        vy1=float(dv1[1])

                        #Tuncate scanner readings
                        inv_rotz=np.transpose(rotz)                                     # Build rotation matrix (to change from the base-footprint to the odom frame)
                        pl1=pr1+inv_rotz.dot(dl)                                        # Determine laser B1 position in the odom frame 
                                    
                        la1=pa-pl1                                                      # Build cone of vision in the odom frame
                        lb1=pb-pl1
                        calpha1=(np.dot(la1,dv1_o))/(LA.norm(la1)*(LA.norm(dv1_o))); alpha1=math.acos(calpha1) 
                        calpha2=(np.dot(lb1,dv1_o))/(LA.norm(lb1)*(LA.norm(dv1_o))); alpha2=math.acos(calpha2)

                        blalpha1=phi+alpha1-yaw1[0]-theta                               # Pass bounding angles from the odom to the laser-link frame
                        tt=blalpha1//6.28318530718; blalpha1=blalpha1-tt*6.28318530718
                        if blalpha1 > 3.14159265359:
                            blalpha1=blalpha1-6.28318530718
                        if blalpha1 < -3.14159265359:
                            blalpha1=blalpha1+6.28318530718

                        blalpha2=phi-alpha2-yaw1[0]-theta                           
                        uu=blalpha2//6.28318530718; blalpha2=blalpha2-uu*6.28318530718
                        if blalpha2 > 3.14159265359:
                            blalpha2=blalpha2-6.28318530718
                        if blalpha2 < -3.14159265359:
                            blalpha2=blalpha2+6.28318530718

                        get_laser_index = lambda angle, angle_min, freq: int((angle-angle_min)/freq)+ int((angle-angle_min)%freq > (freq/2))
                        i1= get_laser_index(blalpha1, amin, ainc) 
                        i2=  get_laser_index(blalpha2, amin, ainc) 
                        i_readings=readings[i2:i1]
                        i_angles=angles[i2:i1] 

                        #Compute collision-dangerous distance for each reading direction:
                        oi_angles=i_angles+theta+yaw1[0]                                #Pass truncated reading angles to the odom frame
                        uu=oi_angles//6.28318530718; oi_angles=oi_angles-uu*6.28318530718; 
                        if np.any(oi_angles <0):
                            ig=np.where(oi_angles <0)
                            oi_angles[ig]=oi_angles[ig]+6.28318530718

                        cdd=(x_lim-pl1[0])/np.cos(oi_angles); cdd=abs(cdd)

                        #Detect possible collsision
                        if np.any(i_readings-cdd < 0)==True:

                            #Robot detected: Collision avoidance activated
                            print('Robot detected')
                            id=np.where(i_readings-cdd< 0); dread=i_readings[id];dang=i_angles[id]; dong=oi_angles[id]

                            if m==0: #Initialization
                                #print(cdd[id],dread,dang*57.2957795131,dong*57.2957795131)

                                #Estimate robot 2's position 
                                dc= np.amin(dread); ic=np.where(dread==dc); ac=dang[ic]; oc=dong[ic]
                                blop=np.ones(2)
                                blop[0]=(R+dc)*np.cos(ac);blop[1]=(R+dc)*np.sin(ac) #Cartesian coordinates of the detected objects (in laser-link frame)
                                fop=inv_rotzl.dot(blop) # Base-footprint frame
                                op=inv_rotz.dot(fop)    # Odom frame
                                pr2=op+pl1
                                res=oc-phi

                                #print(pr2,dv1_o[0]*speed1, phi*57.2957795131,oc*57.2957795131, res*57.2957795131)
                                if abs(dv1_o[0]*speed1)>tol4:
                                    print('robot 1 is moving in the x axis')
                                    if abs(res)>tol3:
                                        print('robot 2 is moving in the y axis')
                                        pr2[0]=cr[0]
                                    else:
                                        print('robot 2 is moving in the x axis')
                                        pr2[1]=cr[1] 
                                else:
                                    print('robot 1 is moving in the y axis')
                                    if abs(res)>tol3:
                                        print('robot 2 is moving in the x axis')
                                        pr2[1]=cr[1]
                                    else:
                                        print('robot 2 is moving in the y axis')
                                        pr2[0]=cr[0]   

                                print(pr2_real,pr2,t)
                                pr2=pr2_real        # Eliminar

                                opr2[0]=pr2[0];opr2[1]=pr2[1]
                                t_old=t

                                twist = Twist() # create a twist message
                                twist.linear.x = vx1*speed1 # add linear x value
                                twist.linear.y = vy1*speed1 
                                twist.linear.z = 0.0
                                twist.angular.x = 0.0   # add angular x value
                                twist.angular.y = 0.0
                                twist.angular.z = 0.0
                                pub.publish(twist) # Publish message to the subscriber

                            elif (m!=0 and (m%MS) == 0):
                                #print(cdd[id],dread,dang*57.2957795131,dong*57.2957795131)
                                
                                #Estimate robot 2's position 
                                dc= np.amin(dread); ic=np.where(dread==dc); ac=dang[ic]; oc=dong[ic]
                                blop=np.ones(2)
                                blop[0]=(R+dc)*np.cos(ac);blop[1]=(R+dc)*np.sin(ac) #Cartesian coordinates of the detected objects (in laser-link frame)
                                fop=inv_rotzl.dot(blop) # Base-footprint frame
                                op=inv_rotz.dot(fop)    # Odom frame
                                pr2=op+pl1
                                res=oc-phi

                                #print(pr2,dv1_o[0]*speed1, phi*57.2957795131, oc*57.2957795131, res*57.2957795131)
                                if abs(dv1_o[0]*speed1)>tol4:
                                    print('robot 1 is moving in the x axis')
                                    if abs(res)>tol3:
                                        print('robot 2 is moving in the y axis')
                                        pr2[0]=cr[0]
                                    else:
                                        print('robot 2 is moving in the x axis')
                                        pr2[1]=cr[1] 
                                else:
                                    print('robot 1 is moving in the y axis')
                                    if abs(res)>tol3:
                                        print('robot 2 is moving in the x axis')
                                        pr2[1]=cr[1]
                                    else:
                                        print('robot 2 is moving in the y axis')
                                        pr2[0]=cr[0]

                                print(pr2_real,pr2,t)
                                pr2=pr2_real          # Eliminar

                                #Estimate robot 2's velocity---> MOURE DE LLOC!!
                                v2=SF*(pr2-opr2)/(t-t_old)

                                # Eliminar
                                rotz2[0,0]=math.cos(yaw2[0]); rotz2[0,1]=-math.sin(yaw2[0])        # Build rotation matrix (to change from the odom to the base-footprint frame)        
                                rotz2[1,0]=math.sin(yaw2[0]); rotz2[1,1]=math.cos(yaw2[0])
                                v2_real=rotz2.dot(v2_real)

                                print(v2_real,v2)
                                v2=v2_real # Eliminar
                                speed2=LA.norm(v2)
                                print(speed2)
                                
                                opr2[0]=pr2[0];opr2[1]=pr2[1]
                                t_old=t

                                #Collision avoidance algorithm
                                if pr2[0]<2.5 and pr2[0]>-0.5 and pr2[1]>1.1 and pr2[1]<3.2: # robot 2 is in region A
                                    print('robot 2 is in region A')
                                    
                                    if (pr1[0]<4 and pr1[0]>2.5 and pr1[1]<3.2 and pr1[1]>1.1) or (pr1[0]<2.5 and pr1[0]>-0.5 and pr1[1]<4.7 and pr1[1]>3.2) or (pr1[0]<2.5 and pr1[0]>-0.5 and pr1[1]<1.1 and pr1[1]>-0.4): # robot 1 is in region B  
                                        print('robot 1 is in region B')
                                        vx1 = 0.0
                                        vy1 = 0.0
                                        
                                        twist = Twist() # create a twist message
                                        twist.linear.x = vx1*speed1 # add linear x value
                                        twist.linear.y = vy1*speed1 
                                        twist.linear.z = 0.0
                                        twist.angular.x = 0.0   # add angular x value
                                        twist.angular.y = 0.0
                                        twist.angular.z = 0.0
                                        pub.publish(twist) # Publish message to the subscriber
                                    
                                    else:    #robot 1 is in region C

                                        print('robot 1 is in region C')
                                        twist = Twist() # create a twist message
                                        twist.linear.x = vx1*speed1 # add linear x value
                                        twist.linear.y = vy1*speed1 
                                        twist.linear.z = 0.0
                                        twist.angular.x = 0.0   # add angular x value
                                        twist.angular.y = 0.0
                                        twist.angular.z = 0.0
                                        pub.publish(twist) # Publish message to the subscriber

                                else :

                                    #Estimate robot 2's velocity --> AFEGIR!!!

                                    if speed2<tol2: # robot 2 is stopped
                                        print('robot 2 is stopped')
                                        dir=100
                                    else:    # robot 2 moves
                                        cdir=(np.dot(v2,(cr-pr2)))/(LA.norm(v2)*(LA.norm(cr-pr2))); dir=math.acos(cdir)
                                        print(dir*57.2957795131)

                                    if abs(dir)<1.57079632679:   #robot 2 is going to the crosssing
                                        print('robot 2 is going to the crosssing')
                                        
                                        if (pr1[0]<4 and pr1[0]>2.5 and pr1[1]<3.2 and pr1[1]>1.1) or (pr1[0]<2.5 and pr1[0]>-0.5 and pr1[1]<4.7 and pr1[1]>3.2) or (pr1[0]<2.5 and pr1[0]>-0.5 and pr1[1]<1.1 and pr1[1]>-0.4):# robot 1 is in region B
                                            print('robot 1 is in region B: Activate rules')
                                            
                                            if (pr2[0]>4 and pr2[1]<3.2 and pr2[1]>1.1) or (pr2[0]<2.5 and pr2[0]>-0.5 and pr2[1]>4.7) or (pr2[0]<2.5 and pr2[0]>-0.5 and pr2[1]<-0.4): #robot 2 is in region C
                                                print('robot 2 is in region C')
                                                
                                                twist = Twist() # create a twist message
                                                twist.linear.x = vx1*speed1 # add linear x value
                                                twist.linear.y = vy1*speed1 
                                                twist.linear.z = 0.0
                                                twist.angular.x = 0.0   # add angular x value
                                                twist.angular.y = 0.0
                                                twist.angular.z = 0.0
                                                pub.publish(twist) # Publish message to the subscriber
                                            
                                            else: #robot 2 is in region B
                                                print('robot 2 is in region B')

                                                #angular coordinate of robot 1
                                                tb1=math.tan(pr1[1]/pr1[0])
                                                cb1=math.sin(pr1[0]/LA.norm(pr1))
                                                beta1=math.atan(pr1[1]/pr1[0])
                                                if cb1<0:
                                                    beta1=beta1+3.14159265359
                                                if tb1<0 and cb1>0:
                                                    beta1=beta1+6.28318530718

                                                #angular coordinate of robot 2
                                                tb2=math.tan(pr2[1]/pr2[0])
                                                sb2=math.sin(pr2[1]/LA.norm(pr2))
                                                beta2=math.atan(pr2[1]/pr2[0])
                                                if tb2*sb2<0:
                                                    beta2=beta2+3.14159265359
                                                if tb2<0 and sb2>0:
                                                    beta2=beta2+6.28318530718
                                                
                                                print(beta1*57.2957795131,beta2*57.2957795131)
                                                if beta1<beta2:
                                                    speed1 = 0.15  
                                                    twist = Twist() # create a twist message
                                                    twist.linear.x = vx1*speed1 # add linear x value
                                                    twist.linear.y = vy1*speed1 
                                                    twist.linear.z = 0.0
                                                    twist.angular.x = 0.0   # add angular x value
                                                    twist.angular.y = 0.0
                                                    twist.angular.z = 0.0
                                                else:
                                                    vx1 = 0.0
                                                    vy1 = 0.0
                                                    
                                                    twist = Twist() # create a twist message
                                                    twist.linear.x = vx1*speed1 # add linear x value
                                                    twist.linear.y = vy1*speed1 
                                                    twist.linear.z = 0.0
                                                    twist.angular.x = 0.0   # add angular x value
                                                    twist.angular.y = 0.0
                                                    twist.angular.z = 0.0
                                                    pub.publish(twist) # Publish message to the subscriber

                                        else: # robot 1 is in region C
                                            print('robot 1 is in region C: Adapt velocity')

                                            C2_sup=(LA.norm(cr-pr1)-R)/(LA.norm(cr-pr2)+R)
                                            C1_inf=(LA.norm(cr-pr1)+R)/(LA.norm(cr-pr2)-R)
                                            if speed1 >= C2_sup*speed2 and speed1 <= C1_inf*speed2 :    # Robots are on a collision-path
                                                    
                                                    if (LA.norm(cr-pr1)-LA.norm(cr-pr2))>tol:       # robot 2 is closer to the crossing's centre than robot 1
                                                        print('robot 2 is closer to Cr than robot 1')
                                                        speed1=C2_sup*speed2*Ad

                                                        twist = Twist() # create a twist message
                                                        twist.linear.x = vx1*speed1 # add linear x value
                                                        twist.linear.y = vy1*speed1 
                                                        twist.linear.z = 0.0
                                                        twist.angular.x = 0.0   # add angular x value
                                                        twist.angular.y = 0.0
                                                        twist.angular.z = 0.0
                                                        pub.publish(twist) # Publish message to the subscriber

                                                    elif (LA.norm(cr-pr1)-LA.norm(cr-pr2))<tol:                                           # robot 1 is closer to the crossing's centre than robot 2
                                                        print('robot 1 is closer to Cr than robot 2')

                                                        twist = Twist() # create a twist message
                                                        twist.linear.x = vx1*speed1 # add linear x value
                                                        twist.linear.y = vy1*speed1 
                                                        twist.linear.z = 0.0
                                                        twist.angular.x = 0.0   # add angular x value
                                                        twist.angular.y = 0.0
                                                        twist.angular.z = 0.0
                                        
                                            else:       # Robots are not on a collision-path
                                                    print('robots are not on a collision-path')

                                                    twist = Twist() # create a twist message
                                                    twist.linear.x = vx1*speed1 # add linear x value
                                                    twist.linear.y = vy1*speed1 
                                                    twist.linear.z = 0.0
                                                    twist.angular.x = 0.0   # add angular x value
                                                    twist.angular.y = 0.0
                                                    twist.angular.z = 0.0


                                    else:   #robot 2 is not going to the crosssing
                                        print('robot 2 is not going to the crosssing')
                                        
                                        speed1 = 0.15
                                        twist = Twist() # create a twist message
                                        twist.linear.x = vx1*speed1 # add linear x value
                                        twist.linear.y = vy1*speed1
                                        twist.linear.z = 0.0
                                        twist.angular.x = 0.0   # add angular x value
                                        twist.angular.y = 0.0
                                        twist.angular.z = 0.0
                                        pub.publish(twist) # Publish message to the subscriber

                            else: #Waiting
                                twist = Twist() # create a twist message
                                twist.linear.x = vx1*speed1 # add linear x value
                                twist.linear.y = vy1*speed1 
                                twist.linear.z = 0.0
                                twist.angular.x = 0.0   # add angular x value
                                twist.angular.y = 0.0
                                twist.angular.z = 0.0
                                pub.publish(twist) # Publish message to the subscriber
                                
                            m=m+1

                        else:
                                
                            #No collisions detected
                            twist = Twist() # create a twist message
                            twist.linear.x = vx1*speed1 # add linear x value
                            twist.linear.y = vy1*speed1 
                            twist.linear.z = 0.0
                            twist.angular.x = 0.0   # add angular x value
                            twist.angular.y = 0.0
                            twist.angular.z = 0.0
                            pub.publish(twist) # Publish message to the subscriber

                        scan=False

                    if (p%15) == 0:
                        print("real position:", pr1)
                        print('yaw', yaw1[0] )
                        print('robot 1 linear velocity:', dv1_o*speed1)
                        
                    p=p+1

                    if LA.norm(pd-pr1)< tol:
                        print("Goal achieved")
                        
                        vx1 = 0.0
                        vy1 = 0.0
                        
                        twist = Twist() # create a twist message
                        twist.linear.x = vx1*speed1 # add linear x value
                        twist.linear.y = vy1*speed1 
                        twist.linear.z = 0.0
                        twist.angular.x = 0.0   # add angular x value
                        twist.angular.y = 0.0
                        twist.angular.z = 0.0
                        pub.publish(twist) # Publish message to the subscriber

                        break

          
            elif kk ==1: # Collision detection deactivated + turning
                turn=0.1
                speed1=0.15
                while(1):
                    rclpy.spin_once(node,executor=None, timeout_sec=1)
                    
                    dv1_o=(pd-pr1)/LA.norm(pd-pr1) # unitary vector that gives the direction and sense of the linear velocity in the odom frame
                    rotz[0,0]=math.cos(yaw1[0]); rotz[0,1]=math.sin(yaw1[0])
                    rotz[1,0]=-math.sin(yaw1[0]); rotz[1,1]=math.cos(yaw1[0])
                    dv1=rotz.dot(dv1_o) # Change the velocity comands to the base-footprint frame

                    vx1=float(dv1[0])
                    vy1=float(dv1[1])

                    if abs(yaw1[0]) > tol:
                        if yaw1[0]<0:
                            wz1=1
                        if yaw1[0]>0:
                            wz1=-1
                    elif abs(yaw1[0]) < tol:
                        wz1=0

                    twist = Twist() # create a twist message
                    twist.linear.x = vx1*speed1 # add linear x value
                    twist.linear.y = vy1*speed1 
                    twist.linear.z = 0.0
                    twist.angular.x = 0.0   # add angular x value
                    twist.angular.y = 0.0
                    twist.angular.z = wz1*turn
                    pub.publish(twist) # Publish message to the subscriber

                    if (p%15) == 0:
                        print("real position:", pr1)
                        print('yaw', yaw1[0] )
                        print('robot 1 linear velocity:', dv1_o*speed1)
                    p=p+1

                    if LA.norm(pd-pr1)< tol:
                        print("Goal achieved")
                        
                        vx1 = 0.0
                        vy1 = 0.0
                            
                        twist = Twist() # create a twist message
                        twist.linear.x = vx1*speed1 # add linear x value
                        twist.linear.y = vy1*speed1 
                        twist.linear.z = 0.0
                        twist.angular.x = 0.0   # add angular x value
                        twist.angular.y = 0.0
                        twist.angular.z = 0.0
                        pub.publish(twist) # Publish message to the subscriber

                        break

            else : # Collision detection deactivated
                turn=0
                speed1=0.15
                while(1):
                    rclpy.spin_once(node,executor=None, timeout_sec=1)

                    dv1_o=(pd-pr1)/LA.norm(pd-pr1) #  unitary vector that gives the direction and sense of the linear velocity in the odom frame
                    rotz[0,0]=math.cos(yaw1[0]); rotz[0,1]=math.sin(yaw1[0])
                    rotz[1,0]=-math.sin(yaw1[0]); rotz[1,1]=math.cos(yaw1[0])
                    dv1=rotz.dot(dv1_o) # Change the velocity comands to the base-footprint frame

                    vx1=float(dv1[0])
                    vy1=float(dv1[1])

                    twist = Twist() # create a twist message
                    twist.linear.x = vx1*speed1 # add linear x value
                    twist.linear.y = vy1*speed1 
                    twist.linear.z = 0.0
                    twist.angular.x = 0.0   # add angular x value
                    twist.angular.y = 0.0
                    twist.angular.z = 0.0
                    pub.publish(twist) # Publish message to the subscriber

                    if (p%15) == 0:
                        print("real position:", pr1)
                        print('yaw', yaw1[0] )
                        print('robot 1 linear velocity:', dv1_o*speed1)
                    p=p+1

                    if LA.norm(pd-pr1)< tol:
                        print("Goal achieved")
                            
                        vx1 = 0.0
                        vy1 = 0.0
                            
                        twist = Twist() # create a twist message
                        twist.linear.x = vx1*speed1 # add linear x value
                        twist.linear.y = vy1*speed1 
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
        vx1 = 0.0
        vy1 = 0.0
        speed1 = 0.2

        twist = Twist() # create a twist message
        twist.linear.x = vx1*speed1 # add linear x value
        twist.linear.y = vy1*speed1 
        twist.linear.z = 0.0
        twist.angular.x = 0.0   # add angular x value
        twist.angular.y = 0.0
        twist.angular.z = 0.0
        pub.publish(twist) # Publish message to the subscriber