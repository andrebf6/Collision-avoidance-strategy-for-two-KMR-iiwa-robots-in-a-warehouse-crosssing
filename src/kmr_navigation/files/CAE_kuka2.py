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
Robot KUKA2 following a desired trajectory in a warehouse crossing while avoiding KUKA1.

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

    pr2[0]=xr; pr2[1]=yr; yaw2[0]= yawc

def listener_callback1(scan_data):
    read=scan_data.ranges
    readings[:]=read[:]
    global scan
    scan= True
    
    time_sec=scan_data.header.stamp.sec
    time_nan=scan_data.header.stamp.nanosec; time_nan=time_nan*1e-9
    global t
    t=time_sec+time_nan

if __name__=="__main__":# Proteger parte del código. Assegura que el bloque solo se ejecutará si tu módulo es el programa principal.
    rclpy.init() #initialize
    # Create node : KUKA_travels
    node = rclpy.create_node('KUKA2_travels')

    # The node is subscribed to the topic /odom, and recives messages of the type Pose and Twist, with a "queue size" of 10 queued messages
    sub = node.create_subscription(Odometry, '/robot2/odom',listener_callback, 10)
    # The node is subscribed to the topic /scan, and recives messages of the type Laserscan, with the quality of service specified by the publisher (GAZEBO plugin)
    sub2 = node.create_subscription(LaserScan, '/robot2/scan',listener_callback1, rclpy.qos.qos_profile_sensor_data)

    # The node publishes messages of the type Twist, to the topic /cmd_vel, with a "queue size" of 10 queued messages
    pub = node.create_publisher(Twist,'/robot2/cmd_vel', 10)

    #Initialization values for driving robot 2:
    vx2 = 0.0                                       # Unitary vector for the desired linear velocity (x component) 
    vy2 = 0.0                                       # Unitary vector for the desired linear velocity (y component)
    speed2 = 0.15                                    # Modulus of the desired linear velocity (m/s)
    dv2_o=np.zeros(2)*2000                          #Initialization value for the unitary vector for the desired linear velocity in theodom frame (used only in the first printing-p=0)

    pr2=np.ones(2)*2000                             # Position of the robot 2's centre of mass in the odom frame  (m)
    dl=np.ones(2); dl[0]=0.483; dl[1]=-0.258        # Laser B1 position in the base-footprint frame (m)
    yaw2=np.ones(1)*2000                            # Orientation of the robot in the 2D plane (rad) 
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
    pb=np.ones(2);pb[0]=2.1; pb[1]=3.2             # Points that limit the vision cone
    pc=np.ones(2);pc[0]=-0.5; pc[1]=3.2

    #Compute maximum collision-danger distance
    pa=np.ones(2);pa[0]=2.5; pa[1]=1.1
    x_lim1=-0.4                                       #Condition for the linear constrain                                   
    x_lim2=2.4
    y_lim=1.3

    #Position estimate for robot 1:
    tol4=5*1e-3                                     # Tolerance for robot 2's x-component of the velocity
    tol3=0.09                                       # Tolerance for robot 1's velocity direction
    R=0.626                                         # Robot's radius
    cr=np.ones(2);cr[0]=0.8; cr[1]=2.1              # Crossing's centre
    
    #Velocity estimate for robot 1:
    global t
    t=2000                                          # Initialization value of the time at which the last sensor reading has been finished (in the Gazebo time frame)
    inct=1                                          # Aproximate time interval to compute the velocity
    MS=int(inct*scan_freq)                          # Samples distance to compute the velocity
    opr1=np.ones(2)*2000                            # Initialization of the array to store old position values
    SF=2                                            # Simulation time factor
    stop=False
    
    #Collision avoidance                                          
    tol2=1e-4                                       # Tolerance for robot 1's modulus of the velocity
    Ad=0.5                                          # Adaptation coeficient for the linear velocity modulus

    # Stop the robot at the begginging of the simulation
    twist = Twist() # create a twist message
    twist.linear.x = vx2*speed2 # add linear x value
    twist.linear.y = vy2*speed2
    twist.linear.z = 0.0
    twist.angular.x = 0.0   # add angular x value
    twist.angular.y = 0.0
    twist.angular.z = 0.0
    pub.publish(twist) # Publish message to the subscriber

    try: # The try block lets you test a block of code for errors.
        print(msg) # Initial message

        #Define trajectory for robot2
        nw=3
        waypoints=np.ones((nw,2))
        waypoints[0,0]=0.8; waypoints[0,1]=3.4482
        waypoints[1,0]=0.8; waypoints[1,1]=1.1
        waypoints[2,0]=0.8; waypoints[2,1]=-1

        for kk in range(nw):
            pd=waypoints[kk,:]
            print("desired position:", pd)

            p=0.0
            m=0.0
            if kk ==0:              # Collision detection activated
                while(1):
                    rclpy.spin_once(node,executor=None, timeout_sec=1)

                    if pr2[0]!=2000 and pr2[1]!=2000 and scan==True: # Position information has been published at least once and a new sensor reading has entred.

                        #Compute unitary vector that gives the direction and sense of the desired linear velocity to drive robot1 to pd (in the odom frame)
                        dv2_o=(pd-pr2)/LA.norm(pd-pr2)                                  # unitary linear velocity vector (cartesian coordinates)
                        tphi=math.tan(dv2_o[1]/dv2_o[0])                                # unitary linear velocity vector angular component (polar coordinates)     
                        cphi=math.sin(dv2_o[0]/LA.norm(dv2_o))
                        phi=math.atan(dv2_o[1]/dv2_o[0])
                        if cphi<0:
                            phi=phi+3.14159265359
                        if tphi<0 and cphi>0:
                            phi=phi+6.28318530718  

                        rotz[0,0]=math.cos(yaw2[0]); rotz[0,1]=math.sin(yaw2[0])        # Build rotation matrix (to change from the odom to the base-footprint frame)        
                        rotz[1,0]=-math.sin(yaw2[0]); rotz[1,1]=math.cos(yaw2[0])
                        
                        dv2=rotz.dot(dv2_o)                                             # Change the velocity comands to the base-footprint frame
                        vx2=float(dv2[0])
                        vy2=float(dv2[1])

                        #Tuncate scanner readings
                        inv_rotz=np.transpose(rotz)                                     # Build rotation matrix (to change from the base-footprint to the odom frame)
                        pl2=pr2+inv_rotz.dot(dl)                                        # Determine laser B1 position in the odom frame 
                                
                        lb2=pb-pl2                                                      # Build cone of vision in the odom frame
                        lc2=pc-pl2
                        calpha3=(np.dot(lb2,dv2_o))/(LA.norm(lb2)*(LA.norm(dv2_o))); alpha3=math.acos(calpha3) 
                        calpha4=(np.dot(lc2,dv2_o))/(LA.norm(lc2)*(LA.norm(dv2_o))); alpha4=math.acos(calpha4)

                        blalpha3=phi+alpha3-yaw2[0]-theta;                              # Pass bounding angles from the odom to the laser-link frame
                        tt=blalpha3//6.28318530718; blalpha3=blalpha3-tt*6.28318530718; 
                        if blalpha3 > 3.14159265359:
                            blalpha3=blalpha3-6.28318530718
                        if blalpha3 < -3.14159265359:
                            blalpha3=blalpha3+6.28318530718

                        blalpha4=phi-alpha4-yaw2[0]-theta; 
                        uu=blalpha4//6.28318530718; blalpha4=blalpha4-uu*6.28318530718; 
                        if blalpha4 > 3.14159265359:
                            blalpha4=blalpha4-6.28318530718
                        if blalpha4 < -3.14159265359:
                            blalpha4=blalpha4+6.28318530718

                        get_laser_index = lambda angle, angle_min, freq: int((angle-angle_min)/freq)+ int((angle-angle_min)%freq > (freq/2))
                        i1= get_laser_index(blalpha3, amin, ainc) 
                        i2=  get_laser_index(blalpha4, amin, ainc) 
                        i_readings=readings[i2:i1]
                        i_angles=angles[i2:i1]

                        #Compute collision-dangerous distance for each reading direction:
                        oi_angles=i_angles+theta+yaw2[0]
                        uu=oi_angles//6.28318530718; oi_angles=oi_angles-uu*6.28318530718; 
                        if np.any(oi_angles <0):
                            ig=np.where(oi_angles <0)
                            oi_angles[ig]=oi_angles[ig]+6.28318530718

                        delt4=alpha4
                        cdelt5=(np.dot(pa-pl2,dv2_o))/(LA.norm(pa-pl2)*(LA.norm(dv2_o))); delt5=math.acos(cdelt5)
                        cdelt6=(np.dot(pa-pl2,pb-pl2))/(LA.norm(pa-pl2)*(LA.norm(pb-pl2))); delt6=math.acos(cdelt6)
                        
                        id4=np.where((phi-delt4 <= oi_angles) & (oi_angles<phi))
                        if len(id4[0]) != 0:
                            if id4[0][0]!=0:
                                id4=np.insert(id4,0,0)
                        id5=np.where((phi<= oi_angles) & (oi_angles < phi+delt5))
                        id6=np.where((phi+delt5<= oi_angles) & (oi_angles <= phi+delt5+delt6))
                        if len(id6[0]) != 0:
                            if id6[0][-1]!=(len(oi_angles)-1):
                                vv=np.arange(id6[0][-1]+1,len(oi_angles))
                                id6=np.concatenate((id6[0],vv))

                        oi_angles_d4=oi_angles[id4]
                        oi_angles_d5=oi_angles[id5]
                        oi_angles_d6=oi_angles[id6]
                        
                        cdd_d4=(x_lim1-pl2[0])/np.cos(oi_angles_d4); cdd_d4=abs(cdd_d4)
                        cdd_d5=(x_lim2-pl2[0])/np.cos(oi_angles_d5); cdd_d5=abs(cdd_d5)
                        cdd_d6=(y_lim-pl2[1])/np.sin(oi_angles_d6)
                        cdd=np.concatenate((cdd_d4,cdd_d5,cdd_d6))

                        #Detect possible collsision
                        if np.any(i_readings-cdd < 0)==True:

                            #Robot detected: Collision avoidance activated
                            id=np.where(i_readings-cdd< 0); dread=i_readings[id];dang=i_angles[id]; dong=oi_angles[id]

                            if m==0: #Initialization
                                print('Robot detected at time t=',t)
                               
                                #Estimate robot 1's position in the odom frame
                                dc= np.amin(dread); ic=np.where(dread==dc); ac=dang[ic]; oc=dong[ic]
                                blop=np.ones(2)
                                blop[0]=(R+dc)*np.cos(ac);blop[1]=(R+dc)*np.sin(ac) #Cartesian coordinates of the detected objects (in laser-link frame)
                                fop=inv_rotzl.dot(blop) # Base-footprint frame
                                op=inv_rotz.dot(fop)    # Odom frame
                                pr1=op+pl2 
                                res=oc-phi

                                if abs(dv2_o[0]*speed2)>tol4:
                                    #robot 2 is moving in the x axis
                                    if abs(res)>tol3:
                                        #robot 1 is moving in the y axis
                                        pr1[0]=cr[0]
                                    else:
                                        #robot 1 is moving in the x axis
                                        pr1[1]=cr[1] 
                                else:
                                    #robot 2 is moving in the y axis
                                    if abs(res)>tol3:
                                        #robot 1 is moving in the x axis
                                        pr1[1]=cr[1]
                                    else:
                                        #robot 1 is moving in the y axis
                                        pr1[0]=cr[0]  

                                print(pr1,t)
                                                                
                                opr1[0]=pr1[0];opr1[1]=pr1[1]
                                t_old=t

                                twist = Twist() # create a twist message
                                twist.linear.x = vx2*speed2 # add linear x value
                                twist.linear.y = vy2*speed2 
                                twist.linear.z = 0.0
                                twist.angular.x = 0.0   # add angular x value
                                twist.angular.y = 0.0
                                twist.angular.z = 0.0
                                pub.publish(twist) # Publish message to the subscriber

                            elif (m!=0 and (m%MS) == 0):
                                                                
                                #Estimate robot 1's position in the odom frame
                                dc= np.amin(dread); ic=np.where(dread==dc); ac=dang[ic]; oc=dong[ic]
                                blop=np.ones(2)
                                blop[0]=(R+dc)*np.cos(ac);blop[1]=(R+dc)*np.sin(ac) #Cartesian coordinates of the detected objects (in laser-link frame)
                                fop=inv_rotzl.dot(blop) # Base-footprint frame
                                op=inv_rotz.dot(fop)    # Odom frame
                                pr1=op+pl2
                                res=oc-phi

                                print(dv2_o[0]*speed2,res*57.2957795)
                                if abs(dv2_o[0]*speed2)>tol4:
                                    #robot 2 is moving in the x axis
                                    if abs(res)>tol3:
                                        #robot 1 is moving in the y axis
                                        pr1[0]=cr[0]
                                    else:
                                        #robot 1 is moving in the x axis')
                                        pr1[1]=cr[1] 
                                else:
                                    #robot 2 is moving in the y axis')
                                    if abs(res)>tol3:
                                       #robot 1 is moving in the x axis
                                        pr1[1]=cr[1]
                                    else:
                                        #robot 1 is moving in the y axis
                                        pr1[0]=cr[0]   

                                if (m%(2*MS)) == 0:print(pr1,t)

                                #Estimate robot 1's velocity in the odom frame
                                v1=SF*(pr1-opr1)/(t-t_old)
                                speed1=LA.norm(v1)
                                if (m%(2*MS)) == 0:print(v1,speed1)
                            
                                opr1[0]=pr1[0];opr1[1]=pr1[1]
                                t_old=t

                                #Collision avoidance algorithm
                                if pr1[0]<2.5 and pr1[0]>-0.5 and pr1[1]>1.1 and pr1[1]<3.2: # robot 1 is in region A
                                    print('robot 1 is in region A')
                                    
                                    if (pr2[0]<4 and pr2[0]>2.5 and pr2[1]<3.2 and pr2[1]>1.1) or (pr2[0]<2.5 and pr2[0]>-0.5 and pr2[1]<4.7 and pr2[1]>3.2) or (pr2[0]<2.5 and pr2[0]>-0.5 and pr2[1]<1.1 and pr2[1]>-0.4): # robot 2 is in region B  
                                        print('robot 2 is in region B')
                                        vx2 = 0.0
                                        vy2 = 0.0
                                        stop=True
                                        
                                        twist = Twist() # create a twist message
                                        twist.linear.x = vx2*speed2 # add linear x value
                                        twist.linear.y = vy2*speed2 
                                        twist.linear.z = 0.0
                                        twist.angular.x = 0.0   # add angular x value
                                        twist.angular.y = 0.0
                                        twist.angular.z = 0.0
                                        pub.publish(twist) # Publish message to the subscriber
                                    
                                    else:    #robot 1 is in region C

                                        print('robot 2 is in region C')
                                        speed2 = 0.15
                                        stop=False

                                        twist = Twist() # create a twist message
                                        twist.linear.x = vx2*speed2 # add linear x value
                                        twist.linear.y = vy2*speed2 
                                        twist.linear.z = 0.0
                                        twist.angular.x = 0.0   # add angular x value
                                        twist.angular.y = 0.0
                                        twist.angular.z = 0.0
                                        pub.publish(twist) # Publish message to the subscriber

                                else :

                                    if speed1<tol2: # robot 1 is stopped
                                        print('robot 1 is stopped')
                                        dir=100
                                    else:    # robot 2 moves
                                        cdir=(np.dot(v1,(cr-pr1)))/(LA.norm(v1)*(LA.norm(cr-pr1))); dir=math.acos(cdir)

                                    if abs(dir)<1.57079632679:   #robot 1 is going to the crosssing
                                        print('robot 1 is going to the crosssing')
                                        
                                        if (pr2[0]<4 and pr2[0]>2.5 and pr2[1]<3.2 and pr2[1]>1.1) or (pr2[0]<2.5 and pr2[0]>-0.5 and pr2[1]<4.7 and pr2[1]>3.2) or (pr2[0]<2.5 and pr2[0]>-0.5 and pr2[1]<1.1 and pr2[1]>-0.4): # robot 2 is in region B 
                                            print('robot 2 is in region B: Activate rules')
                                            
                                            if (pr1[0]>4 and pr1[1]<3.2 and pr1[1]>1.1) or (pr1[0]<2.5 and pr1[0]>-0.5 and pr1[1]>4.7) or (pr1[0]<2.5 and pr1[0]>-0.5 and pr1[1]<-0.4): #robot 1 is in region C
                                                print('robot 1 is in region C')
                                                stop=False
                                                
                                                twist = Twist() # create a twist message
                                                twist.linear.x = vx2*speed2 # add linear x value
                                                twist.linear.y = vy2*speed2 
                                                twist.linear.z = 0.0
                                                twist.angular.x = 0.0   # add angular x value
                                                twist.angular.y = 0.0
                                                twist.angular.z = 0.0
                                                pub.publish(twist) # Publish message to the subscriber
                                            
                                            else: #robot 1 is in region B
                                                print('robot 1 is in region B')

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
                                                
                                                if (m%(2*MS)) == 0:print(beta1*57.2957795131,beta2*57.2957795131)
                                                if beta2<beta1:
                                                    speed2 = 0.15
                                                    stop=False

                                                    twist = Twist() # create a twist message
                                                    twist.linear.x = vx2*speed2 # add linear x value
                                                    twist.linear.y = vy2*speed2 
                                                    twist.linear.z = 0.0
                                                    twist.angular.x = 0.0   # add angular x value
                                                    twist.angular.y = 0.0
                                                    twist.angular.z = 0.0
                                                else:
                                                    vx2 = 0.0
                                                    vy2 = 0.0
                                                    stop=True
                                                    
                                                    twist = Twist() # create a twist message
                                                    twist.linear.x = vx2*speed2 # add linear x value
                                                    twist.linear.y = vy2*speed2 
                                                    twist.linear.z = 0.0
                                                    twist.angular.x = 0.0   # add angular x value
                                                    twist.angular.y = 0.0
                                                    twist.angular.z = 0.0
                                                    pub.publish(twist) # Publish message to the subscriber

                                        else: # robot 2 is in region C
                                            print('robot 2 is in region C: Adapt velocity')

                                            C1_sup=(LA.norm(cr-pr2)-R)/(LA.norm(cr-pr1)+R)
                                            C2_inf=(LA.norm(cr-pr2)+R)/(LA.norm(cr-pr1)-R)
                                            if speed2 >= C1_sup*speed1 and speed2 <= C2_inf*speed1 :    # Robots are on a collision-path
                                                    
                                                    if (LA.norm(cr-pr2)-LA.norm(cr-pr1))>tol:       # robot 1 is closer to the crossing's centre than robot 2
                                                        print('robot 1 is closer to Cr than robot 2')
                                                        speed2=C1_sup*speed1*Ad
                                                        stop=False

                                                        twist = Twist() # create a twist message
                                                        twist.linear.x = vx2*speed2 # add linear x value
                                                        twist.linear.y = vy2*speed2 
                                                        twist.linear.z = 0.0
                                                        twist.angular.x = 0.0   # add angular x value
                                                        twist.angular.y = 0.0
                                                        twist.angular.z = 0.0
                                                        pub.publish(twist) # Publish message to the subscriber

                                                    elif (LA.norm(cr-pr2)-LA.norm(cr-pr1))<tol:                                           # robot 1 is closer to the crossing's centre than robot 2
                                                        print('robot 2 is closer to Cr than robot 1')
                                                        stop=False

                                                        twist = Twist() # create a twist message
                                                        twist.linear.x = vx2*speed2 # add linear x value
                                                        twist.linear.y = vy2*speed2 
                                                        twist.linear.z = 0.0
                                                        twist.angular.x = 0.0   # add angular x value
                                                        twist.angular.y = 0.0
                                                        twist.angular.z = 0.0
                                        
                                            else:       # Robots are not on a collision-path
                                                    print('robots are not on a collision-path')
                                                    stop=False

                                                    twist = Twist() # create a twist message
                                                    twist.linear.x = vx2*speed2 # add linear x value
                                                    twist.linear.y = vy2*speed2 
                                                    twist.linear.z = 0.0
                                                    twist.angular.x = 0.0   # add angular x value
                                                    twist.angular.y = 0.0
                                                    twist.angular.z = 0.0


                                    else:   #robot 1 is not going to the crosssing
                                        print('robot 1 is not going to the crosssing')
                                        speed2 = 0.15
                                        stop=False

                                        twist = Twist() # create a twist message
                                        twist.linear.x = vx2*speed2 # add linear x value
                                        twist.linear.y = vy2*speed2
                                        twist.linear.z = 0.0
                                        twist.angular.x = 0.0   # add angular x value
                                        twist.angular.y = 0.0
                                        twist.angular.z = 0.0
                                        pub.publish(twist) # Publish message to the subscriber


                            else: # Waiting

                                if stop== True:
                                    vx2 = 0.0
                                    vy2 = 0.0

                                twist = Twist() # create a twist message
                                twist.linear.x = vx2*speed2 # add linear x value
                                twist.linear.y = vy2*speed2 
                                twist.linear.z = 0.0
                                twist.angular.x = 0.0   # add angular x value
                                twist.angular.y = 0.0
                                twist.angular.z = 0.0
                                pub.publish(twist) # Publish message to the subscriber
                                
                            m=m+1

                        else:
                                
                            #No collisions detected
                            twist = Twist() # create a twist message
                            twist.linear.x = vx2*speed2 # add linear x value
                            twist.linear.y = vy2*speed2 
                            twist.linear.z = 0.0
                            twist.angular.x = 0.0   # add angular x value
                            twist.angular.y = 0.0
                            twist.angular.z = 0.0
                            pub.publish(twist) # Publish message to the subscriber

                        scan=False

                    #if (p%15) == 0:
                        #print("real position:", pr2)
                        #print('yaw', yaw2[0] )
                        #print('robot 2 linear velocity:', dv2_o*speed2)
                        
                    p=p+1

                    if LA.norm(pd-pr2)< tol:
                        print("Goal achieved")
                        
                        vx2 = 0.0
                        vy2 = 0.0
                        
                        twist = Twist() # create a twist message
                        twist.linear.x = vx2*speed2 # add linear x value
                        twist.linear.y = vy2*speed2 
                        twist.linear.z = 0.0
                        twist.angular.x = 0.0   # add angular x value
                        twist.angular.y = 0.0
                        twist.angular.z = 0.0
                        pub.publish(twist) # Publish message to the subscriber

                        break

           
            else : # Collision detection deactivated
                speed2=0.15
                while(1):
                    rclpy.spin_once(node,executor=None, timeout_sec=1)

                    dv2_o=(pd-pr2)/LA.norm(pd-pr2) # unitary vector that gives the direction in the x-y plain
                    rotz[0,0]=math.cos(yaw2[0]); rotz[0,1]=math.sin(yaw2[0])
                    rotz[1,0]=-math.sin(yaw2[0]); rotz[1,1]=math.cos(yaw2[0])
                    dv2=rotz.dot(dv2_o) # Change the velocity comands to the base-footprint frame

                    vx2=float(dv2[0])
                    vy2=float(dv2[1])

                    twist = Twist() # create a twist message
                    twist.linear.x = vx2*speed2 # add linear x value
                    twist.linear.y = vy2*speed2 
                    twist.linear.z = 0.0
                    twist.angular.x = 0.0   # add angular x value
                    twist.angular.y = 0.0
                    twist.angular.z = 0.0
                    pub.publish(twist) # Publish message to the subscriber

                    #if (p%15) == 0:
                        #print("real position:", pr2)
                        #print('yaw', yaw2[0] )
                        #print('robot 2 linear velocity:', dv2_o*speed2)
                    p=p+1

                    if LA.norm(pd-pr2)< tol:
                        print("Goal achieved")

                        speed2 = 0.2
                        vx2 = 0.0
                        vy2 = 0.0
                            
                        twist = Twist() # create a twist message
                        twist.linear.x = vx2*speed2 # add linear x value
                        twist.linear.y = vy2*speed2 
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
        vx2 = 0.0
        vy2 = 0.0
        speed2 = 0.15

        twist = Twist() # create a twist message
        twist.linear.x = vx2*speed2 # add linear x value
        twist.linear.y = vy2*speed2 
        twist.linear.z = 0.0
        twist.angular.x = 0.0   # add angular x value
        twist.angular.y = 0.0
        twist.angular.z = 0.0
        pub.publish(twist) # Publish message to the subscriber