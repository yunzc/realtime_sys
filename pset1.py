# -*- coding: utf-8 -*-
"""
Created on Thu Feb  9 03:26:56 2017

@author: Yun Chang 
16.35 Real Time System and Software 
Problem Set 1
"""
import math 
import numpy as np 
class Control(object):
    """
    two qunatites: forward speed and rotational speed
    """
    def __init__(self, s, omega):
        """
        s is the forward speed (m/s)
        omega is the rotational velocity (rad/s)
        """
        self.s = s
        self.omega = omega 
    
    def getSpeed(self):
        """
        return the stored value of the forward speed s
        """
        return self.s
    
    def getRotVel(self):
        """
        return the stored value of the rotational velocity omega 
        """
        return self.omega 
    
class GroundVehicle(object):
    """
    contains current pose and derivative of pose of the vehicle 
    constraint: 
        0 <= x <= 100
        0 <= y <= 100
        -math.pi <= theta <= math.pi 
        
    """
    def __init__(self, pose, s, omega):
        """
        pose is array [x, y, theta]
        s is the forward speed (m/s)
        omega is the rotational velocity (rad/s)
        """
        self.pose = pose
        x_dot = math.cos(pose[-1])*s #speed times the cosine of angle 
        y_dot = math.sin(pose[-1])*s
        self.velocity = np.array([x_dot, y_dot, omega])
        self.s = s
        self.omega = omega
    
    def getPosition(self):
        """
        return and array of 3 floating point values that indicate x,y,theta
        """
        return self.pose
    
    def getVelocity(self):
        """
        return array of 3 values corresponding to the derivative of the pose
        """
        return self.velocity
    
    def setPosition(self, pose):
        """
        takes array of 3 values and set the corresponding internal representation 
        if attempt to exceed constraint, will be clamped at the limit closest 
        to desired 
        """
        for i in range(2): #check if x, y value is out or constrained range
            if pose[i] < 0:
                pose[i] = 0
            elif pose[i] > 100:
                pose[i] = 100
        self.pose = pose #update stored value 
        
    def setVelocity(self, velocity):
        """
        takes 3 values and sets internal representation 
        If the setVelocity method attempts to exceed the velocity constraints, the 
        position will be clamped at the nearest limit, where in the linear 
        velocities, the “nearest” velocity is the closest velocity in Euclidean 
        distance in velocity space.
        """
        x_dot = velocity[0]
        y_dot = velocity[1]
        #check constraint 
        if math.sqrt(x_dot**2 + y_dot**2) < 5:
            ang = math.atan2(y_dot, x_dot) #retain the angle and adjust speed (magnitude)
            velocity[0] = 5*math.cos(ang)
            velocity[1] = 5*math.sin(ang)
        elif math.sqrt(x_dot**2 + y_dot**2) > 10:
            ang = math.atan2(y_dot, x_dot) 
            velocity[0] = 5*math.cos(ang)
            velocity[1] = 5*math.sin(ang)
        self.velocity = velocity 
    
    def controlVehicle(self, c):
        """
        c is a Control instance. Modifies internal velocities accordinig to 
        specified forward speed and angular velocity
        """
        orien = self.pose[-1] #get current vehicle orientation 
        speed = c.getSpeed()
        rotSpeed = c.getRotVel()
#        print("speed: ",speed)
#        print("rot vel: ",rotSpeed)
        x_dot = speed*math.cos(orien) #calculate the velocity from speed 
        y_dot = speed*math.sin(orien)
        self.setVelocity([x_dot, y_dot, rotSpeed])
    
    def updateState(self, sec, msec):
        """
        changes the vehicles state by computing the change that will occur in time t
        t is given in sec and msec
        """
        time = sec + msec/1000
        veloc = self.getVelocity()
        posit = self.getPosition()
        #update position 
        newpose = []
        for i in range(3):
            newpose.append(posit[i] + veloc[i]*time)
        newpose[-1] = newpose[-1] % (2*math.pi)
        #update velocity 
        newveloc = []
        speed = math.sqrt(veloc[0]**2 + veloc[1]**2)
        thet = newpose[-1]
        newveloc.append(speed*math.cos(thet))#xdot
        newveloc.append(speed*math.sin(thet))#ydot 
        newveloc.append(veloc[-1])
        #update internal state 
        self.setPosition(newpose)
        self.setVelocity(newveloc)

class Simulator(object):
    
    def __init__(self):
        #starts time at 0 before run()
        self.clockSec = 0
        self.clockMSec = 0
        #Construc a Ground vehicle 
        self.vehicle = GroundVehicle([0,0,0],0,0)
        #trajectory polygon sides. Default is 5 
        self.polygonSides = 5
    
    def getCurrentSec(self):
        """
        Returns the seconds component of the current Simulator clock. 
        """
        return self.clockSec
    
    def getCurrentMSec(self):
        """
        returns the milli-seconds component of the current simulator clock 
        """
        return self.clockMSec
    
    def getControl(self, sec, msec):
        """
        returns a control if one should be issued at the time(sec,msec)
        if no control should be issued at the time, return None
        The control should help the vehicle trace out an n sided polygon
        Vehicle will start forward and turn n - 1 times 
        Each time the angle to be turned is 360/n
        Robot will go to (50,0) first before starting to trace the polygon 
        """
        incTime = 10
        n = self.polygonSides
        radius = 25 #radius of the circle circumscribing the polygon 
        #define how to turn corner: partition to two parts, first turn half
        #the totally needed turn angle, then turn the other half
        totalTime = int(2000/n)*n*10 #in milisecs 
        timePerSide = int(2000/n)*10 #in milisecs; want each interval to be 10
        #use law of cosines to get side length 
        extAng = math.radians(360/n) #polygon external angle (turn angle) in radians 
        sidelength = radius*math.sqrt(2 - 2*math.cos(extAng))
        cruise_speed = sidelength/(timePerSide/1000) #speed when on straigh line (m/s)
        turn_speed = cruise_speed*math.cos(extAng/2)/math.cos((extAng/6)) #speed on the turn segs
        #implement a four segment, three point turn 
        #first segment, still on start part but angling for next segment 
        control = None 
        if sec < 10: #give vehicle 10 secs to get to (50,0)
            if sec == 0 and msec == 0:
                control = Control(5,0) #just getting to (50,0)
        elif (sec*1000+msec) > totalTime + 10*1000: 
            control = None #only want to trace polygon once 
        else: 
            if ((sec-10)*1000+msec) == totalTime + 10*1000: 
                #stop when the polygon has been traced out by vehicle 
                control = Control(0,0)
            elif ((sec-10)*1000+msec) % timePerSide == timePerSide - 2*incTime:
                control = Control(cruise_speed, extAng/3/(incTime/1000))
            #second and third segment at turn speed angling for next turn 
            elif ((sec-10)*1000+msec) % timePerSide == timePerSide - incTime:
                control = Control(turn_speed, extAng/3/(incTime/1000))
            #fourth segment back to straight part 
            elif ((sec-10)*1000+msec) % timePerSide == incTime:
                control = Control(cruise_speed, 0)

        return control 
    
    def setNumSides(self, n):
        """
        set the number of sides of the polygon the vehicle is tracing out 
        """
        if n >=3 and n <= 10:
            self.polygonSides = n
    
    def run(self):
        """
        set clock to zero, while total time is less than 100 secs, get control, 
        apply control, update state, print current time, position, and orientation 
        then increment clock by 10 millisecs 
        """
        f = open('data.txt','w')
        incTime = 10
        self.clockSec = 0
        self.clockMSec = 0
        while self.clockSec < 100:
            control = self.getControl(self.clockSec, self.clockMSec) #get control 
            if control != None: 
                self.vehicle.controlVehicle(control) #apply control 
            self.vehicle.updateState(0, 10)
            time = round(self.clockSec + self.clockMSec/1000,2)
            pose = self.vehicle.getPosition()
            print(time," ",round(pose[0],2)," ",round(pose[1],2)," ",round(pose[2],1))
            f.write(str(time)+" "+str(round(pose[0],2))+" "+str(round(pose[1],2))+" "+str(round(pose[2],1))+"\n")
            self.clockMSec += incTime
            #increment time 
            if self.clockMSec >= 1000:
                self.clockMSec = self.clockMSec%1000
                self.clockSec += 1
            
s = Simulator()
s.run()
            
    
        
        
        
        
        

        

        
        
        