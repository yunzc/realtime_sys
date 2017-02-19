# -*- coding: utf-8 -*-
"""
Created on Thu Feb  9 03:26:56 2017

@author: Yun Chang 
16.35 Real Time System and Software 
Problem Set 1
"""
import math 

class Control(object):
    """
    two qunatites: forward speed and rotational speed
    """
    def __init__(self, s, omega):
        """
        s is the forward speed (m/s) between 5 and 10
        omega is the rotational velocity (rad/s) between -pi/4 and pi/4
        """
        #test preconditions
        if s < 5 or s > 10:  
            raise ValueError('invalid forward speed')
        if omega < -math.pi/4 or omega > math.pi/4:
            raise ValueError('invalid rotational velocity')
        #assign to class instance variables 
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
        pose is [x, y, theta]
        s is the forward speed (m/s)
        omega is the rotational velocity (rad/s)
        """
        #test preconditions 
        if s < 5 or s > 10:
            raise ValueError('invalid forward speed')
        if omega < -math.pi/4 or omega > math.pi/4:
            raise ValueError('invalid rotational velocity')
        for i in range(2):
            if pose[i] < 0 or pose[i] > 100:
                raise ValueError('invalid pose')
        if pose[2] < -math.pi or pose[2] > math.pi: 
            raise ValueError('invalid or un-normalized orientation')
        #assignment to class instance variables 
        self.pose = pose
        x_dot = math.cos(pose[-1])*s #speed times the cosine of angle 
        y_dot = math.sin(pose[-1])*s
        self.velocity = [x_dot, y_dot, omega]
    
    def getPosition(self):
        """
        return and list of 3 floating point values that indicate x,y,theta
        """
        return self.pose
    
    def getVelocity(self):
        """
        return list of 3 values corresponding to the derivative of the pose
        """
        return self.velocity
    
    def setPosition(self, pose):
        """
        takes list of 3 values and set the corresponding internal representation 
        if attempt to exceed constraint, will be clamped at the limit closest 
        to desired 
        """
        #check if x, y value is out or constrained range, set to closest 
        for i in range(2): 
            if pose[i] < 0:
                pose[i] = 0 
            elif pose[i] > 100:
                pose[i] = 100
        #check that orientation is within constriant 
        if pose[2] < -math.pi:
            pose[2] = pose[2]%(2*math.pi)
        elif pose[2] > math.pi:
            pose[2] = pose[2]%(2*math.pi) - 2*math.pi 
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
            #retain the angle and adjust speed (magnitude)
            ang = math.atan2(y_dot, x_dot) 
            velocity[0] = 5*math.cos(ang)
            velocity[1] = 5*math.sin(ang)
        elif math.sqrt(x_dot**2 + y_dot**2) > 10:
            ang = math.atan2(y_dot, x_dot) 
            velocity[0] = 10*math.cos(ang)
            velocity[1] = 10*math.sin(ang)
        if velocity[2] > math.pi/4:
            velocity[2] = math.pi/4
        elif velocity[2] < -math.pi/4:
            velocity[2] = -math.pi/4
        self.velocity = velocity 
    
    def controlVehicle(self, c):
        """
        c is a Control instance. Modifies internal velocities accordinig to 
        specified forward speed and angular velocity
        """
        orien = self.pose[-1] #get current vehicle orientation 
        speed = c.getSpeed()
        rotSpeed = c.getRotVel()
        x_dot = speed*math.cos(orien) #calculate the velocity from speed 
        y_dot = speed*math.sin(orien)
        self.setVelocity([x_dot, y_dot, rotSpeed])
    
    def updateState(self, sec, msec):
        """
        changes the vehicles state by computing the change that will occur in time t
        t is given in sec and msec
        """
       
        time = sec + msec/1000
        #check preconditions
        if time < 0: 
            raise ValueError('cannot have negative time')
        veloc = self.getVelocity()
        posit = self.getPosition()
        #update position 
        newpose = []
        for i in range(3):
            newpose.append(posit[i] + veloc[i]*time)
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
        self.vehicle = GroundVehicle([50,50,0],5,0)
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
        #check that the input time is valid 
        if (sec + msec/1000) < 0: 
            raise ValueError('unclear what negative time means')
        incTime = 10
        turnSpeed = 5.0
        cruiseSpeed = 10.0 
        n = self.polygonSides
        radius = 25 #radius of the circle circumscribing the polygon 
        #define how to turn corner: partition to two parts, first turn half
        #the totally needed turn angle, then turn the other half
        #use law of cosines to get side length 
        extAng = math.radians(360/n) #polygon external angle (turn angle) in radians 
        sidelength = radius*math.sqrt(2 - 2*math.cos(extAng))
        turn_seg = turnSpeed*incTime/1000 #turn with small little segments 
        turnTime = (int(extAng/(math.pi/4))+1)*1000 #in mS
        turnAng = extAng/(turnTime/10) #turn by turning a small amount each time
        turnRotVel = turnAng/(0.01) #update turning rotation 
        turnRadius = turn_seg/math.tan(turnAng/2)/2 #the radius of turn 
        cruiseLen = sidelength - 2*turnRadius*math.tan(extAng/2)

        cruiseTime = (int(cruiseLen/cruiseSpeed)+1)*1000 #want this to be multiple of 10 (in mS)
        cruiseSpeed = cruiseLen/cruiseTime*1000 #update the cruise speed 
        timePerSide = cruiseTime + turnTime

        #if these quantities are not positive, something is seriously wrong 
        assert turnTime > 0
        assert cruiseTime > 0
        assert cruiseLen > 0 
        ######
        control = None 
        if ((sec)*1000 + msec) % timePerSide == 0:
            #at these times vehicle turning 
            control = Control(turnSpeed, turnRotVel) 
        elif ((sec)*1000 + msec) % timePerSide == turnTime:
            control = Control(cruiseSpeed, 0)
            #back to going straight
        return control 
    
    def setNumSides(self, n):
        """
        set the number of sides of the polygon the vehicle is tracing out 
        """
        if n >=3 and n <= 10:
            self.polygonSides = n
    
    def run(self, writeToFile = True):
        """
        set clock to zero, while total time is less than 100 secs, get control, 
        apply control, update state, print current time, position, and orientation 
        then increment clock by 10 millisecs 
        has the option of writing the data to a file in order to plot in Matlab 
        """
        if writeToFile: 
            f = open('data.txt','w')
        incTime = 10
        self.clockSec = 0
        self.clockMSec = 0
        while self.clockSec < 100:
            control = self.getControl(self.clockSec, self.clockMSec) #get control 
            if control != None: 
                self.vehicle.controlVehicle(control) #apply control 
            self.vehicle.updateState(0, 10) #increment by 10 milliseconds 
            time = round(self.clockSec + self.clockMSec/1000, 2)
            pose = self.vehicle.getPosition()
            print(time," ",round(pose[0],2)," ",round(pose[1],2)," ",round(pose[2],1))
            if writeToFile: 
                f.write(str(time)+" "+str(round(pose[0],2))+" "+str(round(pose[1],2))+" "+str(round(pose[2],1))+"\n")
            self.clockMSec += incTime
            #increment time 
            if self.clockMSec >= 1000:
                self.clockMSec = self.clockMSec%1000
                self.clockSec += 1
    
    def main(self):
        s = Simulator()
        s.run()

if __name__ == '__main__':
    Simulator().main()
            
    
        
        
        
        
        

        

        
        
        