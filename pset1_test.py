# -*- coding: utf-8 -*-
"""
Created on Mon Feb 13 15:58:23 2017

@author: Yun Chang 
16.35 Real Time System and Software 
Problem Set 1 - Unit Tests 
"""

import unittest 
import pset1
import math 
import random 

class TestControl(unittest.TestCase):
    
    """
    testing the constructor: 
    want to make sure the correct exception is raised if initialization values 
    are not in constrained range 
    input equivalence classes: 
        - s < 5, 5 <= s <= 10, s > 10
        - omega < (-pi/4), (-pi/4) <= omega <= (pi/4), omega > (pi/4)
    """
    def test_constructor_valid(self):
        #first test normal initialization 
        #boundary cases 
        c1 = pset1.Control(5, -math.pi/4) #valid but on border s = 5, omega = -pi/4
        c2 = pset1.Control(5, math.pi/4) #valid but on border s = 5, omega = pi/4
        c3 = pset1.Control(10, -math.pi/4) #valid, on border s = 10, omega = -pi/4 
        self.assertEqual(c1.getSpeed(), 5, "speed initialization problem")
        self.assertEqual(c1.getRotVel(), -math.pi/4, "rotation velocity initialization problem")
        self.assertEqual(c2.getRotVel(), math.pi/4, "rotation velocity initialization problem")
        self.assertEqual(c3.getSpeed(), 10, "speed initialization problem")
        self.assertEqual(c3.getRotVel(), -math.pi/4, "rotation velocity initialization problem")
        #test 3 random cases
        random.seed(0)
        for i in range(3): 
            speed = random.random()*5 + 5 
            omeg = random.random()*math.pi/2 - math.pi/4
            c = pset1.Control(speed, omeg)
            self.assertEqual(c.getSpeed(),speed)
            self.assertEqual(c.getRotVel(),omeg)
    
    def test_constructor_invalid(self):
        #now test that for out of bound in speed an exception is raised 
        #boundary cases 
        self.assertRaises(ValueError, pset1.Control, 4, math.pi/4) #invalid, on border s = 4
        self.assertRaises(ValueError, pset1.Control, 11, -math.pi/4) #invalid, on border s = 11
        #test out of bound for omega 
        self.assertRaises(ValueError, pset1.Control, 10, math.pi/4 + 0.1) #invalid, on border, omega = pi/4+0.1
        self.assertRaises(ValueError, pset1.Control, 5, -math.pi/4 - 0.1) #invalid, on border, omega = -pi/4-0.1
        #some other random cases 
        self.assertRaises(ValueError, pset1.Control, 0, 0) #invalid s
        self.assertRaises(ValueError, pset1.Control, 7, 7) #invalid omega 
        
class TestGroundVehicle(unittest.TestCase):
    
    def setUp(self): 
        #set up with valid initialization to test methods to avoid redundancy 
        self.G = pset1.GroundVehicle([0,0,0], 5, math.pi/4)
    
    """
    testing constructor:
    want to make sure the correct exception is raised if initialization values 
    are not in constrained range 
    Input Equivalence Class:
        - pose = [x,y,theta]
        - x < 0, 0 <= x <= 100, x > 100
        - y < 0, 0 <= y <= 100, y > 100
        - theta < -pi, -pi <= theta <= pi, theta > pi 
        - s < 5, 5 <= s <= 10, s > 10
        - omega < (-pi/4), (-pi/4) <= omega <= (pi/4), omega > (pi/4)
    """  
    def testConstructor_valid(self): 
        #first test normal initialization
        #boundary cases 
        pose1 = [0,0,-math.pi]
        pose2 = [100, 100, math.pi]
        pose3 = [0,0,0]
        g1 = pset1.GroundVehicle(pose1, 5, -math.pi/4)
        g2 = pset1.GroundVehicle(pose2, 10, math.pi/4)
        g3 = pset1.GroundVehicle(pose3, 5, math.pi/4)
        self.assertEqual(g1.getPosition(), pose1, "pose initialization problem")
        self.assertEqual(g2.getPosition(), pose2, "pose initialization problem")
        self.assertEqual(g3.getPosition(), pose3, "pose initialization problem")
        #random cases 
        random.seed(0)
        for i in range(3): 
            x = random.random()*100
            y = random.random()*100
            thet = random.random()*2*math.pi-math.pi
            pose = [x, y, thet]
            #test with valid speed and omega
            g = pset1.GroundVehicle(pose, 5, math.pi/4)
            self.assertEqual(g.getPosition(), pose, "pose initialization problem")
    
    def testConstructor_invalid_s_or_omega(self): 
        #test for invalid speed
        #give a valid pose
        p = [0,0,0]
        #invalid speed
        self.assertRaises(ValueError, pset1.GroundVehicle, p, 4, math.pi/4)
        self.assertRaises(ValueError, pset1.GroundVehicle, p, 11, -math.pi/4)
        #invalid omega 
        self.assertRaises(ValueError, pset1.GroundVehicle, p, 5, math.pi/4+0.1)
        self.assertRaises(ValueError, pset1.GroundVehicle, p, 10, -math.pi/4-0.1)
    
    def testConstructor_invalid_pose(self):
        #test invalid initialization
        #some borderline invalid poses to test of exception raised 
        invalPose1 = [101, 0, math.pi]
        invalPose2 = [-1, 0, -math.pi]
        invalPose3 = [0, 101, math.pi]
        invalPose3 = [0, -1, -math.pi]
        invalPose4 = [0, 100, -math.pi-0.1]
        invalPose5 = [100, 0, math.pi+0.1]
        #now test that for out-of-bound in pose an exception is raised 
        self.assertRaises(ValueError, pset1.GroundVehicle, invalPose1, 5, -math.pi/4)
        self.assertRaises(ValueError, pset1.GroundVehicle, invalPose2, 10, math.pi/4)
        self.assertRaises(ValueError, pset1.GroundVehicle, invalPose3, 5, math.pi/4)
        self.assertRaises(ValueError, pset1.GroundVehicle, invalPose4, 10, -math.pi/4)
        self.assertRaises(ValueError, pset1.GroundVehicle, invalPose5, 7, 0)
        #3 random out of bound cases 
        random.seed(0)
        for i in range(3): 
            p = [0,0,0]
            #psuedo randomly alter one value to be out of constraint 
            if i != 2: 
                p[i] = random.random()*100 + 100
            else: 
                p[i] = -math.pi - random.random()*2*math.pi 
            self.assertRaises(ValueError, pset1.GroundVehicle, p, 7, 0)
    
    """
    test that position is set correctly and out of constraint case is handled correctly 
    input equivalence class:
        - pose = [x,y,theta]
            x < 0, 0 <= x <= 100, x > 100
            y < 0, 0 <= y <= 100, y > 100
            theta < -pi, -pi <= theta <= pi, theta > pi 
    """
    def testSetPosition_normal(self):
        G = self.G
        #normal cases, position should eqaul argument test for equality 
        sp1 = [0,0,-math.pi]
        sp2 = [100,100,math.pi]
        sp3 = [50, 50, 0]
        G.setPosition(sp1)
        self.assertEqual(G.getPosition(),sp1,'standard setPos failed')
        G.setPosition(sp2)
        self.assertEqual(G.getPosition(),sp2,'standard setPos failed')
        G.setPosition(sp3)
        self.assertEqual(G.getPosition(),sp3,'standard setPos failed')
    
    def testSetPosition_outSide(self):
        G = self.G
        #out of constriant cases, position should be the closest in bound 
        sp1 = [-1,-1,-math.pi-0.1]
        sp2 = [101, 101,math.pi+0.1]
        sp1_equal = [0,0,math.pi-0.1]
        sp2_equal = [100,100,-math.pi+0.1]
        #very out of bound cases, position should still be closest in bound 
        sp3 = [1000, 1000, math.pi+2*math.pi+0.1]
        sp4 = [-1000, -1000, -math.pi-2*math.pi-0.1]
        sp3_equal = [100, 100, -math.pi+0.1]
        sp4_equal = [0, 0, math.pi-0.1]
        
        G.setPosition(sp1)
        self.assertEqual(G.getPosition(),sp1_equal, 'outside constraint setPos failed')
        G.setPosition(sp2)
        self.assertEqual(G.getPosition(),sp2_equal, 'outside constraint setPos failed')
        #needed to use almost eqaul since didn't give exact answer (to the 8th floating point)
        for i in range(3): 
            G.setPosition(sp3)
            self.assertAlmostEqual(G.getPosition()[i],sp3_equal[i])
            G.setPosition(sp4)
            self.assertAlmostEqual(G.getPosition()[i],sp4_equal[i])
            
    """
    test that velocity is set correctly and out of constraint cases are handled correctly 
    input equivalence class: 
        - vel = [x_dot, y_dot, omega] 
            sqrt(x_dot^2 + y_dot^2) < 5, 5 <= sqrt(x_dot^2 + y_dot^2) <= 10, sqrt(x_dot^2 + y_dot^2) > 10
            omega < -pi/4, -pi/4 <= omega <= pi/4, omega > pi/4 
    """
    def testSetVelocity_normal(self):
        #testing normal velocities 
        G = self.G
        #test in constaint borderline cases 
        v1 = [3,4,-math.pi/4]
        v2 = [8,6,math.pi/4]
        #test randomly chosen in constraint case 
        v3 = [3,3,0] 
        #test equality 
        G.setVelocity(v1)
        self.assertEqual(G.getVelocity(),v1,'standard setVel failed')
        G.setVelocity(v2)
        self.assertEqual(G.getVelocity(),v2,'standard setVel failed')
        G.setVelocity(v3)
        self.assertEqual(G.getVelocity(),v3,'standard setVel failed')
    
    def testSetVelocity_outSide(self):
        G = self.G
        #test out of constraint borderline cases 
        v1 = [2.7,3.6,-math.pi/4-0.1]
        v2 = [8.8,6.6,math.pi/4+0.1]
        v1_equal = [3, 4, -math.pi/4]
        v2_equal = [8, 6, math.pi/4]
        #way out case 
        v3 = [100, 100, 100*math.pi/4]
        v3_equal = [10/math.sqrt(2), 10/math.sqrt(2),math.pi/4]
        v4 = [0, 0, -100*math.pi/4]
        v4_equal = [5, 0, -math.pi/4]
        
        for i in range(3):
            G.setVelocity(v1)
            self.assertAlmostEqual(G.getVelocity()[i],v1_equal[i])
            G.setVelocity(v2)
            self.assertAlmostEqual(G.getVelocity()[i],v2_equal[i])
            G.setVelocity(v3)
            self.assertAlmostEqual(G.getVelocity()[i],v3_equal[i])
            G.setVelocity(v4)
            self.assertAlmostEqual(G.getVelocity()[i],v4_equal[i])

    def testControlVehicle(self): 
        """
        test that the control modifies the vehicle velocity correctly 
        """
        G = self.G
        c1 = pset1.Control(5,-math.pi/4)
        c2 = pset1.Control(10,math.pi/4)
        v1 = [5,0,-math.pi/4]
        v2 = [3,4,-math.pi/4]
        v3 = [0,10,math.pi/4]
        #don't need to test out of bound cases since Control class already restircts out of bound initialization
        G.controlVehicle(c1)
        for i in range(3):
            self.assertAlmostEqual(G.getVelocity()[i],v1[i])
        G.setPosition([0,0,math.atan2(4,3)]) #change orientation 
        #just test that controlVehicle generates velocity correctly from speed
        G.controlVehicle(c1)
        for i in range(3):
            self.assertAlmostEqual(G.getVelocity()[i],v2[i])
        G.setPosition([0,0,math.pi/2])  
        G.controlVehicle(c2)
        for i in range(3): 
            self.assertAlmostEqual(G.getVelocity()[i],v3[i])
    
    def testUpdateState_catchInvalidTime(self): 
        G = self.G
        #check that error is raised if input negative time 
        self.assertRaises(ValueError,G.updateState, 0, -0.1)
        
    def testUpdateState(self): 
        """
        test that state is updated correctly and catches if time is negative 
        """ 
        G = self.G
        p1 = [0,0,0]
        v1 = [5, 0, math.pi/4]
        #check that 0,0 the fundamental case works (stays in place) 
        G.updateState(0,0)
        self.assertEqual(G.getPosition(),p1)
        self.assertEqual(G.getVelocity(),v1)
        #check that moving a second gives correct update
        p2 = [5,0,math.pi/4]
        v2 = [5*math.cos(math.pi/4),5*math.sin(math.pi/4),math.pi/4]
        G.updateState(1,0)
        for i in range(3): 
            self.assertAlmostEqual(G.getPosition()[i],p2[i])
            self.assertAlmostEqual(G.getVelocity()[i],v2[i])
        #check that moving for a milisecond gives correct update
        p3 = [p2[0]+v2[0]*0.001, p2[1]+v2[1]*0.001, p2[2]+v2[2]*0.001]
        v3 = [5*math.cos(p3[2]), 5*math.sin(p3[2]), math.pi/4]
        G.updateState(0,1)
        for i in range(2):
            self.assertAlmostEqual(G.getPosition()[i],p3[i])
            self.assertAlmostEqual(G.getVelocity()[i],v3[i])

class TestSimulator(unittest.TestCase): 
    """
    for getControl method, input equivalence classes: 
    sec + msec/1000 < 0, 0 <= sec + msec/1000 
    """
    def setUp(self):
        #set up to test methods to avoid redundancy 
        self.Sim = pset1.Simulator()
        #iterate through once get take data 
        #note vehicle will start off turning before going straight
        #start -> turn -> striaght -> turn -> straight (loop) 
        numTurns = 0 #note how many time it turns to later check is reasonable
        cruiseTime = 0  #note the time it is going in a striaght line to check reasonable
        time_cruiseTime = True  #when to time the cruiseTime
        turnTime = 0 #not the time vehicle turns to check reasonable 
        time_turnTime = True 
        timePoints = [i*.01 for i in range(10000)]
        for time in timePoints:#iterate through the 100 seconds 
            sec = int(time)
            msec = 1000*(time - sec)
            control = self.Sim.getControl(sec, msec)
            if control == None: 
                if time_turnTime: 
                    turnTime += 0.01 
                elif time_cruiseTime:
                    cruiseTime += 0.01
            elif control.getRotVel() != 0: 
                numTurns += 1 #there is a turn when there is a rotation velocity
                if cruiseTime != 0: 
                    time_cruiseTime = False #this will be on the seocnd turn, time first turn
            else: 
                if turnTime != 0:
                    time_turnTime = False #this is when it just finished first turn 
        self.numTurns = numTurns 
        self.cruiseTime = cruiseTime
        self.sideTime = cruiseTime + turnTime 
        self.turnTime = turnTime 

    def testGetControl_negativeTime(self):
        S = self.Sim
        self.assertRaises(ValueError, S.getControl, -1, 0) #negative seconds 
        self.assertRaises(ValueError, S.getControl, 0, -1) #neg millisec 
        self.assertRaises(ValueError, S.getControl, -10, -10) #neg time 

    def testGetControl_numSides(self): 
        """
        monitor the number of times controls implies turning to check that 
        that the number of times the vehicle turns  >= the number of sides 
        of the polygon
        """
        S = self.Sim
        numSides = S.polygonSides
        turnNum = self.numTurns
        self.assertGreaterEqual(turnNum, numSides)
    
    def testGetControl_turnTime(self):
        """
        check that the turn time is reasonable considering the omega constraints 
        and the turn angle 
        """ 
        S = self.Sim
        n = S.polygonSides
        extAng = 2*math.pi/n #external angle of polygon, the angle the vehicle needs to turn 
        #check that if turning at the fastest omega the turn would be >= extAng
        largestPossibleTurn = self.turnTime*math.pi/4 #since largest omega is pi/4
        self.assertGreaterEqual(largestPossibleTurn, extAng)
    
    def testGetControl_sideTime(self): 
        """
        check that the time spent on a side is reasonable considering the side 
        lengths according to the number of sides 
        """
        S = self.Sim
        n = S.polygonSides
        extAng = 2*math.pi/n #polygon external angle (turn angle) in radians 
        sidelength = 25*math.sqrt(2 - 2*math.cos(extAng)) #POLYGON circumscribed in 50m diameter circle 
        #check that the sidelength is between 5*cruiseTime and 10*cruiseTime
        sideTime = self.sideTime
        self.assertLessEqual(5*sideTime, sidelength)
        self.assertGreaterEqual(10*sideTime, sidelength)
        
        
if __name__ == '__main__':
    testClasses = [TestControl, TestGroundVehicle, TestSimulator]
    loader = unittest.TestLoader()
    suites = []
    for testClass in testClasses:
        suite = loader.loadTestsFromTestCase(testClass)
        suites.append(suite)
    testSuites = unittest.TestSuite(suites)
    unittest.TextTestRunner(verbosity=2).run(testSuites)

        
