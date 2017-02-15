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

class TestControl(unittest.TestCase):
    
    def test_constructor(self):
        """
        want to make sure the correct exception is raised if initialization values 
        are not in constrained range 
        input equivalence classes: 
            - s < 5, 5 <= s <= 10, s > 10
            - omega < (-pi/4), (-pi/4) <= omega <= (pi/4), omega > (pi/4)

        """
        #first a normal initialization 
        c1 = pset1.Control(5, -math.pi/4) #valid but on border s = 5, omega = -pi/4
        c2 = pset1.Control(5, math.pi/4) #valid but on border s = 5, omega = pi/4
        c3 = pset1.Control(10, -math.pi/4) #valid, on border s = 10, omega = -pi/4 
        self.assertEqual(c1.getSpeed(), 5, "speed initialization problem")
        self.assertEqual(c1.getRotVel(), -math.pi/4, "rotation velocity initialization problem")
        self.assertEqual(c2.getRotVel(), math.pi/4, "rotation velocity initialization problem")
        self.assertEqual(c3.getSpeed(), 10, "speed initialization problem")
        self.assertEqual(c3.getRotVel(), -math.pi/4, "rotation velocity initialization problem")
        #now test that for out of bound in speed an exception is raised 
        self.assertRaises(ValueError, pset1.Control, 4, math.pi/4) #invalid, on border s = 4
        self.assertRaises(ValueError, pset1.Control, 11, -math.pi/4) #invalid, on border s = 11
        #test out of bound for omega 
        self.assertRaises(ValueError, pset1.Control, 10, math.pi/4 + 0.1) #invalid, on border, omega = pi/4+0.1
        self.assertRaises(ValueError, pset1.Control, 5, -math.pi/4 - 0.1) #invalid, on border, omega = -pi/4-0.1
        #some other cases 
        self.assertRaises(ValueError, pset1.Control, 0, 0) #invalid s
        self.assertRaises(ValueError, pset1.Control, 7, 7) #invalid omega 
        
class TestGroundVehicle(unittest.TestCase):
    
    def setUp(self): 
        self.G = pset1.GroundVehicle([0,0,0], 5, -math.pi/4)
    
    def test_constructor(self):
        """
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
        #first a normal initialization 
        pose1 = [0,0,-math.pi]
        pose2 = [100, 100, math.pi]
        pose3 = [0,0,0]
        #some borderline invalid poses to test of exception raised 
        invalPose1 = [101, 0, math.pi]
        invalPose2 = [-1, 0, -math.pi]
        invalPose3 = [0, 101, math.pi]
        invalPose3 = [0, -1, -math.pi]
        invalPose4 = [0, 100, -math.pi-0.1]
        invalPose5 = [100, 0, math.pi+0.1]
        g1 = pset1.GroundVehicle(pose1, 5, -math.pi/4)
        g2 = pset1.GroundVehicle(pose2, 10, math.pi/4)
        g3 = pset1.GroundVehicle(pose3, 5, math.pi/4)
        self.assertEqual(g1.getPosition(), pose1, "pose initialization problem")
        self.assertEqual(g2.getPosition(), pose2, "pose initialization problem")
        self.assertEqual(g3.getPosition(), pose3, "pose initialization problem")
        self.assertRaises(ValueError, pset1.GroundVehicle, pose1, 4, math.pi/4)
        self.assertRaises(ValueError, pset1.GroundVehicle, pose2, 11, -math.pi/4)
        self.assertRaises(ValueError, pset1.GroundVehicle, pose1, 5, math.pi/4+0.1)
        self.assertRaises(ValueError, pset1.GroundVehicle, pose2, 10, -math.pi/4-0.1)
        #now test that for out-of-bound in pose an exception is raised 
        self.assertRaises(ValueError, pset1.GroundVehicle, invalPose1, 5, -math.pi/4)
        self.assertRaises(ValueError, pset1.GroundVehicle, invalPose2, 10, math.pi/4)
        self.assertRaises(ValueError, pset1.GroundVehicle, invalPose3, 5, math.pi/4)
        self.assertRaises(ValueError, pset1.GroundVehicle, invalPose4, 10, -math.pi/4)
        self.assertRaises(ValueError, pset1.GroundVehicle, invalPose5, 7, 0)

    def testSetPosition(self):
        """
        test that position is set correctly and out of constraint case is handled correctly 
        input equivalence class:
            - pose = [x,y,theta]
                x < 0, 0 <= x <= 100, x > 100
                y < 0, 0 <= y <= 100, y > 100
                theta < -pi, -pi <= theta <= pi, theta > pi 
        """
        G = self.G
        #normal cases, position should eqaul argument 
        sp1 = [0,0,-math.pi]
        sp2 = [100,100,math.pi]
        #just out of borderline cases, position should be the closest in bound 
        sp3 = [-1,-1,-math.pi-0.1]
        sp4 = [101, 101,math.pi+0.1]
        sp3_equal = [0,0,math.pi-0.1]
        sp4_equal = [100,100,-math.pi+0.1]
        #very out of bound cases, position should still be closest in bound 
        sp5 = [1000, 1000, math.pi+2*math.pi+0.1]
        sp6 = [-1000, -1000, -math.pi-2*math.pi-0.1]
        sp5_equal = [100, 100, -math.pi+0.1]
        sp6_equal = [0, 0, math.pi-0.1]
        G.setPosition(sp1)
        self.assertEqual(G.getPosition(),sp1,'standard setPos failed')
        G.setPosition(sp2)
        self.assertEqual(G.getPosition(),sp2,'standard setPos failed')
        G.setPosition(sp3)
        self.assertEqual(G.getPosition(),sp3_equal, 'outside constraint setPos failed')
        G.setPosition(sp4)
        self.assertEqual(G.getPosition(),sp4_equal, 'outside constraint setPos failed')
        for i in range(3): 
            G.setPosition(sp5)
            self.assertAlmostEqual(G.getPosition()[i],sp5_equal[i])
            G.setPosition(sp6)
            self.assertAlmostEqual(G.getPosition()[i],sp6_equal[i])
        
    def testSetVelocity(self):
        """
        test that velocity is set correctly and out of constraint cases are handled correctly 
        input equivalence class: 
            - vel = [x_dot, y_dot, omega] 
                sqrt(x_dot^2 + y_dot^2) < 5, 5 <= sqrt(x_dot^2 + y_dot^2) <= 10, sqrt(x_dot^2 + y_dot^2) > 10
                omega < -pi/4, -pi/4 <= omega <= pi/4, omega > pi/4 
        """
        G = self.G
        #test in constaint borderline cases 
        v1 = [3,4,-math.pi/4]
        v2 = [8,6,math.pi/4]
        #test out of constraint borderline cases 
        v3 = [2.7,3.6,-math.pi/4-0.1]
        v4 = [8.8,6.6,math.pi/4+0.1]
        v3_equal = [3, 4, -math.pi/4]
        v4_equal = [8, 6, math.pi/4]
        #way out case 
        v5 = [100, 100, 100*math.pi/4]
        v5_equal = [10/math.sqrt(2), 10/math.sqrt(2),math.pi/4]
        G.setVelocity(v1)
        self.assertEqual(G.getVelocity(),v1,'standard setVel failed')
        G.setVelocity(v2)
        self.assertEqual(G.getVelocity(),v2,'standard setVel failed')
        for i in range(3):
            G.setVelocity(v3)
            self.assertAlmostEqual(G.getVelocity()[i],v3_equal[i])
            G.setVelocity(v4)
            self.assertAlmostEqual(G.getVelocity()[i],v4_equal[i])
            G.setVelocity(v5)
            self.assertAlmostEqual(G.getVelocity()[i],v5_equal[i])

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
        G.controlVehicle(c1)
        for i in range(3):
            self.assertAlmostEqual(G.getVelocity()[i],v2[i])
        G.setPosition([0,0,math.pi/2])  
        G.controlVehicle(c2)
        for i in range(3): 
            self.assertAlmostEqual(G.getVelocity()[i],v3[i])
    
    def testUpdateState(self): 
        """
        test that state is updated correctly and catches if time is negative 
        """ 
        G = self.G
        #check that error is raised if input negative time 
        self.assertRaises(ValueError,G.updateState, 0, -0.1)
        p1 = [0,0,0]
        v1 = [5, 0, -math.pi/4]
        #check that 0,0 the fundamental case works 
        G.updateState(0,0)
        self.assertEqual(G.getPosition(),p1)
        self.assertEqual(G.getVelocity(),v1)
        p2 = [5,0,-math.pi/4]
        v2 = [5*math.cos(-math.pi/4),5*math.sin(-math.pi/4),-math.pi/4]
        G.updateState(1,0)
        for i in range(3): 
            self.assertAlmostEqual(G.getPosition()[i],p2[i])
            self.assertAlmostEqual(G.getVelocity()[i],v2[i])

        
if __name__ == '__main__':
    unittest.main()
        
