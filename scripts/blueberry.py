#!/usr/bin/env python
# license removed for brevity
import rospy
from geometry_msgs.msg import Point
import serial
import random
import numpy as np
from sympy import sin, cos, Matrix, symbols

class IKSolver:
    """Class for performing iterative inverse kinematics for the blueberry robot"""

    #Note: Joint arrays are listed in order according to the following indexing:
    #[0] = shoulder rotation, [0] = shoulder flexure, [0] = elbow flexure, [0] = wrist rotation, [4] = wrist flexure 

    #Joint Limits in radians as tuples of [0] = lower bound and [1] = upper bound
    _joint_limits = [(-1.134, 1.833), (-2.182, 0.785), (-1.309, 1.658), (-1.571, 1.396), (-1.396, 1.571)]
    #Joints for which the servos were mounted in reverse (I.E. positive increase in input value equals radian decrease)
    _rev_joints = [False, True, False, False, True]
    #Home position for the robot (pointing straight up)
    _home = [70, 130, 80, 95, 85]
    #Neutral gripper position to avoid servo strain
    _grip_neutral = 30
    #Fine tuning parameters for iterative inverse kinematics
    #---How many gradient descents to perform while looking for a solution
    _maxIterations = 5
    #---How far to move on each step of an interation
    _stepSize = .01
    #---How close to the goal the solution must be to be sufficient
    _goalAccuracy = .05
    #---How many steps for each gradient descent
    _subIterations = 250
    #Current joint angles of the robot
    _current = [0, 0, 0, 0, 0]

    #Homogeneous transforms representing each frame from the base of the robot 
    # (Directly under the center of the robot)
    # to the gripper ee (1 cm below the deepest part of the gripper while open)
    # used to calculate forward kinematics
    _q1, _q2, _q3, _q4, _q5 = symbols('_q1 _q2 _q3 _q4 _q5')
    _T1 = Matrix([[cos(_q1),  -sin(_q1), 0,       0],
                 [sin(_q1),  cos(_q1),  0,       0],
                 [0,        0,        1,       0.085],
                 [0,        0,        0,       1]])
    _T2 = Matrix([[cos(_q2),  0,        sin(_q2), cos(_q2 + np.pi/2) * 0.085],
                 [0,        1,        0,       0],
                 [-sin(_q2), 0,        cos(_q2), sin(_q2 + np.pi/2) * 0.085],
                 [0,        0,        0,       1]])
    _T3 = Matrix([[cos(_q3),  0,        sin(_q3), cos(_q3 + np.pi/2) * 0.115],
                 [0,        1,        0,       0],
                 [-sin(_q3), 0,        cos(_q3), sin(_q3 + np.pi/2) * 0.115],
                 [0,        0,        0,       1]])
    _T4 = Matrix([[cos(_q4),  -sin(_q4), 0,       0],
                 [sin(_q4),  cos(_q4),  0,       0],
                 [0,        0,        1,       0],
                 [0,        0,        0,       1]])
    _T5 = Matrix([[cos(_q5),  0,        sin(_q5), cos(_q5 + np.pi/2) * 0.06],
                 [0,        1,        0,       0],
                 [-sin(_q5), 0,        cos(_q5), sin(_q5 + np.pi/2) * 0.06],
                 [0,        0,        0,       1]])
    _FK = _T1 * _T2 * _T3 * _T4 * _T5


    def __init__(self, usbPort):
        """Initialize a connection to the robot with inverse kinematic control"""
        #Connect to board via usb (must have BlueberryHardwareControl loaded on it)
        self.ser = serial.Serial(usbPort, 9600, timeout = .5)
        #Get the translation only portion of the forward kinematics (since we don't have full 6 DOF for translation and orientation) 
        justTranslation = self._FK.col(-1)
        justTranslation.row_del(-1)
        input = Matrix([self._q1, self._q2, self._q3, self._q4, self._q5])
        #Get jacobian for gradient descent
        self.translationJacobian = justTranslation.jacobian(input)

    def getEE(self, q):
        """Get the homogeneous transform of the end effector in the base frame given an array of joint angles"""
        substituteDict = {self._q1: q[0],
                          self._q2: q[1],
                          self._q3: q[2],
                          self._q4: q[3],
                          self._q5: q[4]}
        return self._FK.evalf(subs=substituteDict)

    def solveForEFTranslation(self, goalX):
        """Get the joint configuration that will get the robot's end effector to (or close to) the goal position. 
        Takes in a 3d vector as a goal position
        Returns a vector of joint angles """
        iterations = 0
        goal = Matrix(goalX)
        dx = Matrix([1, 1, 1])
        bestDX = Matrix([999, 999, 999])
        bestQ = []
        #Iterate until you find an acceptable end effector position or hit the max iterations
        while iterations < self._maxIterations and bestDX.norm() > self._goalAccuracy:
            iterations += 1
            #Initialize to a random initial configuration (avoid local minimums)
            q = [self._joint_limits[0][0] + random.random()*self._joint_limits[0][1],
            self._joint_limits[1][0] + random.random()*self._joint_limits[1][1],
            self._joint_limits[2][0] + random.random()*self._joint_limits[2][1],
            self._joint_limits[3][0] + random.random()*self._joint_limits[3][1],
            self._joint_limits[4][0] + random.random()*self._joint_limits[4][1]]
            k = 0
            #Perfrom gradient descent over the next subIteration number of steps
            numNudges = 0
            while k < self._subIterations:
                k += 1
                #Get the difference of the end effector from the goal position
                x = self.getEE(q)
                x = x.col(-1)
                x.row_del(-1)
                dx = goal - x
                #Use the inverse (pseduo, as this isn't square) of the jacobian at the current joint configuration to get a 
                #change in q that lowers the difference of the end effector from the goal position
                substituteDict = {self._q1: q[0],
                                  self._q2: q[1],
                                  self._q3: q[2],
                                  self._q4: q[3],
                                  self._q5: q[4]}
                jacobOutput = self.translationJacobian.evalf(subs=substituteDict)
                dq = self._stepSize * (jacobOutput.pinv() * dx)
                #Use change in q to approach the best joint configuration
                newQ = Matrix(q) + dq
                #Exit solution attempt if too many nudges are occurring relative to the maximum number of gradient descent steps
                #We don't want to spend too much time on a solution that is boarderline not reachable
                if numNudges >= self._subIterations/2:
                    break
                #Check the new configuration to make sure it is in bounds, and nudge it back in bounds if so
                ind = 0
                for angle in newQ:
                    nudged = True
                    if not self._rev_joints[ind]:
                        if angle < self._joint_limits[ind][0]:
                            q[ind] = self._joint_limits[ind][0]
                        elif angle > self._joint_limits[ind][1]:
                            q[ind] = self._joint_limits[ind][1]
                        else:
                            q[ind] = angle
                            nudged = False
                    else:
                        if angle < -1 * self._joint_limits[ind][1]:
                            q[ind] = -1 * self._joint_limits[ind][1]
                        elif angle > -1 * self._joint_limits[ind][0]:
                            q[ind] = -1 * self._joint_limits[ind][0]
                        else:
                            q[ind] = angle
                            nudged = False
                    ind += 1
                if nudged:
                    numNudges += 1
                
            #Recalculate the distance to the goal angle post nudging to check if it is the best solution yet
            x = self.getEE(q)
            x = x.col(-1)
            x.row_del(-1)
            dx = goal - x
            #Store the configuration if it is the best one we've seen thusfar
            if dx.norm() < bestDX.norm():
                print("Best dx (" + str(bestDX.norm()) + ") replaced with new diff: " + str(dx.norm()))
                bestDX = dx
                bestQ = q
            else:
                print("dx not replaced")    
        return bestQ

    def setAnglesAndGrip(self, qVector, grip):
        """Send a vector of joint angles and a gripper position over serial to the robot
        Returns the new joint angles"""
        #Clear the serial connection for a new exchange
        self.ser.flushInput()
        self.ser.flushOutput()
        #Combine joint vector into a single string command for sending over serial
        intQ = []
        for floatVal in qVector:
            intQ.append(int(round(floatVal)))
        command = str(intQ[0]) + ',' + str(intQ[1]) + ',' + str(intQ[2]) + ',' + str(intQ[3]) + ',' + str(intQ[4]) + ',' + str(grip) + '\n'
        print("Sending: " + command)
        self.ser.write(command.encode('utf-8'))

        #wait until a repsonse if found from the arduino
        OK = 'no'
        while (OK != 'd'):
            print("WaitingOnResponse: " + OK)
            OK = self.ser.read(1).decode("utf-8")
        print("ReceivedResponse")

        return intQ + [grip]

    def convertIKQToDegrees(self, q):
        """convert a vector of joint angles in radians from IK solution to degrees suitable for sending to the servos"""
        qNew = []
        ind = 0
        for angle in q:
            if not self._rev_joints[ind]:
                qNew.append((angle * (180.0/np.pi)) + self._home[ind])
            else:
                qNew.append(self._home[ind] - (angle * (180.0/np.pi)))
            ind += 1
        return qNew

    def home(self):
        """Send robot to home position and record angles"""
        self._current = self.setAnglesAndGrip(self._home, self._grip_neutral)

    def moveToGoalAbsolute(self, point):
        """Send robot to the given goal position and record angles"""
        pointAsVector = [point.x, point.y, point.z]
        bestConfig = self.solveForEFTranslation(pointAsVector)
        print("output angles")
        print(bestConfig)
        print("fk of output angles")
        print(self.getEE(bestConfig))
        qNew = self.convertIKQToDegrees(bestConfig)
        self._current = self.setAnglesAndGrip(qNew, self._grip_neutral)

    def moveToGoalRelative(self, point):
        """Adjust robot by the given differential vector and record angles""" 
        pointAsVector = [point.x, point.y, point.z]
        x = self.getEE(self._current)
        x = x.col(-1)
        x.row_del(-1)
        relativePoint = x + Matrix(pointAsVector)
        bestConfig = self.solveForEFTranslation(relativePoint)
        print("output angles")
        print(bestConfig)
        print("fk of output angles")
        print(self.getEE(bestConfig))
        qNew = self.convertIKQToDegrees(bestConfig)
        self._current = self.setAnglesAndGrip(qNew, self._grip_neutral)
        

if __name__ == '__main__':
    #Start robot connection and reset robot
    blueberry = IKSolver("/dev/ttyUSB0")
    blueberry.home()
    #Advertise self to ros
    rospy.init_node('blueberry', anonymous=True)
    #Spin up multiple control nodes
    rospy.Subscriber("blueberry/translationAbsolute", Point, blueberry.moveToGoalAbsolute)
    rospy.Subscriber("blueberry/translationRelative", Point, blueberry.moveToGoalRelative)
    #Keep node alive while doing callbacks
    rospy.spin()