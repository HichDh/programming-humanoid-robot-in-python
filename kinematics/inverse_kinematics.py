'''In this exercise you need to implement inverse kinematics for NAO's legs

* Tasks:
    1. solve inverse kinematics for NAO's legs by using analytical or numerical method.
       You may need documentation of NAO's leg:
       http://doc.aldebaran.com/2-1/family/nao_h21/joints_h21.html
       http://doc.aldebaran.com/2-1/family/nao_h21/links_h21.html
    2. use the results of inverse kinematics to control NAO's legs (in InverseKinematicsAgent.set_transforms)
       and test your inverse kinematics implementation.
'''


from forward_kinematics import ForwardKinematicsAgent
from numpy.matlib import identity, sin, cos, pi, matrix, random, linalg, asarray
from math import atan2
from scipy.linalg import pinv
import numpy as np


class InverseKinematicsAgent(ForwardKinematicsAgent):
    def inverse_kinematics(self, effector_name, transform):
        '''solve the inverse kinematics

        :param str effector_name: name of end effector, e.g. LLeg, RLeg
        :param transform: 4x4 transform matrix
        :return: list of joint angles
        '''
        joint_angles = []
        lambda_ = 1
        max_step = 0.1
        
        # Extracting joint angles for joints of the chain
        joint_angles = {name : self.perception.joint[name] for name in self.chains[effector_name]}
        
        # Setting end effector for the given effector name
        target_effector = self.chains[effector_name][-1] 
        
        # Getting target x,y,z theta_x, theta_y, theta_z
        target = matrix(self.from_trans(transform)).T
        
        for i in range(1000):
            # Executing forward kinematics
            self.forward_kinematics(joint_angles) 
            
            # Result of forward kinematics for effector joints
            Ts = [0] * len(self.chains[effector_name])
            for i, name in enumerate(self.chains[effector_name]):
                Ts[i] = self.transforms[name]
               
            # Getting current x,y,z theta_x, theta_y, theta_z for end effectors 
            Te = matrix([self.from_trans(Ts[-1])]).T
            
            # Calculating distance between target and current result
            e = target - Te
            
            # Adjusting error factor 
            e[e > max_step] = max_step
            e[e < -max_step] = -max_step
            
            T = matrix([self.from_trans(i) for i in Ts[0:len(self.chains[effector_name])]]).T
            
            # Getting jacobian matrix
            J = target - T
            dT = target - T
            J[0, :] = -dT[1, :] # x
            J[1, :] = dT[0, :] # y
            J[-1, :] = 1
                        
            d_theta = lambda_ * pinv(J) * e
            
            for i, name in enumerate(self.chains[effector_name]):
                joint_angles[name] += np.asarray(d_theta.T)[0][i]
                
            # Breaking if scalar product of error is lower than tolerated error (angles are satisfiable)
            if np.linalg.norm(d_theta) < 1e-3:
                break
        
        return joint_angles

    def from_trans(self, T):
        """
        :param transform: transformation matrix
        :return array with coordinates and angles
        """
        # row major
        # x, y, z = T[-1, 0], T[-1, 1], T[-1, 2]
        x, y, z = T[3, 0], T[3, 1], T[3, 2]

        theta_x, theta_y, theta_z = 0, 0, 0

        if T[0, 0] == 1:
            theta_x = atan2(T[2, 1], T[1, 1])
        elif T[1, 1] == 1:
            theta_y = atan2(T[0, 2], T[0, 0])
        elif T[2, 2] == 1:
            theta_z = atan2(T[1, 0], T[0, 0])

        return np.array([x, y, z, theta_x, theta_y, theta_z])
    
    
    def set_transforms(self, effector_name, transform):
        '''solve the inverse kinematics and control joints use the results
        '''
        # YOUR CODE HERE
        
        joint_angles = self.inverse_kinematics(effector_name, transform)
        names = self.chains[effector_name]
        times = [[0, 5]] * len(names)
        keys = []
        for i, name in enumerate(names):
            keys.insert(i, [[self.perception.joint[name], [3, 0, 0]], [joint_angles[name], [3, 0, 0]]])
        
        self.keyframes = (names, times, keys)  # the result joint angles have to fill in

if __name__ == '__main__':
    agent = InverseKinematicsAgent()
    # test inverse kinematics
    T = identity(4)
    # row major
    T[-1, 1] = -0.50
    T[-1, 2] = 0.50
    agent.set_transforms('LArm', T)
    agent.run()
