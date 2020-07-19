'''In this file you need to implement remote procedure call (RPC) server

* There are different RPC libraries for python, such as xmlrpclib, json-rpc. You are free to choose.
* The following functions have to be implemented and exported:
 * get_angle
 * set_angles
 * get_posture
 * execute_keyframes
 * get_transform
 * set_transform
* You can test RPC server with ipython before implementing agent_client.py
'''

# add PYTHONPATH
from xmlrpc.server import SimpleXMLRPCServer
from xmlrpc.server import SimpleXMLRPCRequestHandler

import threading
import os 
import sys

from numpy.matlib import matrix, identity

sys.path.append(os.path.join(os.path.abspath(os.path.dirname(__file__)), '..', 'kinematics'))

from inverse_kinematics import InverseKinematicsAgent


# Restrict to a particular path.
class RequestHandler(SimpleXMLRPCRequestHandler):
    rpc_paths = ('/RPC2',)

class ServerAgent(InverseKinematicsAgent):
    '''ServerAgent provides RPC service
    '''
    # YOUR CODE HERE
    
    def get_angle(self, joint_name):
        '''get sensor value of given joint'''
        # YOUR CODE HERE
        print("server executing get_angle:",joint_name)
        return self.target_joints.get(joint_name)
    
    def set_angle(self, joint_name, angle):
        '''set target angle of joint for PID controller
        '''
        # YOUR CODE HERE
        print("server executing set_angle:", joint_name)
        self.target_joints[joint_name] = angle

    def get_posture(self):
        '''return current posture of robot'''
        # YOUR CODE HERE
        print("server executing get_posture")
        return self.posture

    def execute_keyframes(self, keyframes):
        '''excute keyframes, note this function is blocking call,
        e.g. return until keyframes are executed
        '''
        # YOUR CODE HERE 
        print("server executing keyframes")
        self.keyframes = keyframes

    def get_transform(self, name):
        '''get transform with given name
        '''
        # YOUR CODE HERE
        T = self.transforms[name]
        print("get transform T: \n", T)

        # marshalling 
        lst = [None] * 16
        lst = [T[0, 0], T[1, 0], T[2, 0], T[3, 0],
               T[0, 1], T[1, 1], T[2, 1], T[3, 1],
               T[0, 2], T[1, 2], T[2, 2], T[3, 2],
               T[0, 3], T[1, 3], T[2, 3], T[3, 3]]
        

        b = [float(x) for x in lst]
        print("server executing get_transform for:", name,"\n | \n",type(lst))

        return b

    def set_transform(self, effector_name, transform):
        '''solve the inverse kinematics and control joints use the results
        '''
        # YOUR CODE HERE
        print("server executing set_transform:", effector_name)
        # unmarshalling 
        T = transform
        transform = matrix([[T[0], T[4], T[8], T[12]],
                            [T[1], T[5], T[9], T[13]],
                            [T[2], T[6], T[10], T[14]],
                            [T[3], T[7], T[11], T[15]]])
        print("tranform:\n",transform)
        self.set_transforms(effector_name, transform)

if __name__ == '__main__':
    print("~#~#~#~#~#~ Starting S Agent ~#~#~#~#~#~")
    agent = ServerAgent()

    #print("~#~#~#~#~#~ Simple XML RPC Server ~#~#~#~#~#~")
    #server = SimpleXMLRPCServer(("localhost", 8000), requestHandler=RequestHandler,  allow_none=True, logRequests=True)
    server = SimpleXMLRPCServer(
        ("localhost", 8000), requestHandler=RequestHandler,  allow_none=True)
    #print("~#~#~#~#~#~ Register Introspection Functions ~#~#~#~#~#~")
    server.register_introspection_functions()

    #print("~#~#~#~#~#~ Register Multicall Functions ~#~#~#~#~#~")
    server.register_multicall_functions()
    
    #print("~#~#~#~#~#~ Register Instance ~#~#~#~#~#~")
    server.register_instance(agent)
    
    #print("~#~#~#~#~#~ Thread: server serve forever ~#~#~#~#~#~")
    thread = threading.Thread(target=server.serve_forever)
    thread.start()

    #print("~#~#~#~#~#~ Server started on localhost:8000 ~#~#~#~#~#~")
    agent.run()

