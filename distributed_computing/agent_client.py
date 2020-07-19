'''In this file you need to implement remote procedure call (RPC) client

* The agent_server.py has to be implemented first (at least one function is implemented and exported)
* Please implement functions in ClientAgent first, which should request remote call directly
* The PostHandler can be implement in the last step, it provides non-blocking functions, e.g. agent.post.execute_keyframes
 * Hints: [threading](https://docs.python.org/2/library/threading.html) may be needed for monitoring if the task is done
'''

from numpy.matlib import matrix, identity

from keyframes import hello
from keyframes import leftBackToStand

import weakref
import threading
import xmlrpc.client as xmlrpc
import time 


class PostHandler(object):
    '''the post hander wraps function to be excuted in paralle
    '''
    def __init__(self, obj):
        self.proxy = weakref.proxy(obj)

    def execute_keyframes(self, keyframes):
        '''non-blocking call of ClientAgent.execute_keyframes'''
        # YOUR CODE HERE
        thread = threading.Thread(target=self.proxy.execute_keyframes, args=[keyframes])
        thread.start()


    def set_transform(self, effector_name, transform):
        '''non-blocking call of ClientAgent.set_transform'''
        # YOUR CODE HERE
        thread = threading.Thread(target=self.proxy.set_transform, args=[effector_name, transform])
        thread.start()

    

class ClientAgent(object):
    '''ClientAgent request RPC service from remote server
    '''
    # YOUR CODE HERE
    def __init__(self):
        self.post = PostHandler(self)
        self.rpcProxy = xmlrpc.ServerProxy("http://localhost:8000")
    
    def get_angle(self, joint_name):
        '''get sensor value of given joint'''
        # YOUR CODE HERE
        print("client executing get_angle:", joint_name)
        return self.rpcProxy.get_angle(joint_name)
    
    def set_angle(self, joint_name, angle):
        '''set target angle of joint for PID controller
        '''
        # YOUR CODE HERE
        print("client executing set_angle:", joint_name)
        self.rpcProxy.set_angle(joint_name, angle)

    def get_posture(self):
        '''return current posture of robot'''
        # YOUR CODE HERE
        print("client executing get_posture")
        # return self.rpcProxy.get_posture(self)
        return self.rpcProxy.get_posture()

    def execute_keyframes(self, keyframes):
        '''excute keyframes, note this function is blocking call,
        e.g. return until keyframes are executed
        '''
        # YOUR CODE HERE
        print("server executing execute keyframes")
        self.rpcProxy.execute_keyframes(keyframes)

    def get_transform(self, name):
        '''get transform with given name
        '''
        # YOUR CODE HERE
        T = self.rpcProxy.get_transform(name)
        print("client executing get_transform:", name)

        # Unmarshalling list to matrix 
        transform = matrix([[T[0], T[4], T[8], T[12]],
                             [T[1], T[5], T[9], T[13]],
                             [T[2], T[6], T[10], T[14]],
                             [T[3], T[7], T[11], T[15]]])
        #print("transform:\n",transform)
        return transform

    def set_transform(self, effector_name, transform):
        '''solve the inverse kinematics and control joints use the results
        '''
        # YOUR CODE HERE
        print("client executing set_transform:", effector_name)

        # Marshalling 
        lmp = [None] * 16
        lmp = [T[0, 0], T[1, 0], T[2, 0], T[3, 0],
            T[0, 1], T[1, 1], T[2, 1], T[3, 1],
            T[0, 2], T[1, 2], T[2, 2], T[3, 2],
            T[0, 3], T[1, 3], T[2, 3], T[3, 3]]

        b = [float(x) for x in lmp]
        self.rpcProxy.set_transform(effector_name, b)

if __name__ == '__main__':
    agent = ClientAgent()

    # TEST CODE HERE
    print("~#~#~#~#~#~ Starting C Agent ~#~#~#~#~#~")
    
    menu = {}
    menu['1'] = "Get Angle."
    menu['2'] = "Set Angle."
    menu['3'] = "Get Posture"
    menu['4'] = "Execute Keyframe"
    menu['5'] = "Get Transform"
    menu['6'] = "Set Transform"
    menu['7'] = "list of methods"
    menu['8'] = "Exit"

    while True:
        options = menu.keys()
        for entry in options:
            print(entry, menu[entry])

        selection = input("Please Select:")
        print("-------RESULT-------")
        if selection == '1':
            print("get_angle")
            angle = agent.get_angle("HeadYaw")  # RHipYawPitch LLHipYawPitch
            print("getting angle for joint: HeadYaw", angle)
        elif selection == '2':
            print("set_angle")
            agent.set_angle("HeadYaw",-10)  # RHipYawPitch LLHipYawPitch
            print("setting angle for joint: HeadYaw")
        elif selection == '3':
            print("get_posture")
            posture = agent.get_posture()
            print("result posture:",posture)
        elif selection == '4':
            print("execute_keyframes")
            agent.execute_keyframes(hello())
        elif selection == '5':
            print("get_transform")
            result = agent.get_transform("HeadYaw")  # RHipYawPitch LLHipYawPitch
            print('result: \n', result)
        elif selection == '6':
            print("set_transform")
            T = identity(4)
            # row major
            T[-1, 1] = -0.30
            T[-1, 2] = -0.10
            print('Matrix set transform: \n', T)
            agent.set_transform("LArm",T)
        elif selection == '7':
            print("list of methods: \n",agent.rpcProxy.system.listMethods())
        elif selection == '8':
            print("break")
            break;
        else:
            print("Unknown Option Selected!")

        print("~#~#~#~#~#~~#~#~#~#~#~")

    print("~#~#~#~#~#~ Client END ~#~#~#~#~#~")
