import time ;
import numpy as np ;

from typing import * ;
from threading import Lock ;
from scipy.spatial.transform import Rotation

import gz.transport13 as gz
from gz.msgs10.double_pb2 import Double
from gz.msgs10.model_pb2 import Model
from gz.transport13 import Node ;
from gz.msgs10.empty_pb2 import Empty ;
from gz.msgs10.scene_pb2 import Scene ;
from gz.msgs10.world_control_pb2 import WorldControl ;
from gz.msgs10.boolean_pb2 import Boolean ;
from gz.msgs10.pose_v_pb2 import Pose_V ;

class RobotArmEnvironmentManager(Node):
    def __init__(self,env_wrapper,**kwargs):
        self.init_node()
        self.mutex = Lock()

        self.env_wrapper = env_wrapper
        self.world_name='/world/robot-arm'
        self.robot_name='/model/panda_arm'

        self.step = 0
        self.current_joint = self.env_wrapper.robot.joint_amount
        self.task_milestone = 0
        self.last_obs_time = 0

        if self.subscribe(Pose_V,f'{self.world_name}/dynamic_pose/info',self.gz_handle_observation_callback):
            print("Subscribed to dynamic_pose/info!")

        if self.subscribe(Model,f'{self.robot_name}/joints/state',self.gz_handle_joint_states_callback):
            print("Subscribed to joints/state!")

        self.gz_actions = {}
        index = 1
        for joint in self.env_wrapper.robot.joints:
            joint_name = f"joint{index}"
            index +=1
            self.gz_actions[joint_name] = self.advertise(f'{self.robot_name}/joint/panda_{joint_name}/0/cmd_pos',Double)

        self.wait_for_simulation()

        self.start = [self.env_wrapper.robot.hand.position.x,self.env_wrapper.robot.hand.position.y,self.env_wrapper.robot.hand.position.z]

        self.world_control_service = f'{self.world_name}/control'
        self.res_req = WorldControl()

    def init_node(self):
        super().__init__()

    def wait_for_simulation(self):
        #print("WAITING FOR THE SIMULATION! THIS IS A WORKAROUND SINCE THE PANDA ROBOT BREAKS THE SCENE/INFO SERVICE!")
        time.sleep(5)
        return

    def request_scene(self):
        result = False;
        start_time = time.time()
        #print(f'Waiting for {self.world_name}/scene/info ...')
        while result is False:
            # Request the scene information
            result, response = self.request(f'{self.world_name}/scene/info', Empty(), Empty, Scene, 1)
            print(f'\rWaiting for {self.world_name}/scene/info ... {(time.time() - start_time):.2f} sec', end='')
            time.sleep(0.1)
        print('\nScene received!')
        return response

    def get_step(self):
        return self.step

    def get_data(self):
        return self.env_wrapper.robot

    def get_position(self):
        return self.env_wrapper.robot.hand.position
    
    def get_orientation(self):
        return self.env_wrapper.robot.hand.rotation
    
    def get_orientation_euler(self):
        return Rotation.from_quat(self.env_wrapper.robot.hand.rotation)

    def get_last_obs_time(self):
        return self.last_obs_time

    def gz_handle_observation_callback(self,msg):
        with self.mutex:
            self.env_wrapper.robot.hand_callback(msg)
            self.last_obs_time = msg.header.stamp.sec * 1000000000 + msg.header.stamp.nsec

    def gz_handle_joint_states_callback(self,msg):
        with self.mutex:
            self.env_wrapper.robot.joints_callback(msg)

    def gz_perform_action(self, action):
        with self.mutex:
            self.step += 1
            self.current_joint -= 1
            
            joint_name = f"joint{self.current_joint+1}"
            self.perform_joint_rotation(joint_name,self.env_wrapper.robot.get_joint_rotation_with_num(self.current_joint,action.amount))
            print(f'Action published: ', action.label)
        if self.current_joint == 0:
            time.sleep(2.0) # WAIT FOR THE FINAL ROTATION
        else:
            time.sleep(0.2) # WAIT FOR ANY OTHER ROTATION (small wait might result in inaccurate positional data)

    def perform_switch(self, task_index:str):
        pass

    def perform_reset(self):
        print("\n/* --------------- Reset Robot Joints --------------- */\n")
        with self.mutex:
            for joint_name in self.gz_actions.keys():
                self.perform_joint_rotation(joint_name, 0.0)
        time.sleep(1.0)
        self.current_joint = self.env_wrapper.robot.joint_amount
        print("\n/* -------------------------------------------------- */\n")

    def perform_joint_rotation(self,joint_name,target_rotation):
        message = Double()
        message.data = target_rotation
        success = self.gz_actions[joint_name].publish(message)
        if not success and self.env_config.debug:
            print(f"Error sending message to {joint_name}!!!!")

    def trigger_pause(self, pause):
        raise Exception("THIS DOES NOT WORK IN THIS SCENARIO! ALL SERVICES ARE BROKEN WHEN THE PANDA ROBOT IS IN THE SCENE.")
