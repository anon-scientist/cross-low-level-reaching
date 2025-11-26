"""
CRoSS benchmark suite, LLR  benchmark. Example for standalone use of provided
robot manager, without RL framework
"""
from gazebo_sim.simulation.PandaRobot import PandaRobot
from src.cross_low_level_reaching.EnvironmentManager import RobotArmEnvironmentManager
end=False

class RobotAction():
    def __init__(self,label,amount):
        self.label = label
        self.amount = amount

class Wrapper():
    def __init__(self,robot):
        self.robot = robot

actions = []
actions.append(RobotAction("1",0.1))
actions.append(RobotAction("2",0.2))
actions.append(RobotAction("3",0.5))
actions.append(RobotAction("4",0.7))
actions.append(RobotAction("5",0.9))

wrapper = Wrapper(PandaRobot(actions))

manager = RobotArmEnvironmentManager(wrapper)
manager.perform_switch(0)
iter = 0
while not end:
    joint = iter % 7
    if joint == 0:
        print(f"Current Pos: {manager.get_data().hand.position}. RESETTING ...")
        manager.perform_reset()
        
    userin = input(f"Move joint_{joint} with [12345]: ")
    print("Please wait for the action to be published!")
    if userin == "e" or userin == "exit":
        end = True
        break;
    elif int(userin) > 0 and int(userin)<=len(actions):
        manager.gz_perform_action(actions[int(userin)-1])
    else:
        print("Illegal Input. To exit write 'e' or 'exit'")
        continue
    iter+=1

