from src.cross_low_level_reaching.EnvironmentWrapper import RobotArmWrapper

args_dict = {"task_list":["right_up","right_down"],
             "use_coords_in_obs":"yes"}


# instantiate environment
# compute nsec delay between two observations
# complicated by the fact that gazebo computes step durations only to msec precision
# so a frame rate of 30 means that the delay between two frames is 33msec, but not 33.3333 msec
# so we have to round down if we want to work with nsec delays     
hz = 30. ; # we have to know this, definedi n the robot sdf file, camera sensor plugin
nsec_per_frame = int(1000./hz) * 1000000. ;
nsec = nsec_per_frame * (hz / 15) ; # obs_per_sec_sim_time = 15

env = RobotArmWrapper(nsec,**args_dict) ;

print("Define a task") ;
env.switch(0) ;

end = False
iter = 0
while not end:
    joint=iter%7
    if joint == 0: env.reset()
    action_index = 2 # For the possible actions look in Environment.py
    obs, reward, terminated, truncated, info = env.step(action_index)
    print(f"JOINT_{joint} with action {action_index} -> State: {obs}")

    if joint == 6:
        print("Reward:", reward)

    if iter >= 13:
        end=True
        break
    iter+=1
