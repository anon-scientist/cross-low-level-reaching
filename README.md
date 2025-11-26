# Low-Level Reaching Benchmark (LLR), part of the CRoSS benchmark suite
This repository contains the physically simulated variant of the Low-Level Reaching (LLR) environment from the CRoSS suite.
In this version, the 7-DoF robotic arm is simulated in Gazebo, including realistic joint limits, dynamics, and message-based control through the Gazebo Transport system.
Robot motion is executed by publishing joint commands and receiving sensor feedback through Gazebo’s message interface, providing a realistic model of latency, timing, and actuation, closely reflecting real-world robotic operation.
The environment preserves the same observation, action, and reward structure as the kinematic variant, but incorporates physical simulation effects such as velocity constraints and joint-limit enforcement.
Although slower than the kinematic version, the physical setup provides higher realism, full compatibility with Gazebo plugins and sensors, and can be connected to real hardware through a ROS-Gazebo bridge with only minimal modifications.
The corresponding kinematic version and additional benchmarks are linked from the main [CRoSS](https://github.com/anon-scientist/continual-robotic-simulation-suite/) repository.

*This repository is provided exclusively for anonymous review purposes.
To preserve anonymity during the submission process, the repository is maintained in read-only mode and does not accept issues, pull requests, or discussions at this stage.
Upon acceptance of the paper, we will make the full public version of this project available under our official organization account.*

## Overview
The Low-Level Reaching (LLR) benchmark evaluates continual reinforcement learning using joint-level control of a simulated 7-DoF robotic arm.
Instead of issuing Cartesian end-effector motions, the agent directly selects joint angle targets for each of the seven arm joints.
An episode consists of exactly seven steps, one for each joint in sequence, after which the final end-effector position is compared to the task’s goal position.

The action space is discretized per joint: for each joint, a fixed number of discrete angle targets is derived by evenly dividing that joint’s allowable motion range.
This creates a combinatorial action structure that requires effective exploration to discover good joint configurations leading to the desired end-effector pose.

This repository provides the physically simulated version of the LLR benchmark, where robot motion is computed via the Gazebo simulator.
This approach is more realistic, while preserving identical observations, actions, and reward definitions to the kinematic version.

The benchmark consists of eight tasks, each defined by a distinct 3D target position positioned around the robot at different heights and orientations.
The agent must learn joint configurations that place the end-effector at these targets, despite the sparse reward structure and the sequential joint-control constraint.

## Requirements
-   icrl repository
-   cl_experiment repository
-   Apptainer software
-   icrl/icrl.def for building the container

## How to Run
1. Clone this repository in a folder, where all other required repositories will be cloned into!!!
```bash
git clone https://github.com/anon-scientist/cross-kinematic-low-level-reaching
```
2. Clone both the icrl and cl_experiment repositories from the requirements above.
(Run these where you want them to end up in your file system)
``` bash
git clone https://github.com/anon-scientist/icrl
git clone https://github.com/anon-scientist/cl_experiment
```
3. Create the Apptainer container from the `icrl.def` file in the icrl repository.
```bash
cd icrl
apptainer build ../my_container.sif icrl.def
cd ..
```
4. (Optional) Modify the cmd line parameters in `main.bash` of this repository to what you want to test.
5. Run the `main.bash` by executing it with the apptainer image.
    * change "path_to_benchmark_repo" to the path where you cloned the repository into (including its name)
    * Make sure that the image has execute permission for the `main.bash`
```bash
apptainer exec my_container.sif path_to_benchmark_repo/main.bash
```
6. Look at the simulation results located in a new results folder inside the cloned repository

## Files


## Command Line Parameters
Here are some of the relevant hyperparameters the RL-Framework accepts. For more information look at the `main.bash` or the individual relevant python files.
```bash
--benchmark # The name for the benchmark.
--exp_id # Name of the experiment to use as an identifier for the generated result.
--root_dir # Directory where all experiment results and logs are stored.
--training_duration # Defines the number of iterations á training_duration_unit.
--evaluation_duration # Defines the number of iterations á evaluation_duration_unit.
--training_duration_unit # Sets the unit (abstraction level) t
--evaluation_duration_unit # Sets the unit (abstraction level) t
--max_steps_per_episode # Sets the number of steps after which an episode gets terminated.
--task_list # tasks to execute and in what order
--start_task # set the task from where to start the experiments
--eval_start_task # at what point evaluation should start being performed
--exploration_start_task # all other tasks before this are pure babbeling (random moves)
--training_duration_task_0 # first task duration (for babbeling phases)
--train_batch_size # Defines the mini-batch size that is used for training.
--algorithm # Defines the algorithm to use for the training
--goal_discrepency_threshold # max distance to goal that is considered reached
--action_speed # step-size for each cartesian action
--reward_skew # To give the reward non-linearity
--exploration # The exploration strategy the agent should use.
--replay_buffer # The type of buffer to use
--capacity # the replay buffer capacity
```

## Related Repositories
* [**CRoSS** - Entry Repository](https://github.com/anon-scientist/continual-robotic-simulation-suite/)
* [ICRL - RL-Framework Repository](https://github.com/anon-scientist/icrl/)
* [CL_Experiments - Utils Repository](https://github.com/anon-scientist/cl_experiments/)
* [LLR-K with Kinematic Simulation](https://github.com/anon-scientist/cross-kinematic-low-level-reaching)
