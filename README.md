# F1Tenth_Hackathon_Evaluation
This is a ROS2 package that contains the evaluation scripts needed for the F1Tenth Hackathon

## Notes
- **For evaluating your algorithms, please ensure:**
    1. Add your algorithm file in the `F1Tenth_Hackathon_Evaluation/evaluation` directory 
    2. Add your node executable to the console_scripts list in `F1Tenth_Hackathon_Evaluation/setup.py`
    3. In `F1Tenth_Hackathon_Evaluation/launch/test_eval_launch.py`, replace `wall_follower` with the executable name you just defined in setup.py
    4. Under your workspace `src/` directory, `colcon build` and source `install/local_setup.bash`
- **The evaluation script will only run for 10 minutes and will print the final results and kill the node.**
- **Make sure the f1tenth simulator is running before launching the nodes**
- **To visualise the lap_marker, in RViz click "Add" > click on the "By topic" tab > click on the topic "`/lap_marker`" > click "Ok"**
  
## Installation
> NOTE: Run the following commands in the docker container which contains the F1Tenth Simulator.
Navigate to your workspace `src/` directory for example:
```sh
cd ros2_ws/src/
```
Clone the repository
```sh
git clone https://github.com/NTU-Autonomous-Racing-Team/F1Tenth_Hackathon_Evaluation.git
```
Navigate to the root of your workspace and build the workspace
```sh
cd ros2_ws/
colcon build
```
Source the installation
```sh
source install/local_setup.bash
```
Run the launch file
```sh
ros2 launch evaluation test_eval_launch.py
```
