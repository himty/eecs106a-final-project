# Path Planning

# Pure trig version

No ROS dependencies!

## Running the system

To get next target point given current end effector position and command sphere location:
- Use `NextPointPlanner` in planner/next_pt_planner.py

To get forward/inverse kinematics
- Initialize `KinematicsCalculator()` from `planner/kinematics_calculator_trig.py`
- Call inverse_kinematics() or forward_kinematics() :D

# Moveit! IKFast version

## Installation to change the ikfast solver (NOT necessary for running the output of ikfast)

You probably DON'T have to install this stuff

```
sudo apt-get install liburdfdom-tools
sudo apt install ros-kinetic-moveit
sudo apt-get install ros-kinetic-franka-description
```

Also some more things from the ikfast tutorial http://docs.ros.org/en/kinetic/api/moveit_tutorials/html/doc/ikfast/ikfast_tutorial.html
```
sudo apt-get install cmake g++ git ipython minizip python-dev python-h5py python-numpy python-scipy qt4-dev-tools
sudo apt-get install libassimp-dev libavcodec-dev libavformat-dev libavformat-dev libboost-all-dev libboost-date-time-dev libbullet-dev libfaac-dev libglew-dev libgsm1-dev liblapack-dev liblog4cxx-dev libmpfr-dev libode-dev libogg-dev libpcrecpp0v5 libpcre3-dev libqhull-dev libqt4-dev libsoqt-dev-common libsoqt4-dev libswscale-dev libswscale-dev libvorbis-dev libx264-dev libxml2-dev libxvidcore-dev

sudo apt-get install libcairo2-dev libjasper-dev libpoppler-glib-dev libsdl2-dev libtiff5-dev libxrandr-dev
git clone https://github.com/openscenegraph/OpenSceneGraph.git --branch OpenSceneGraph-3.4
cd OpenSceneGraph
mkdir build; cd build
cmake .. -DDESIRED_QT_VERSION=4
make -j$(nproc)
sudo make install

pip install --upgrade --user sympy==0.7.1
sudo apt remove python-mpmath

sudo apt-get install ros-kinetic-moveit-kinematics
```

And install OpenRAVE
```
git clone --branch latest_stable https://github.com/rdiankov/openrave.git
cd openrave && mkdir build && cd build
cmake -DODE_USE_MULTITHREAD=ON -DOSG_DIR=/usr/local/lib64/ ..
make -j$(nproc)
sudo make install
```

Maybe for debugging
```
sudo apt-get install ros-kinetic-joint-state-publisher
sudo apt-get install ros-kinetic-joint-state-publisher-gui
```

#### URDF helpful commands

- `rosrun xacro xacro.py -o arm_bot.urdf arm_bot.xacro`- convert .xacro file to .urdf

- `rosrun collada_urdf urdf_to_collada arm_bot.urdf arm_bot.dae`- convert .urdf file to .dae file (Collada)

- `check_urdf arm_bot.urdf`- checks if urdf is formatted correctly

- `roslaunch moveit_setup_assistant setup_assistant.launch`- Start the MoveIt! Setup Assistant (requires xacro or urdf file as input)

- `rosrun moveit_commander moveit_commander_cmdline.py`- Start the MoveIt! Commander commandline tool for nice data http://docs.ros.org/en/kinetic/api/moveit_tutorials/html/doc/moveit_commander_scripting/moveit_commander_scripting_tutorial.html

- `openrave0.9.py --database inversekinematics --robot=arm_bot_wrapper.xml --iktype=translation3d --iktests=1000`- Create an IKFast solution. The XML wrapper is a workaround from https://answers.ros.org/question/263925/generating-an-ikfast-solution-for-4-dof-arm/ . Translation3D was picked from here http://openrave.org/docs/latest_stable/openravepy/ikfast/#ik-types . The file appears in \~/.openrave. I put this output into path_planning/urdf

- `rosrun moveit_kinematics create_ikfast_moveit_plugin.py arm_bot arm arm_bot_arm_kinematics `pwd`/kinematics.c7db6c67e555d156ee590ef3d3726851/ikfast0x10000049.Translation3D.0_1_2.cpp` - where the kinematics.c7db.. stuff is the output of the openrave command that was originally created in \~/.openrave. To create the ikfast moveit plugin package arm_bot_arm_kinematics for use by the semantic robot description (set up with setup_assistant above). Run catkin_make again to use this!

- `roslaunch arm_bot_moveit_config demo.launch`- Visualize the ikfast planner in RViz

### Bugfixes to the template code done along the way

- Follow https://answers.ros.org/question/342725/moveit-and-4-dof-arm/?answer=343951#post-id-343951 and implement the IKP_TranslationXAxisAngle4D, IKP_TranslationYAxisAngle4D, and IKP_TranslationZAxisAngle4D cases in arm_bot_arm_kinematics/src/arm_bot_arm_ikfast_moveit_plugin.cpp. Weird how it's not implemented when ikfast was run specifically for a 4DOF robot

- arm_bot_arm_kinematics from the create_ikfast_moveit_plugin.py run might be named incorrectly. Make sure the package name in CMakeLists.tst, package.xml, and the folder match with the Kinematic Solver under Planning Groups in the setup_assistant

## Running the system

To get next target point given current end effector position and command sphere location:
- Use `NextPointPlanner` in planner/next_pt_planner.py
- No roslaunches necessary

To get forward/inverse kinematics
- (One terminal) `roscore`
- (Another terminal) `roslaunch path_planning arm_bot_publish_moveit.launch`
- (Yet another terminal) Publish JointState messages to the `/joint_states` topic (for example, run `rosrun path_planning moveit_ik_test_pub.py`)
- Initialize `KinematicsCalculator('arm')` from `planner/kinematics_calculator_moveit.py`, where arm is the group name of the arm_bot's arm joints
- Call inverse_kinematics() or forward_kinematics() :D (may take about 0.2-0.3 seconds to return)

# tinyik version

This is the first iteration. Being a naive solver, each call for inverse kinematics took about 2 seconds, which is too long. This code is still in the `tinyik_files` folder for happy memories.

### Requirements
I'm pretty sure it's only `pip install tinyik open3d==0.10.0.0` (higher open3d versions give me a segmentation fault; open3d also doesn't support python >= 3.9)

Alternatively, cd to this folder and do `pip install -r requirements.txt` for extra fancy points


### Visualization

2D visualization looking down on the robot: `python vis/vis_planner_2d.py` (a z coordinate is hardcoded in the code)

3D visualization: `python vis/vis_planner_3d.py` (may take 2 seconds between actions because inverse kinematics runs slower on a multithreading thread :c)
- To make the first joint of the robot show up, you have to modify the tinyik source code (division by 0 D:)
- Go to somewhere like `/Users/himty/miniconda3/envs/proj106a/lib/python3.8/site-packages/tinyik/visualization.py` (see a root path by importing tinyik then printing `tinyik.__file__`)
- Replace `base = np.array([0., 0, norm])` on line 80 with `base = np.array([0., 1e-12, norm])`