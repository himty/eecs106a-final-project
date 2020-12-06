# Path Planning

## Moveit! IKFast

## Requirements

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

- `openrave0.9.py --database inversekinematics --robot=arm_bot_wrapper.xml --iktype=translationzaxisangle4d --iktests=1000`- Create an IKFast solution. The XML wrapper is a solution from https://answers.ros.org/question/263925/generating-an-ikfast-solution-for-4-dof-arm/ and the type of solution translationzaxisangle4d, which is picked from here http://openrave.org/docs/latest_stable/openravepy/ikfast/#ik-types . The file appears in \~/.openrave

- `roslaunch arm_bot_moveit_config demo.launch rviz_tutorial:=true`- Visualize the ikfast planner in RViz

## tinyik

### Requirements
I'm pretty sure it's only `pip install tinyik open3d==0.10.0.0` (higher open3d versions give me a segmentation fault; open3d also doesn't support python >= 3.9)

Alternatively, cd to this folder and do `pip install -r requirements.txt` for extra fancy points



### Visualization

2D visualization looking down on the robot: `python vis_planner_2d.py` (a z coordinate is hardcoded in the code)

3D visualization: `python vis_planner_3d.py` (may take 2 seconds between actions because inverse kinematics runs slower on a multithreading thread :c)
- To make the first joint of the robot show up, you have to modify the tinyik source code (division by 0 D:)
- Go to somewhere like `/Users/himty/miniconda3/envs/proj106a/lib/python3.8/site-packages/tinyik/visualization.py` (see a root path by importing tinyik then printing `tinyik.__file__`)
- Replace `base = np.array([0., 0, norm])` on line 80 with `base = np.array([0., 1e-12, norm])`