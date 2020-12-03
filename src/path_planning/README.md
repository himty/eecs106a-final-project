# Path Planning

### Requirements
I'm pretty sure it's only `pip install tinyik open3d==0.10.0.0` (higher open3d versions give me a segmentation fault; open3d also doesn't support python >= 3.9)

Alternatively, cd to this folder and do `pip install -r requirements.txt` for extra fancy points

### Visualization

2D visualization looking down on the robot: `python vis_planner_2d.py` (a z coordinate is hardcoded in the code)

3D visualization: `python vis_planner_3d.py` (may take 2 seconds between actions because inverse kinematics runs slower on a multithreading thread :c)
- To make the first joint of the robot show up, you have to modify the tinyik source code (division by 0 D:)
- Go to somewhere like `/Users/himty/miniconda3/envs/proj106a/lib/python3.8/site-packages/tinyik/visualization.py` (see a root path by importing tinyik then printing `tinyik.__file__`)
- Replace `base = np.array([0., 0, norm])` on line 80 with `base = np.array([0., 1e-12, norm])`