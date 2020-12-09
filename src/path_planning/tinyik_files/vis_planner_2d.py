import matplotlib.pyplot as plt
import tinyik
import numpy as np
import sys
sys.path.insert(0, "/Users/himty/Desktop/Serious_Stuff/UC_Berkeley/3_Junior/EECS 106A/eecs106a-final-project/src/path_planning/moveit_planner")
from next_pt_planner import NextPointPlanner

def make_2Donclick(fig, ax, planner):
    # yeet higher order functions
    ee_pos = None
    obj_pos = None

    base_height = 7.5
    l1 = 10
    l2 = 12

    # 'z' and 'x' are rotation axes.
    # Each array is the length of the joint between rotation axes
    # I don't think this library supports prismatic joints? 
    arm = tinyik.Actuator([
            'z',
            [0, 0, base_height],
            'x',
            [0, l1, 0],
            'x',
            [0, l2, 0],
        ])

    pts = []

    def onclick(event):
        nonlocal ee_pos, obj_pos, pts

        x, y = event.xdata, event.ydata
        if ee_pos is None:
            # Hardcoded z coordinate
            ee_pos = np.array([x, y, 5])
            print('Got end effector pos:', ee_pos)

            pts.append(ax.scatter(x, y, label='End Effector'))

            plt.legend()
            plt.title('Next, click where the object should go')
            plt.pause(0.1)
        elif obj_pos is None:
            # Hardcoded z coordinate
            obj_pos = np.array([x, y, 5])
            print('Got obj pos:', obj_pos)

            pts.append(plt.scatter(x, y, label='Object'))
            plt.pause(0.1)

            near_pos = planner.get_near_point(ee_pos, obj_pos)
            pts.append(plt.scatter(*near_pos[:2], label='Near'))
            pts.append(plt.plot(*zip(ee_pos[:2], near_pos[:2]), linestyle=':')[0])
            print("Near pos:", near_pos)
            arm.ee = near_pos
            print("Near pos's joint angles:", arm.angles)

            far_pos = planner.get_far_point(ee_pos, obj_pos)
            pts.append(plt.scatter(*far_pos[:2], label='Far'))
            pts.append(plt.plot(*zip(ee_pos[:2], far_pos[:2]), linestyle=':')[0])
            print("Far pos:", far_pos)
            arm.ee = far_pos
            print("Far pos's joint angles:", arm.angles)

            plt.legend()
            plt.title('Plotted planner points. Click the figure again to clear the screen')
            plt.pause(0.1)
        else:
            [p.remove() for p in pts]
            pts = []
            plt.legend()
            plt.title('Click where the end effector should go')
            plt.pause(0.1)

            ee_pos = None
            obj_pos = None

            print('Cleared.')
            print('\n---------------------\n')
    return onclick

if __name__ == "__main__":
    planner = NextPointPlanner(2, [])

    fig,ax = plt.subplots()
    ax.set_xlim([-30, 30])
    ax.set_ylim([-30, 30])
    fig.canvas.mpl_connect('button_press_event', make_2Donclick(fig, ax, planner))

    plt.title('Click where the end effector should go')
    plt.scatter(0, 0, label='Origin')
    plt.legend()
    plt.show()