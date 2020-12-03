from path_planner import AlgPathPlanner
import matplotlib.pyplot as plt
import numpy as np

def make_2Donclick(fig, ax, planner):
    # yeet higher order functions
    ee_pos = None
    obj_pos = None

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
            print("Near pos's joint angles:", planner.inverse_kinematics(near_pos))

            far_pos = planner.get_far_point(ee_pos, obj_pos)
            pts.append(plt.scatter(*far_pos[:2], label='Far'))
            pts.append(plt.plot(*zip(ee_pos[:2], far_pos[:2]), linestyle=':')[0])
            print("Far pos:", far_pos)
            print("Far pos's joint angles:", planner.inverse_kinematics(far_pos))

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
    planner = AlgPathPlanner(2, [])

    fig,ax = plt.subplots()
    ax.set_xlim([-10, 10])
    ax.set_ylim([-10, 10])
    fig.canvas.mpl_connect('button_press_event', make_2Donclick(fig, ax, planner))

    plt.title('Click where the end effector should go')
    plt.scatter(0, 0, label='Origin')
    plt.legend()
    plt.show()