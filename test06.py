import numpy as np
from niryo_robot_python_ros_wrapper import (NiryoRosWrapper,
    JointsPosition,
    Pose,
    ArmMoveCommand
)
import math
from Robotctl import NiryoRobot
from time import sleep
from importlib.metadata import version
def main():
    try:
        print(version("niryo_robot_python_ros_wrapper"))
        robot = NiryoRobot.init_with_node()
        robot.calibrate_auto()
        print("Calibrated")
        robot.move(JointsPosition(0,0,0,0,0,0))
        print('done')
        target = Pose(x = 0.0, y = 0.1, z=0.5, roll = 0, pitch=math.pi/6, yaw=math.pi/6)
        print(f'pose = {target}')
        newpose = robot.insertionpose(prepose = target, length = 0.3)
        print(f'newpose = {newpose}')
        res = robot.letsmove(target)
        print(res)
        res = robot.letsmove(newpose,linear=True)
        print(res)
        sleep(1)
        for i in range(6):
            robot.letsmove(target,linear=True)
            robot.letsmove(newpose,linear=True)
    except Exception as e:
        print(e)
    finally:
        robot.move_to_sleep_pose()
        print('done')

if __name__ == "__main__":
    main()