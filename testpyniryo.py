from pyniryo import NiryoRobot, PoseObject, JointsPosition
from time import sleep
from importlib.metadata import version
import math
from robotctl import PynRobot
def main():
    BOT_IP = '169.254.200.200'
    try:
        bot = PynRobot(BOT_IP)
        bot.calibrate_auto()
        print('done')
        target = PoseObject(x = 0.2, y = 0.0, z=0.3,
                            roll = 0.0,
                            pitch=0,
                            yaw=0)
        insertionP = bot.insertionpose(target,0.2)
        print(target)
        print(insertionP)
        for i in range(1000):
            bot.letsmove(target,linear=True)
            bot.letsmove(insertionP,linear=True)
            print(f'loop = {i}')

        sleep(1)
    except Exception as e:
        print(e)
    finally:
        bot.go_to_sleep()
        bot.close_connection()
        print('done')

if __name__ == "__main__":
    main()