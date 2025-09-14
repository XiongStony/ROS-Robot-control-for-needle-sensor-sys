import numpy as np
import math
# import modern_robotics as mr
# from niryo_robot_python_ros_wrapper import (NiryoRosWrapper, # pyright: ignore[reportMissingImports]
#                                             Pose,
#                                             ArmMoveCommand,
#                                             )
from pyniryo import (NiryoRobot,
                    PoseObject,
                    JointsPosition,
                    Command
                    )
class PynRobot(NiryoRobot):
    def letsmove(self,target:PoseObject,linear:bool = False,*args,**kwargs):
        msg = self.move(target,linear=linear,*args,**kwargs)
        print(f"Done: {msg}")
        return msg
    def insertionpose(self, prepose:PoseObject,length = 0.0, axis:str = 'x'):
        R_x = np.array([[1, 0, 0],
                        [0, math.cos(prepose.roll), -math.sin(prepose.roll)],
                        [0, math.sin(prepose.roll), math.cos(prepose.roll)]])
        
        R_y = np.array([[math.cos(prepose.pitch),0,math.sin(prepose.pitch)],
                       [0,1,0],
                       [-math.sin(prepose.pitch),0,math.cos(prepose.pitch)]])
        
        R_z = np.array([[math.cos(prepose.yaw),-math.sin(prepose.yaw),0],
                        [math.sin(prepose.yaw),math.cos(prepose.yaw),0],
                        [0, 0, 1]])
        R = R_z @ R_y @ R_x
        col_idx = {'x': 0, 'y': 1, 'z': 2}[axis]
        direction = R[:, col_idx] / np.linalg.norm(R[:, col_idx]) 
        dx, dy, dz = length*direction
        newpose = PoseObject(
            x = prepose.x + dx,
            y = prepose.y + dy,
            z = prepose.z + dz,
            roll=prepose.roll,
            pitch=prepose.pitch,
            yaw=prepose.yaw
            )
        return newpose
# class NiryoRobotWrapper(NiryoRosWrapper):
#     def letsmove(self,target:Pose,linear:bool = None,*args,**kwargs):
#         cmd = ArmMoveCommand.LINEAR_POSE if linear else None
#         code,msg = self.move(target,move_cmd=cmd,*args,**kwargs)
#         print(f"Done: {msg}")
#         return code
#     def insertionpose(self, prepose:Pose,length = 0.0, axis:str = 'x'):
#         R_x = np.array([[1, 0, 0],
#                         [0, math.cos(prepose.roll), -math.sin(prepose.roll)],
#                         [0, math.sin(prepose.roll), math.cos(prepose.roll)]])
        
#         R_y = np.array([[math.cos(prepose.pitch),0,math.sin(prepose.pitch)],
#                        [0,1,0],
#                        [-math.sin(prepose.pitch),0,math.cos(prepose.pitch)]])
        
#         R_z = np.array([[math.cos(prepose.yaw),-math.sin(prepose.yaw),0],
#                         [math.sin(prepose.yaw),math.cos(prepose.yaw),0],
#                         [0, 0, 1]])
#         R = R_z @ R_y @ R_x
#         col_idx = {'x': 0, 'y': 1, 'z': 2}[axis]
#         direction = R[:, col_idx] / np.linalg.norm(R[:, col_idx]) 
#         dx, dy, dz = length*direction
#         newpose = Pose(
#             x = prepose.x + dx,
#             y = prepose.y + dy,
#             z = prepose.z + dz,
#             roll=prepose.roll,
#             pitch=prepose.pitch,
#             yaw=prepose.yaw
#             )
#         return newpose

def fourD_invKinematics(xFinal, yFinal, zFinal, theta):
    ELBOW_TO_WRIST = 100     # L2 (mm)
    SHOULDER_TO_ELBOW = 100  # L1 (mm)
    SHOULDER_OFFSET = 35     # d  (mm)
    ANGLE_OFFSET = math.pi/2

    # 末端姿态（弧度）
    endEffector = math.radians(theta)

    # 等效第一段
    L1p   = math.hypot(SHOULDER_TO_ELBOW, SHOULDER_OFFSET)          # L1' = sqrt(L1^2 + d^2)
    alpha = math.atan2(SHOULDER_OFFSET, SHOULDER_TO_ELBOW)

    # 水平半径 ρ 与极角 γ
    rho   = math.hypot(xFinal, yFinal)                              # ρ = √(x^2 + y^2)
    gamma = math.atan2(zFinal, rho)                                 # γ = atan2(z, ρ)

    # 可达性检查
    if rho == 0 and zFinal == 0 and L1p == 0 and ELBOW_TO_WRIST == 0:
        raise ValueError("Degenerate geometry.")
    if rho > L1p + ELBOW_TO_WRIST or rho < abs(L1p - ELBOW_TO_WRIST):
        print("Target out of reach.")
        return None

    # 余弦定理 -> 肘角
    cosArg = (rho**2 + zFinal**2 - ELBOW_TO_WRIST**2 - L1p**2) / (2 * L1p * ELBOW_TO_WRIST)
    cosArg = max(-1.0, min(1.0, cosArg))
    elbowIK = math.acos(cosArg)                                     # [0, π]

    sinElbow = math.sin(elbowIK)
    cosElbow = math.cos(elbowIK)

    # 肩角两分支（肘下/肘上）
    denom = L1p + ELBOW_TO_WRIST * cosElbow
    shoulderIK1 = gamma - math.atan2(ELBOW_TO_WRIST * sinElbow,  denom) + alpha
    shoulderIK2 = gamma - math.atan2(-ELBOW_TO_WRIST * sinElbow, denom) + alpha

    # 选择分支（示例：选更大的那个，实际可按“离当前姿态最近”或指定肘形）
    shoulderIK = shoulderIK1 if shoulderIK1 >= shoulderIK2 else shoulderIK2

    # 按机械零位/方向做偏置映射（与你的硬件约定保持一致）
    shoulder = -(shoulderIK - ANGLE_OFFSET)
    elbow    =  elbowIK  - ANGLE_OFFSET
    wrist    =  endEffector - elbow - shoulder                      # 平面内姿态配平

    # 腰角：按常见世界系定义（如需与你现有代码兼容可改回去）
    waist    = math.atan2(yFinal, xFinal) + (ANGLE_OFFSET - math.pi/2)

    print(f"Waist angle: {waist}")
    print(f"Shoulder angle: {shoulder}")
    print(f"Elbow angle: {elbow}")
    print(f"Wrist angle: {wrist}")
    print("-----")

    return [waist, shoulder, elbow, wrist]
