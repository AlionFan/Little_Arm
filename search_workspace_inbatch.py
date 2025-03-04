import pybullet as p
import time
import pybullet_data
import numpy as np

# 连接到物理引擎（使用 GUI 显示）
physicsClient = p.connect(p.GUI)  # 或者 p.DIRECT 用于非图形化版本

# 设置数据路径
p.setAdditionalSearchPath(pybullet_data.getDataPath())  # 可选

# 设置重力
p.setGravity(0, 0, -10)

# 调整相机视角
p.resetDebugVisualizerCamera(
    cameraDistance=1.5,  # 相机距离
    cameraYaw=45,        # 相机偏航角
    cameraPitch=-30,     # 相机俯仰角
    cameraTargetPosition=[0, 0, 0]  # 相机目标点
)

# 加载地面模型
planeId = p.loadURDF("plane.urdf")

# 定义机器人初始位置和姿态
cubeStartPos = [0, 0, 0]
cubeStartOrientation = p.getQuaternionFromEuler([0, 0, 0])

# 加载机器人模型
robotId = p.loadURDF("tourdf.urdf", cubeStartPos, cubeStartOrientation, 
                   flags=p.URDF_USE_INERTIA_FROM_FILE,
                     useFixedBase=True)

# 获取关节信息
numJoints = p.getNumJoints(robotId)
jointIndices = []
for i in range(numJoints):
    jointInfo = p.getJointInfo(robotId, i)
    jointName = jointInfo[1].decode("utf-8")  # 获取关节名称
    jointType = jointInfo[2]  # 获取关节类型
    if jointType == p.JOINT_REVOLUTE:  # 只处理旋转关节
        jointIndices.append(i)
        print(f"Joint {i}: {jointName}")

# 设置关节控制参数
jointForces = [5, 3, 1]  # 关节最大力矩

# 定义关节角度范围和步长
angle_range = np.arange(-np.pi, np.pi, 0.2)  # 减少采样步长

# 存储末端执行器位置
end_effector_positions = []

# 遍历关节角度组合
for angle1 in angle_range:
    for angle2 in angle_range:
        for angle3 in angle_range:
            # 设置关节角度
            p.setJointMotorControl2(
                bodyUniqueId=robotId,
                jointIndex=jointIndices[0],
                controlMode=p.POSITION_CONTROL,
                targetPosition=angle1,
                force=jointForces[0]
            )
            p.setJointMotorControl2(
                bodyUniqueId=robotId,
                jointIndex=jointIndices[1],
                controlMode=p.POSITION_CONTROL,
                targetPosition=angle2,
                force=jointForces[1]
            )
            p.setJointMotorControl2(
                bodyUniqueId=robotId,
                jointIndex=jointIndices[2],
                controlMode=p.POSITION_CONTROL,
                targetPosition=angle3,
                force=jointForces[2]
            )

            # 步进仿真
            p.stepSimulation()

            # 获取末端执行器位置
            end_effector_state = p.getLinkState(robotId, jointIndices[-1])
            end_effector_position = end_effector_state[0]  # 位置信息
            end_effector_positions.append(end_effector_position)

# 将末端执行器位置转换为 NumPy 数组
end_effector_positions = np.array(end_effector_positions)

# 分批绘制点
batch_size = 65535  # 每批最多 65535 个点
point_color = [1, 0, 0]  # 红色
point_size = 2  # 点的大小

for i in range(0, len(end_effector_positions), batch_size):
    batch = end_effector_positions[i:i + batch_size]
    p.addUserDebugPoints(
        pointPositions=batch,
        pointColorsRGB=[point_color] * len(batch),
        pointSize=point_size
    )

# 运行仿真
while True:
    p.stepSimulation()
    time.sleep(1. / 240.)  # 控制仿真步长
