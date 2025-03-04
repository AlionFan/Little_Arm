import pybullet as p
import time
import pybullet_data

# 连接到物理引擎（使用 GUI 显示）
physicsClient = p.connect(p.GUI)  # 或者 p.DIRECT 用于非图形化版本

# 设置数据路径
p.setAdditionalSearchPath(pybullet_data.getDataPath())  # 可选

# 设置重力
p.setGravity(0, 0, -10)

# 调整相机视角
p.resetDebugVisualizerCamera(
    cameraDistance=0.5,  # 相机距离
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
                   flags=p.URDF_USE_INERTIA_FROM_FILE)

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
targetPositions = [0, 0, 0]  # 初始目标位置
jointForces = [5, 3, 1]  # 关节最大力矩


# 创建控制条（滑块）
sliders = []
for i, jointIndex in enumerate(jointIndices):
    slider = p.addUserDebugParameter(
        paramName=f"Joint {jointIndex}",  # 控制条名称
        rangeMin=-3.14,  # 最小值（-π）
        rangeMax=3.14,   # 最大值（π）
        startValue=0     # 初始值
    )
    sliders.append(slider)

# 运行仿真
while True:
    # 读取控制条的值并设置关节角度
    for i, jointIndex in enumerate(jointIndices):
        targetPosition = p.readUserDebugParameter(sliders[i])  # 读取控制条的值
        p.setJointMotorControl2(
            bodyUniqueId=robotId,
            jointIndex=jointIndex,
            controlMode=p.POSITION_CONTROL,
            targetPosition=targetPosition,
            force=jointForces[i]  # 关节最大力矩
        )

    # 步进仿真
    p.stepSimulation()
    time.sleep(1. / 240.)  # 控制仿真步长
