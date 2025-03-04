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

# 定义随机采样参数
num_samples = 100000  # 总采样点数
batch_size = 10000    # 每批最多 10000 个点
output_file = "end_effector_positions.npy"  # 保存点云数据的文件

# 存储末端执行器位置
end_effector_positions = []

# 随机采样关节角度并计算末端执行器位置
for _ in range(num_samples):
    # 随机生成关节角度
    angles = np.random.uniform(-np.pi, np.pi, len(jointIndices))
    
    # 设置关节角度
    for i, jointIndex in enumerate(jointIndices):
        p.setJointMotorControl2(
            bodyUniqueId=robotId,
            jointIndex=jointIndex,
            controlMode=p.POSITION_CONTROL,
            targetPosition=angles[i],
            force=jointForces[i]
        )
    
    # 步进仿真
    p.stepSimulation()
    
    # 获取末端执行器位置
    end_effector_state = p.getLinkState(robotId, jointIndices[-1])
    end_effector_position = end_effector_state[0]  # 位置信息
    end_effector_positions.append(end_effector_position)

    # 分批保存点云数据
    if len(end_effector_positions) >= batch_size:
        # 将当前批次的点保存到文件
        with open(output_file, "ab") as f:  # 以追加模式打开文件
            np.save(f, np.array(end_effector_positions))
        end_effector_positions = []  # 清空当前批次

# 保存剩余的点
if end_effector_positions:
    with open(output_file, "ab") as f:
        np.save(f, np.array(end_effector_positions))

# 断开连接
p.disconnect()

print(f"点云数据已保存到文件: {output_file}")
