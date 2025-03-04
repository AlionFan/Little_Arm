import numpy as np
import open3d as o3d

# 读取保存的点云数据
output_file = "end_effector_positions.npy"
point_cloud_data = []

# 以追加模式读取文件
with open(output_file, "rb") as f:
    while True:
        try:
            batch = np.load(f)
            point_cloud_data.append(batch)
        except Exception as e:
            break

# 合并所有批次的点云数据
point_cloud_data = np.concatenate(point_cloud_data, axis=0)

# 创建点云对象
point_cloud = o3d.geometry.PointCloud()
point_cloud.points = o3d.utility.Vector3dVector(point_cloud_data)

# 可视化点云
o3d.visualization.draw_geometries([point_cloud])
