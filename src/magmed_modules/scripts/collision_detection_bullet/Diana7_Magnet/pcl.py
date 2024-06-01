import fcl
import numpy as np
from urdf_parser_py.urdf import URDF

# 加载URDF模型
robot = URDF.from_xml_file("robot.urdf")

# 创建fcl的碰撞对象
collision_objects = {}
for link in robot.links:
    if link.collision:
        # 这里假设你有一个将urdf的几何体转换为fcl的几何体的函数
        geom = urdfGeometryToFCLGeometry(link.collision.geometry)
        trans = fcl.Transform3f()
        obj = fcl.CollisionObject(geom, trans)
        collision_objects[link.name] = obj

# 计算末端执行器和机器人其他部分的最小距离
request = fcl.DistanceRequest(enable_nearest_points=True)
result = fcl.DistanceResult()
min_distance = np.inf
for name, obj in collision_objects.items():
    if name != "end_effector_link":
        fcl.distance(collision_objects["end_effector_link"], obj, request, result)
        if result.min_distance < min_distance:
            min_distance = result.min_distance

# 输出最小距离
print("The minimum distance is:", min_distance)
